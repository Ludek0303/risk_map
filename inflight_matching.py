"""Durable feature/pair cache and a separate low-priority in-flight worker."""

import argparse
from collections import OrderedDict, deque
import hashlib
import io
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import time

import cv2
import numpy as np


def pack(**arrays):
    buffer = io.BytesIO()
    np.savez(buffer, **arrays)
    return buffer.getvalue()


class IncrementalCache:
    def __init__(self, path, profile="none", crop=0.8):
        self.profile, self.crop = profile, crop
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path), timeout=30)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS features (key TEXT PRIMARY KEY, data BLOB NOT NULL);
            CREATE TABLE IF NOT EXISTS pairs (
                a TEXT, b TEXT, mode TEXT, data BLOB,
                PRIMARY KEY (a,b,mode));
        """)
        self.memory = OrderedDict()
        self.feature_hits = self.pair_hits = 0

    def close(self):
        self.db.close()
        self.memory.clear()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def key(self, path):
        path = Path(path).resolve()
        stat = path.stat()
        signature = [str(path), stat.st_size, stat.st_mtime_ns,
                     self.profile, self.crop, "sift3500-v1"]
        return hashlib.sha256(json.dumps(signature).encode()).hexdigest()

    def feature(self, path):
        from orthomosaic import normalization, project, read_frame
        key = self.key(path)
        row = self.db.execute("SELECT data FROM features WHERE key=?", (key,)).fetchone()
        if row is None:
            frame = read_frame(path, self.profile, self.crop)
            detector = cv2.SIFT_create(nfeatures=3500, contrastThreshold=0.025)
            keys, descriptors = detector.detectAndCompute(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), None)
            points = np.array([k.pt for k in keys], np.float64).reshape(-1, 2)
            points = project(normalization(frame.shape), points)
            if descriptors is None:
                descriptors = np.empty((0, 128), np.float32)
            blob = pack(shape=frame.shape[:2], points=points, descriptors=descriptors)
            with self.db:
                self.db.execute("INSERT OR REPLACE INTO features VALUES (?,?)", (key, blob))
        else:
            self.feature_hits += 1
        shape, points, descriptors = self.load(key)
        return key, shape, (points, descriptors)

    def load(self, key):
        if key not in self.memory:
            row = self.db.execute("SELECT data FROM features WHERE key=?", (key,)).fetchone()
            if row is None:
                raise KeyError(key)
            with np.load(io.BytesIO(row[0]), allow_pickle=False) as data:
                self.memory[key] = (tuple(data["shape"]), data["points"].copy(), data["descriptors"].copy())
            while len(self.memory) > 12:
                self.memory.popitem(last=False)
        self.memory.move_to_end(key)
        return self.memory[key]

    def pair(self, a, b, fast=False, features=None):
        from orthomosaic import match_features
        mode = "flann96-v1" if fast else "bf-v1"
        row = self.db.execute("SELECT data FROM pairs WHERE a=? AND b=? AND mode=?", (a, b, mode)).fetchone()
        if row is not None:
            self.pair_hits += 1
            if row[0] is None:
                return None  # Failed matches are cached too.
            with np.load(io.BytesIO(row[0]), allow_pickle=False) as data:
                return data["hom"].copy(), data["src"].copy(), data["dst"].copy()
        if features is None:
            features = (self.load(a)[1:], self.load(b)[1:])
        result = match_features(*features, fast=fast)
        blob = None if result is None else pack(hom=result[0], src=result[1], dst=result[2])
        with self.db:
            self.db.execute("INSERT OR REPLACE INTO pairs VALUES (?,?,?,?)", (a, b, mode, blob))
        return result


class InflightMatcher:
    """Capture writes tiny job records; image computation runs outside its process."""
    def __init__(self, session, fast=False):
        self.session = Path(session).resolve()
        self.manifest = self.session / "inflight_jobs.jsonl"
        self.stop = self.session / "inflight.stop"
        self.process = None
        self.log = None
        self.fast = fast

    def submit(self, path, lat, lon, altitude):
        # Start only after the first clean JPEG has been completely written.
        if self.process is None:
            self.stop.unlink(missing_ok=True)
            self.log = (self.session / "inflight_matching.log").open("a")
            env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
            self.process = subprocess.Popen([
                sys.executable, "-u", str(Path(__file__).resolve()), str(self.session),
                "--parent-pid", str(os.getpid()),
                *(["--fast"] if self.fast else []),
            ], stdout=self.log, stderr=subprocess.STDOUT, env=env)
            print("\nIn-flight matching started (see inflight_matching.log)", flush=True)
        if self.process.poll() is not None:
            raise RuntimeError("In-flight worker stopped; see inflight_matching.log")
        with self.manifest.open("a") as handle:
            handle.write(json.dumps([str(Path(path).resolve()), lat, lon, altitude])+"\n")

    def close(self):
        if self.process is None:
            if self.log is not None:
                self.log.close()
                self.log = None
            return
        self.stop.touch()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        if self.log is not None:
            self.log.close()
            self.log = None
        self.process = None


def run_worker(session, fast=False, parent_pid=None):
    session = Path(session)
    cv2.setNumThreads(2)
    if hasattr(os, "nice"):
        os.nice(5)
    manifest = session / "inflight_jobs.jsonl"
    stop = session / "inflight.stop"
    offset = 0
    records, keys, centers = [], [], []
    pending, scheduled = deque(), set()
    completed = accepted = 0
    started = time.monotonic()
    with IncrementalCache(session / ".inflight_matches.sqlite3") as cache:
        while not stop.exists():
            if parent_pid is not None and os.getppid() != parent_pid:
                break
            # Give newly arrived pictures priority over matching the backlog.
            line = ""
            if manifest.exists():
                with manifest.open() as handle:
                    handle.seek(offset)
                    line = handle.readline()
                    if line.endswith("\n"):
                        offset = handle.tell()
                    else:
                        line = ""
            if line:
                path, lat, lon, altitude = json.loads(line)
                key, _, _ = cache.feature(Path(path))
                i = len(records)
                records.append((path, lat, lon, altitude))
                keys.append(key)
                ref_lat, ref_lon = records[0][1:3]
                centers.append(((lon-ref_lon)*111320*np.cos(np.radians(ref_lat)), (lat-ref_lat)*111320))
                xy = np.array(centers)
                distances = np.linalg.norm(xy-xy[-1], axis=1)
                neighbors = 12 if fast else 25
                candidates = set(int(j) for j in np.argsort(distances[:i])[:neighbors] if distances[j] < 120)
                for step in ((1, 2, 3, 5) if fast else (1, 2, 3, 5, 8, 15)):
                    if i >= step:
                        candidates.add(i-step)
                # Also cover earlier cameras whose nearest-neighbor set gains this picture.
                for j in range(i):
                    if distances[j] < 120:
                        previous = np.linalg.norm(xy[:-1]-xy[j], axis=1)
                        if np.count_nonzero(previous < distances[j])-1 < neighbors:
                            candidates.add(j)
                for j in sorted(candidates):
                    pair = (j, i)
                    if pair not in scheduled:
                        scheduled.add(pair)
                        pending.append(pair)
                print(f"Features ready: {i+1}; queued pairs: {len(pending)}", flush=True)
            elif pending:
                i, j = pending.popleft()
                accepted += cache.pair(keys[i], keys[j], fast=fast) is not None
                completed += 1
                if completed % 50 == 0:
                    print(f"Pairs ready: {completed}; accepted: {accepted}; queued: {len(pending)}", flush=True)
            else:
                time.sleep(0.1)
        report = {"features_ready": len(records), "pairs_ready": completed,
                  "accepted_pairs": accepted, "pending_pairs": len(pending),
                  "elapsed_seconds": time.monotonic()-started, "fast": fast}
        (session / "inflight_report.json").write_text(json.dumps(report, indent=2)+"\n")
        print("In-flight worker stopped; completed work is saved for final reconstruction", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--fast", action="store_true")
    parser.add_argument("--parent-pid", type=int)
    args = parser.parse_args()
    run_worker(args.session, args.fast, args.parent_pid)
