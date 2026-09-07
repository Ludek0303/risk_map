"""Gimbal control for a PX4 SITL vehicle over MAVLink (pymavlink).

Points the simulated camera gimbal (e.g. the cgo3 gimbal on x500_gimbal) to
an absolute pitch/yaw - most commonly straight down (nadir) for mapping.

Publishing directly to the Gazebo joint command topics
(``/model/.../command/gimbal_pitch``) does not work for this: PX4's own
gimbal driver continuously republishes its own setpoint to those topics, so
an external one-shot message gets overwritten within one control cycle.
The gimbal has to be commanded the way PX4 expects - via the MAVLink gimbal
manager protocol (MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE to claim control, then
MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW to set the setpoint).
"""

from __future__ import annotations

from typing import Union

from pymavlink import mavutil

# PX4 SITL's companion-computer MAVLink channel: PX4 sends *to* this port,
# so we bind/listen on it (a plain "udpout" connect never sees a heartbeat).
DEFAULT_CONNECTION = "udpin:127.0.0.1:14540"

MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001


class GimbalController:
    """Points a PX4-controlled gimbal via MAVLink gimbal manager commands."""

    def __init__(
        self,
        connection: Union[str, "mavutil.mavfile"] = DEFAULT_CONNECTION,
        target_system: int = 1,
        target_component: int = 1,
        heartbeat_timeout: float = 10.0,
    ) -> None:
        """connection: a MAVLink connection string to open, or an already-
        connected `mavutil.mavlink_connection(...)` object to reuse (e.g.
        when a caller also needs that same socket for telemetry - two
        separate connections can't both bind the same local UDP port).
        """
        self._target_system = target_system
        self._target_component = target_component
        self._owns_connection = isinstance(connection, str)
        if self._owns_connection:
            self._mav = mavutil.mavlink_connection(connection)
            if self._mav.wait_heartbeat(timeout=heartbeat_timeout) is None:
                raise TimeoutError(
                    f"No MAVLink heartbeat from PX4 on '{connection}' - is "
                    "the SITL sim running?"
                )
        else:
            self._mav = connection
        self._configured = False

    def set_orientation(self, pitch_deg: float, yaw_deg: float = 0.0) -> None:
        """Point the gimbal to an absolute pitch/yaw in degrees.

        pitch_deg: 0 = level/forward, negative = down, positive = up.
        yaw_deg: 0 = forward, positive = right (earth frame).
        """
        if not self._configured:
            self._claim_control()
        self._send_command(
            MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            pitch_deg, yaw_deg, float("nan"), float("nan"), 0, 0, 0,
        )

    def point_down(self) -> None:
        """Point the camera straight down (nadir) - e.g. for ground mapping."""
        self.set_orientation(pitch_deg=-90.0, yaw_deg=0.0)

    def point_forward(self) -> None:
        """Level the gimbal back to looking straight ahead."""
        self.set_orientation(pitch_deg=0.0, yaw_deg=0.0)

    def _claim_control(self) -> None:
        # -1 params = "leave unchanged"; our own sysid/compid = "give us
        # primary control" for this gimbal manager.
        self._send_command(
            MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            self._mav.mav.srcSystem, self._mav.mav.srcComponent,
            -1, -1, 0, 0, 0,
        )
        self._configured = True

    def _send_command(self, command: int, p1, p2, p3, p4, p5, p6, p7) -> None:
        self._mav.mav.command_long_send(
            self._target_system, self._target_component,
            command, 0, p1, p2, p3, p4, p5, p6, p7,
        )
        ack = self._mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack is not None and ack.command == command:
            if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise RuntimeError(
                    f"Gimbal command {command} rejected (result={ack.result})"
                )

    def close(self) -> None:
        """Close the underlying MAVLink connection - but only if this
        instance opened it itself; a connection passed in by the caller is
        theirs to close."""
        if self._owns_connection:
            self._mav.close()

    def __enter__(self) -> "GimbalController":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    parser.add_argument(
        "--pitch", type=float, default=-90.0,
        help="Target pitch in degrees (0=level, -90=straight down).",
    )
    parser.add_argument("--yaw", type=float, default=0.0)
    args = parser.parse_args()

    print(f"Connecting to {args.connection} ...")
    gimbal = GimbalController(connection=args.connection)
    print(f"Setting gimbal pitch={args.pitch} yaw={args.yaw}")
    gimbal.set_orientation(pitch_deg=args.pitch, yaw_deg=args.yaw)
    gimbal.close()


if __name__ == "__main__":
    main()
