"""Validate a simple GPS inclusion fence and straight flight legs inside it."""
import math


def cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


def subtract(a, b):
    return a[0]-b[0], a[1]-b[1]


def on_segment(p, a, b):
    return (abs(cross(subtract(b, a), subtract(p, a))) <= 1e-7
            and min(a[0], b[0])-1e-8 <= p[0] <= max(a[0], b[0])+1e-8
            and min(a[1], b[1])-1e-8 <= p[1] <= max(a[1], b[1])+1e-8)


def intersection_parameters(a, b, c, d):
    r, s = subtract(b, a), subtract(d, c)
    denominator = cross(r, s)
    if abs(denominator) < 1e-9:
        length2 = r[0]**2+r[1]**2
        if length2 == 0:
            return [0.] if on_segment(a, c, d) else []
        return [((p[0]-a[0])*r[0]+(p[1]-a[1])*r[1])/length2
                for p in (c, d) if on_segment(p, a, b)]
    t = cross(subtract(c, a), s)/denominator
    u = cross(subtract(c, a), r)/denominator
    return [max(0., min(1., t))] if -1e-9 <= t <= 1+1e-9 and -1e-9 <= u <= 1+1e-9 else []


class FlightBoundary:
    def __init__(self, points):
        from mission_geometry import gps_points, to_local
        self.points = gps_points(points, 'flight_boundary')
        if self.points and self.points[-1] == self.points[0]:
            self.points.pop()
        if not 3 <= len(self.points) <= 70:
            raise ValueError('flight_boundary requires 3 to 70 vertices in perimeter order')
        if len(set(self.points)) != len(self.points):
            raise ValueError('flight_boundary has duplicate vertices')
        self.origin = self.points[0]
        self.xy = [to_local(p, *self.origin) for p in self.points]
        if max(math.dist(a, b) for a in self.xy for b in self.xy) > 20000:
            raise ValueError('flight_boundary exceeds the local projection limit of 20 km')
        self.edges = list(zip(self.xy, self.xy[1:]+self.xy[:1]))
        for i, (a, b) in enumerate(self.edges):
            c = self.xy[(i+2) % len(self.xy)]
            if abs(cross(subtract(b, a), subtract(c, b))) < 1e-7:
                raise ValueError('flight_boundary has collinear consecutive vertices')
            for j, (c, d) in enumerate(self.edges):
                if j <= i or j == i+1 or (i == 0 and j == len(self.edges)-1):
                    continue
                if (intersection_parameters(a, b, c, d)
                        or intersection_parameters(c, d, a, b)):
                    raise ValueError('flight_boundary must not intersect itself')

    def contains(self, point):
        if any(on_segment(point, a, b) for a, b in self.edges):
            return True
        x, y = point
        inside = False
        for a, b in self.edges:
            if (a[1] > y) != (b[1] > y) and x < a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1]):
                inside = not inside
        return inside

    def check_route(self, points, name):
        from mission_geometry import to_local
        xy = [to_local(p, *self.origin) for p in points]
        for i, point in enumerate(xy):
            if not self.contains(point):
                raise ValueError(f'{name}: point {i+1} lies outside flight_boundary')
        for i, (a, b) in enumerate(zip(xy, xy[1:])):
            cuts = sorted({0., 1., *(t for c, d in self.edges
                                    for t in intersection_parameters(a, b, c, d))})
            for start, end in zip(cuts, cuts[1:]):
                t = (start+end)/2
                if not self.contains((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))):
                    raise ValueError(f'{name}: leg {i+1} leaves flight_boundary')
