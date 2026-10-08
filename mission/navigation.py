"""Sensor summaries, public-region coverage and immediate protection; no path planner."""
import math

BOUNDS = (-1.0, 3.6, -2.1, 2.1)
BASE = (0, 0)
SECTORS = {"front": 0, "left": math.pi / 2, "right": -math.pi / 2, "rear": math.pi}


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def inside(point, margin=0):
    return BOUNDS[0] + margin <= point[0] <= BOUNDS[1] - margin and BOUNDS[2] + margin <= point[1] <= BOUNDS[3] - margin


def relative(position, yaw, point):
    return {"distance_m": round(math.dist(position[:2], point[:2]), 3),
            "bearing_deg": round(math.degrees(wrap(math.atan2(point[1] - position[1], point[0] - position[0]) - yaw)), 1)}


def lidar_summary(points):
    sectors = {name: 4.0 for name in SECTORS}
    for x, y, z in points:
        if not all(math.isfinite(v) for v in (x, y, z)):
            continue  # Infinity is a clear ray to the configured maximum range.
        distance = math.hypot(x, y)
        # Exclude self returns using rover coordinates, preserving external front returns.
        if x < 0 and math.hypot(x + .38, y) < .48:
            continue
        angle = math.atan2(y, x)
        for name, direction in SECTORS.items():
            if abs(wrap(angle - direction)) <= math.pi / 4:
                sectors[name] = min(sectors[name], distance)
    return {key: round(value, 3) for key, value in sectors.items()}


class Coverage:
    """Visited neighbourhoods on a public grid; never certifies sample discovery."""
    def __init__(self):
        self.cells = [(BOUNDS[0] + .2 + .4 * x, BOUNDS[2] + .2 + .4 * y)
                      for x in range(11) for y in range(10)]
        self.visited = set()

    def observe(self, position):
        self.visited.update(i for i, point in enumerate(self.cells) if math.dist(position[:2], point) <= .65)

    def state(self, position, yaw):
        novelty = {}
        for name, angle in SECTORS.items():
            novelty[name] = sum(i not in self.visited and math.dist(position[:2], point) < 2 and
                               abs(wrap(math.atan2(point[1] - position[1], point[0] - position[0]) - yaw - angle)) < math.pi / 4
                               for i, point in enumerate(self.cells))
        return {"visited_pct": round(100 * len(self.visited) / len(self.cells), 1), "unvisited_nearby_sectors": novelty,
                "measure": "0.4m public grid centres within 0.65m of GPS track; not visual coverage or proof of all discoveries"}

    def frontier(self, position):
        remaining = [point for i, point in enumerate(self.cells) if i not in self.visited and inside(point, .45)]
        return min(remaining, key=lambda point: math.dist(position[:2], point)) if remaining else None


def safety(position, roll, pitch, sectors, command, yaw=0, valid=True):
    if not valid or not all(math.isfinite(v) for v in (*position, roll, pitch, yaw, *sectors.values())):
        return "invalid_sensors"
    if max(abs(roll), abs(pitch)) > .4:
        return "excessive_tilt"
    if not inside(position):
        return "region_boundary"
    if command == "forward":
        projected = [position[0] + .46 * math.cos(yaw), position[1] + .46 * math.sin(yaw)]
        if not inside(projected, .05):
            return "boundary_ahead"
        if sectors["front"] < .32:
            return "front_obstacle"
    if command == "reverse":
        projected = [position[0] - .46 * math.cos(yaw), position[1] - .46 * math.sin(yaw)]
        if not inside(projected, .05):
            return "boundary_behind"
        # Lidar sits 0.38 m ahead of the rover centre; protect the rear body too.
        if sectors["rear"] < .8:
            return "rear_obstacle"
    if command == "spin" and min(sectors.values()) < .24:
        return "spin_clearance"
    return None
