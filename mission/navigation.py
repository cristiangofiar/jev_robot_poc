"""Small observed occupancy grid + A*. Coordinates in metres, ENU, yaw in radians."""
import heapq
import math

CELL = 0.15
BOUNDS = (-1.0, 3.6, -2.1, 2.1)
RADIUS = 0.43


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def cell(point):
    return tuple(round(v / CELL) for v in point[:2])


def inside(point):
    return BOUNDS[0] <= point[0] <= BOUNDS[1] and BOUNDS[2] <= point[1] <= BOUNDS[3]


class Navigator:
    def __init__(self):
        self.obstacles = set()
        self.path = []
        self.last_plan = -10
        self.goal = None

    def observe(self, position, yaw, ranges, markers):
        # ponytail: static 2D map for gentle slopes; full transforms/clearing for rough or dynamic terrain.
        # One horizontal lidar layer. Infinity denotes clear space to maxRange.
        origin = (position[0] + .38 * math.cos(yaw), position[1] + .38 * math.sin(yaw))
        for x, y, z in ranges:
            if not all(math.isfinite(v) for v in (x, y, z)) or math.hypot(x, y) < .03:
                continue
            point = (origin[0] + x * math.cos(yaw) - y * math.sin(yaw), origin[1] + x * math.sin(yaw) + y * math.cos(yaw))
            if math.dist(point, position[:2]) < .6:
                continue  # Reject rover self returns before building the external map.
            if math.hypot(*point) < .65:
                continue  # The public landing pad is a known traversable home zone.
            if any(math.dist(point, m[:2]) < .25 for m in markers):
                continue  # Noncolliding sample markers are camera targets, not rocks.
            if inside(point):
                self.obstacles.add(cell(point))

    def plan(self, position, goal):
        blocked = set()
        inflation = math.ceil(RADIUS / CELL)
        for x, y in self.obstacles:
            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    if math.hypot(dx, dy) * CELL <= RADIUS:
                        blocked.add((x + dx, y + dy))
        start, end = cell(position), cell(goal)
        if end in blocked:
            return []
        frontier = [(0, start)]
        previous, costs = {start: None}, {start: 0}
        while frontier:
            _, here = heapq.heappop(frontier)
            if here == end:
                path = []
                while here is not None:
                    path.append((here[0] * CELL, here[1] * CELL))
                    here = previous[here]
                return path[::-1]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (here[0] + dx, here[1] + dy)
                if nxt in blocked or not inside((nxt[0] * CELL, nxt[1] * CELL)):
                    continue
                if dx and dy and ((here[0] + dx, here[1]) in blocked or (here[0], here[1] + dy) in blocked):
                    continue
                cost = costs[here] + math.hypot(dx, dy)
                if cost < costs.get(nxt, math.inf):
                    costs[nxt], previous[nxt] = cost, here
                    heapq.heappush(frontier, (cost + math.dist(nxt, end), nxt))
        return []

    def command(self, position, yaw, goal, now):
        if math.dist(position[:2], goal[:2]) < .12:
            return "arrived", 0
        if self.goal != goal or now - self.last_plan > 1:
            self.path = self.plan(position, goal)
            if self.path:
                self.path.append(tuple(goal[:2]))
            self.goal, self.last_plan = goal, now
        while len(self.path) > 1 and math.dist(position[:2], self.path[0]) < .18:
            self.path.pop(0)
        if not self.path:
            return "no_route", 0
        target = self.path[min(1, len(self.path) - 1)]
        error = wrap(math.atan2(target[1] - position[1], target[0] - position[0]) - yaw)
        if abs(error) > .22:
            return "spin", 1 if error > 0 else -1
        return "forward", max(-1, min(1, error / .22))


def safety(position, roll, pitch, ranges, command):
    if not all(math.isfinite(v) for v in (*position, roll, pitch)) or not ranges or any(math.isnan(d) or d < 0 for d in ranges):
        return "invalid_sensors"
    if max(abs(roll), abs(pitch)) > .4:
        return "excessive_tilt"
    if not inside(position):
        return "region_boundary"
    # Forward lidar origin extends 0.38 m beyond the rover reference point.
    middle = len(ranges) // 2
    front = min(ranges[middle - 30:middle + 31])
    if command == "forward" and front < .28:
        return "front_obstacle"
    if command == "spin" and min(ranges) < .22:
        return "spin_clearance"
    return None
