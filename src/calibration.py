# Maps 3 Hall sensor readings to a 0..1 handbrake position.
#
# The sensors are not monotonic: as the magnet passes them, readings dip and
# come back, so "distance from rest" is not a usable position. Instead, the
# calibration records the path the readings trace in 3D while the brake is
# pulled, and the position is how far along that path a reading sits.
#
# Recording: samples are clustered into nodes (any sample within NODE_SIZE of
# an existing node joins it). Consecutive samples in different nodes link those
# nodes, which follows the real movement. Afterwards, the distance along those
# links from the rest node gives each node its position on the path.
#
# Pure Python so it runs on CircuitPython and can be tested on a PC.
import math
from array import array

NODE_SIZE = 200      # raw units; must stay above sensor noise (~150)
MAX_NODES = 300
NEIGHBOR_LINK = 2    # also link nodes closer than NEIGHBOR_LINK * NODE_SIZE
MAX_STEP = 4         # ignore jumps longer than MAX_STEP * NODE_SIZE between
                     # samples: a fast flick would shortcut across the path
DWELL_SAMPLES = 30   # same node this many samples in a row = brake held still
REST_ZONE_MAX = 0.3  # held-still spots below this fraction of the path = rest
NEAREST = 3          # nodes blended when mapping a live reading


def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class Calibrator:
    def __init__(self, rest):
        self.nodes = [tuple(rest)]
        self.links = {}
        self.dwell = set()
        self.samples = 0
        self._current = 0
        self._run = 0

    def add(self, values):
        self.samples += 1
        node = self._node_for(values)
        if node == self._current:
            self._run += 1
            if self._run == DWELL_SAMPLES:
                self.dwell.add(node)
            return
        self._link(self._current, node)
        self._current = node
        self._run = 1

    def _node_for(self, values):
        # Stay on the current node while close enough: keeps noise from
        # flickering between two neighbors.
        if _dist(values, self.nodes[self._current]) < NODE_SIZE:
            return self._current
        best, best_d = 0, None
        for i, node in enumerate(self.nodes):
            d = _dist(values, node)
            if best_d is None or d < best_d:
                best, best_d = i, d
        if best_d < NODE_SIZE or len(self.nodes) >= MAX_NODES:
            return best
        self.nodes.append(tuple(values))
        return len(self.nodes) - 1

    def _link(self, a, b):
        d = _dist(self.nodes[a], self.nodes[b])
        if a != b and d < MAX_STEP * NODE_SIZE:
            self.links[(min(a, b), max(a, b))] = d

    def finish(self):
        nodes = self.nodes
        n = len(nodes)
        for a in range(n):
            for b in range(a + 1, n):
                if _dist(nodes[a], nodes[b]) < NEIGHBOR_LINK * NODE_SIZE:
                    self._link(a, b)

        neighbors = [[] for _ in range(n)]
        for (a, b), d in self.links.items():
            neighbors[a].append((b, d))
            neighbors[b].append((a, d))

        # Dijkstra from the rest node (node 0). O(n^2) is fine for a few
        # hundred nodes and needs no heapq, which CircuitPython lacks.
        inf = float("inf")
        along = [inf] * n
        along[0] = 0.0
        done = [False] * n
        for _ in range(n):
            u, best = -1, inf
            for i in range(n):
                if not done[i] and along[i] < best:
                    u, best = i, along[i]
            if u < 0:
                break
            done[u] = True
            for v, d in neighbors[u]:
                if best + d < along[v]:
                    along[v] = best + d

        reached = [i for i in range(n) if along[i] < inf]
        full = max(along[i] for i in reached)
        if full < 10 * NODE_SIZE:
            raise ValueError("handbrake barely moved during calibration")

        # Where the brake was held still tells where rest and full really are:
        # rest drifts a little after each pull, and the hold at the end stop is
        # slightly short of the farthest point a quick flick reaches.
        held_low = [along[i] for i in self.dwell if along[i] < REST_ZONE_MAX * full]
        held_high = [along[i] for i in self.dwell if full / 2 <= along[i] < inf]
        low = max(held_low) + NODE_SIZE / 2 if held_low else NODE_SIZE
        top = max(held_high) - NODE_SIZE / 2 if held_high else 0.95 * full
        top = max(top, 0.75 * full)

        return Mapping(
            [nodes[i] for i in reached], [along[i] for i in reached], low, top
        )


class Mapping:
    def __init__(self, nodes, along, low, top):
        self.nodes = nodes
        self.along = array("f", along)
        self.low = low
        self.top = top

    def position(self, values):
        # Blend the positions of the nearest nodes, weighted by closeness.
        nearest = []
        for i, node in enumerate(self.nodes):
            d = _dist(values, node)
            if len(nearest) < NEAREST:
                nearest.append((d, i))
                nearest.sort()
            elif d < nearest[-1][0]:
                nearest[-1] = (d, i)
                nearest.sort()
        total = weight = 0.0
        for d, i in nearest:
            w = 1.0 / (d + 1.0)
            total += w * self.along[i]
            weight += w
        pos = (total / weight - self.low) / (self.top - self.low)
        return min(1.0, max(0.0, pos))

    def summary(self):
        return "nodes={} rest_zone={:.0f} full={:.0f} path={:.0f}".format(
            len(self.nodes), self.low, self.top, max(self.along)
        )
