# Maps 3 Hall sensor readings to a 0..1 handbrake position.
#
# The 3 sensors sit in a line along the magnet's travel, so their readings
# change in turn as the magnet passes each one (order s2, s3, s1 when pulling).
# Two things rule out a simple formula:
# - readings are not monotonic: each sensor dips as the magnet goes by;
# - the lever's play changes the magnet's gap, so pulling and releasing trace
#   different paths in 3D: releasing passes closer, with dips ~3x deeper.
#
# So the calibration learns both paths. It records the pulls, finds the two
# ends (rest, measured before calibrating, and the end stop, where the brake
# was held still the longest), cuts the recording into pulls (rest -> end stop)
# and releases (end stop -> rest), and averages each kind into a curve of
# CURVE_POINTS points spread evenly along its length. A live reading's position
# is where it projects onto the nearer curve: 0 at rest, 1 at the end stop.
#
# Memory: everything big is allocated once, in Calibrator(). Growing lists and
# dicts during the recording fragments CircuitPython's small heap until an
# allocation fails with MemoryError, even with 100 KB free.
#
# Pure Python so it runs on CircuitPython and can be tested on a PC.
import math
from array import array

MAX_SAMPLES = 4000   # 30 s at ~120 Hz; later samples are ignored
CURVE_POINTS = 24    # points per learned curve, between rest and end stop
WINDOW = 2           # recording smoothed over 2 * WINDOW + 1 samples
GATE = 150           # raw units; points closer than this are merged before
                     # measuring a path's length, or noise would add to it
ZONE = 400           # raw units around rest / end stop that count as there
STILL_SMOOTHING = 0.2  # weight of a new reading in the stillness average
STILL_RADIUS = 400   # raw units; moving less than this = held still
HOLD_SAMPLES = 40    # samples held still (~0.4 s) to count as a hold
MAX_HOLDS = 8        # distinct places where the brake was held
DEAD_ZONE = 0.04     # this close to either end reads exactly 0 or 1

INF = float("inf")


def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class Calibrator:
    def __init__(self, rest):
        self.rest = [float(r) for r in rest]
        self.raw = array("H", (0 for _ in range(3 * MAX_SAMPLES)))
        self.samples = 0
        # Stillness, also used by code.py for the UI's instructions.
        self.smooth = list(self.rest)
        self.still = False
        self._anchor = list(self.rest)
        self._still_for = 0
        # Places held still away from rest: sum x, y, z and count each.
        self._holds = array("f", (0 for _ in range(4 * MAX_HOLDS)))
        self._hold_count = 0

    def add(self, values):
        if self.samples < MAX_SAMPLES:
            i = 3 * self.samples
            self.raw[i], self.raw[i + 1], self.raw[i + 2] = values
        self.samples += 1

        s = self.smooth
        for k in range(3):
            s[k] += (values[k] - s[k]) * STILL_SMOOTHING
        if _dist(s, self._anchor) > STILL_RADIUS:
            self._anchor[:] = s
            self._still_for = 0
        self._still_for += 1
        self.still = self._still_for >= HOLD_SAMPLES
        if self.still and _dist(s, self.rest) > ZONE:
            self._add_hold(s)

    def _add_hold(self, s):
        holds = self._holds
        for h in range(0, 4 * self._hold_count, 4):
            n = holds[h + 3]
            center = (holds[h] / n, holds[h + 1] / n, holds[h + 2] / n)
            if _dist(center, s) < STILL_RADIUS:
                break
        else:
            if self._hold_count == MAX_HOLDS:
                return
            h = 4 * self._hold_count
            self._hold_count += 1
        holds[h] += s[0]
        holds[h + 1] += s[1]
        holds[h + 2] += s[2]
        holds[h + 3] += 1

    def _smoothed(self, i):
        """Recorded sample i, averaged with its neighbors (no lag)."""
        n = min(self.samples, MAX_SAMPLES)
        lo, hi = max(0, i - WINDOW), min(n, i + WINDOW + 1)
        raw = self.raw
        return [sum(raw[3 * j + k] for j in range(lo, hi)) / (hi - lo) for k in range(3)]

    def finish(self):
        # End stop: where the brake was held still the longest, away from rest.
        holds = self._holds
        best = -1
        for h in range(0, 4 * self._hold_count, 4):
            if best < 0 or holds[h + 3] > holds[best + 3]:
                best = h
        if best < 0:
            raise ValueError("never held still away from rest: pull to the end stop and hold a second")
        n = holds[best + 3]
        end = [holds[best] / n, holds[best + 1] / n, holds[best + 2] / n]
        if _dist(self.rest, end) < 3 * ZONE:
            raise ValueError("handbrake barely moved during calibration")

        # Cut the recording where it enters rest or end-stop zones, and learn
        # from each crossing between the two.
        curves = [_CurveSum(), _CurveSum()]  # pulls, releases
        zone, zone_at = None, 0
        for i in range(min(self.samples, MAX_SAMPLES)):
            p = self._smoothed(i)
            if _dist(p, self.rest) < ZONE:
                here = 0
            elif _dist(p, end) < ZONE:
                here = 1
            else:
                continue
            if zone is not None and here != zone:
                curves[zone].add([self._smoothed(j) for j in range(zone_at, i + 1)], zone == 1)
            zone, zone_at = here, i
        learned = [c.curve(self.rest, end) for c in curves if c.count]
        if not learned:
            raise ValueError("no full pull: pull all the way to the end stop and release")
        return Mapping(learned, curves[0].count, curves[1].count)


class _CurveSum:
    """Sum of crossings, each resampled to CURVE_POINTS evenly spaced points."""

    def __init__(self):
        self.sums = [0.0] * (3 * CURVE_POINTS)
        self.count = 0

    def add(self, points, reverse):
        if reverse:
            points.reverse()
        # Merge points closer than GATE, then measure the length along them.
        kept = [points[0]]
        for p in points[1:]:
            if _dist(p, kept[-1]) >= GATE:
                kept.append(p)
        if kept[-1] is not points[-1]:
            kept.append(points[-1])
        if len(kept) < 3:
            return  # too fast to follow: a few samples across the whole travel
        along = [0.0]
        for a, b in zip(kept, kept[1:]):
            along.append(along[-1] + _dist(a, b))
        total = along[-1]
        seg = 0
        for k in range(CURVE_POINTS):
            target = total * k / (CURVE_POINTS - 1)
            while seg < len(kept) - 2 and along[seg + 1] < target:
                seg += 1
            span = along[seg + 1] - along[seg]
            u = (target - along[seg]) / span if span else 0.0
            for c in range(3):
                a = kept[seg][c]
                self.sums[3 * k + c] += a + (kept[seg + 1][c] - a) * u
        self.count += 1

    def curve(self, rest, end):
        """Averaged points from rest to end, with their 0..1 position."""
        points = [rest]
        for k in range(CURVE_POINTS):
            points.append([self.sums[3 * k + c] / self.count for c in range(3)])
        points.append(end)
        along = [0.0]
        for a, b in zip(points, points[1:]):
            along.append(along[-1] + _dist(a, b))
        coords = array("l", (int(round(v)) for p in points for v in p))
        position = array("f", (a / along[-1] for a in along))
        return coords, position


def _project(coords, position, k, x, y, z):
    """Squared distance to segment k of a curve, and the position there."""
    i = 3 * k
    ax, ay, az = coords[i], coords[i + 1], coords[i + 2]
    bx, by, bz = coords[i + 3] - ax, coords[i + 4] - ay, coords[i + 5] - az
    px, py, pz = x - ax, y - ay, z - az
    length2 = bx * bx + by * by + bz * bz
    u = (px * bx + py * by + pz * bz) / length2 if length2 else 0.0
    u = min(1.0, max(0.0, u))
    dx, dy, dz = px - u * bx, py - u * by, pz - u * bz
    t = position[k] + u * (position[k + 1] - position[k])
    return dx * dx + dy * dy + dz * dz, t


class Mapping:
    def __init__(self, curves, pulls, releases):
        self.curves = curves
        self.pulls = pulls
        self.releases = releases

    def raw_position(self, values):
        """0..1 along the nearest learned curve, without the dead zones.

        Finds each curve's nearest point with integer math first, then projects
        onto the two segments around it. On CircuitPython every float is an
        allocation: projecting onto all segments took ~90 ms per reading."""
        x, y, z = int(values[0]), int(values[1]), int(values[2])
        best_d, best_t = INF, 0.0
        for coords, position in self.curves:
            nearest, nearest_d = 0, None
            for i in range(0, len(coords), 3):
                dx, dy, dz = x - coords[i], y - coords[i + 1], z - coords[i + 2]
                d = dx * dx + dy * dy + dz * dz
                if nearest_d is None or d < nearest_d:
                    nearest, nearest_d = i // 3, d
            for k in (nearest - 1, nearest):
                if 0 <= k < len(position) - 1:
                    d, t = _project(coords, position, k, x, y, z)
                    if d < best_d:
                        best_d, best_t = d, t
        return best_t

    def position(self, values):
        pos = (self.raw_position(values) - DEAD_ZONE) / (1 - 2 * DEAD_ZONE)
        return min(1.0, max(0.0, pos))

    def summary(self):
        return "pulls={} releases={}".format(self.pulls, self.releases)
