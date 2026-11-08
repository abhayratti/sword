"""Choose a block from where the opponent's sword tip is threatening.

Offsets are measured from the tip's position at the opponent's guard, in their frame:
  x: toward the robot's left (the opponent's right)
  y: up
  z: toward the robot
Block names are from the robot's point of view.
"""
import numpy as np

BLOCKS = ['guard', 'high', 'high_left', 'high_right', 'mid_left', 'mid_right']  # also the zone calibration order

# (height band, side band) -> block. Side +1 is the robot's left. There are no low blocks: low strikes
# on a side use that side's mid block, and low in the centre is guard.
ZONES = {(1, 1): 'high_left', (1, 0): 'high', (1, -1): 'high_right',
         (0, 1): 'mid_left', (0, 0): 'guard', (0, -1): 'mid_right',
         (-1, 1): 'mid_left', (-1, 0): 'guard', (-1, -1): 'mid_right'}


def band(value, threshold, margin, current):
    """-1, 0 or +1. Entering a band needs `margin` beyond its edge; leaving needs `margin` back past it."""
    if value > threshold + (-margin if current == 1 else margin): return 1
    if value < -threshold + (margin if current == -1 else -margin): return -1
    return 0


class Blocker:
    def __init__(self, cfg, zones=None):
        self.cfg = cfg
        # Calibrated wind-up point (x, y) per block. Without it, a fixed grid around the guard is used.
        self.zones = zones or {}
        self.reset()

    def reset(self):
        self.zone, self.bands = 'guard', (0, 0)
        self.candidate = self.candidate_since = None
        self.prev = self.prev_at = None
        self.velocity = np.zeros(3)
        self.strike, self.slow_since = False, None
        self.predicted = np.zeros(3)

    def _classify(self, offset):
        c = self.cfg
        if self.zones:
            # Nearest calibrated zone; stay in the current one unless another is clearly closer.
            p = np.asarray(offset[:2])
            dist = {name: np.linalg.norm(p - np.asarray(point)) for name, point in self.zones.items()}
            best = min(dist, key=dist.get)
            if self.zone in dist and dist[self.zone] - dist[best] < c['hysteresis_m']: best = self.zone
            return best, self.bands
        height = band(offset[1], c['height_m'], c['hysteresis_m'], self.bands[0])
        side = band(offset[0], c['side_m'], c['hysteresis_m'], self.bands[1])
        return ZONES[(height, side)], (height, side)

    def _switch(self, zone, bands):
        self.zone, self.bands, self.candidate = zone, bands, None

    def update(self, offset, at):
        c = self.cfg
        offset = np.asarray(offset, float)
        if self.prev is not None and 0 < at - self.prev_at <= .25:
            self.velocity = .5 * self.velocity + .5 * (offset - self.prev) / (at - self.prev_at)
        else:
            self.velocity = np.zeros(3)
        self.prev, self.prev_at = offset, at
        speed = np.linalg.norm(self.velocity)

        if c['strike_lock'] and speed > c['strike_speed_m_s']:
            # Strike: commit to the block for where it started (the wind-up) and hold it,
            # rather than chasing the blade across its path.
            if not self.strike:
                self.strike = True
                self._switch(*self._classify(offset))
            self.slow_since = None
            self.predicted = offset
            return self.zone
        if self.strike:
            self.slow_since = self.slow_since or at
            if at - self.slow_since < c['release_s']: return self.zone
            self.strike = False

        # Not striking: cover where the tip is heading, switching only once it settles there.
        self.predicted = offset + self.velocity * c['lookahead_s']
        zone, bands = self._classify(self.predicted)
        if zone == self.zone:
            self.candidate = None
        elif zone != self.candidate:
            self.candidate, self.candidate_since = zone, at
        if self.candidate and at - self.candidate_since >= c['dwell_s']:
            self._switch(zone, bands)
        return self.zone

    def snapshot(self):
        return {'zone': self.zone, 'strike': self.strike, 'offset': None if self.prev is None else self.prev.tolist(),
                'predicted': self.predicted.tolist(), 'speed': float(np.linalg.norm(self.velocity))}
