#!/usr/bin/env python3
"""Camera-only obstacle / person risk estimation (no ROS dependency).

Method (colour-agnostic for obstacles):
  1. Learn the floor appearance near the robot and the background (sky) colour.
  2. For every image column, scan upward from the bottom row while pixels stay
     floor-like.  The first non-floor pixel is the column's free-space boundary.
  3. Convert the boundary row to an *estimated* ground distance with a flat-floor
     pinhole model (known camera height / pitch from the SDF).  This is an
     estimate, not a range measurement: it assumes a flat floor and an object
     standing on it.
  4. Group neighbouring columns with similar distance into objects, measure their
     approximate metric width / height, and label tall-narrow objects as
     "person-like" (shape heuristic, not a trained person classifier).
  5. Convert distance + lateral offset from the robot path into a 0-100 risk
     score and a discrete state.
"""
import math
import numpy as np
import cv2

# ---- camera model (robot_risk_world.sdf: front_camera) ----------------------
IMG_W, IMG_H = 640, 480
HFOV = 1.05                      # rad
CAM_HEIGHT = 0.22 + 0.45         # chassis z + sensor z  [m]
CAM_PITCH_SDF = 0.10             # rad, sensor pitch in the SDF (positive = down)
# After the wheel-axis fix and 2 mm caster clearance the chassis rocks only
# +-0.77 deg between the casters (measured), so the SDF pitch is used directly.
# (+-0.77 deg -> about +-0.1 m distance error at 2 m.)
CAM_PITCH = CAM_PITCH_SDF
CAM_FORWARD = 0.23               # camera x offset from chassis centre [m]
FOCAL = (IMG_W / 2.0) / math.tan(HFOV / 2.0)
CX, CY = (IMG_W - 1) / 2.0, (IMG_H - 1) / 2.0

# ---- risk parameters --------------------------------------------------------
ROBOT_HALF_WIDTH = 0.24          # wheels reach +-0.24 m
CORRIDOR_HALF = 0.40             # robot half width + 0.16 m margin
MAX_RANGE = 6.0                  # ignore boundaries farther than this [m]
OBST_STOP, OBST_FAR = 1.4, 3.5   # distance where obstacle risk is 100 / 0
PERSON_STOP, PERSON_FAR = 1.9, 4.5
STATE_STOP = 70.0                # risk >= -> STOP
STATE_APPROACH = 30.0            # risk >= -> APPROACHING
STATE_STOP_EXIT = 60.0           # hysteresis: leave STOP only below this
N_SECTORS = 15                   # free-space histogram bins across the FOV

STATE_CLEAR = 'CLEAR'            # nothing detected
STATE_FAR = 'OBSTACLE_FAR'       # detected, low risk
STATE_NEAR = 'APPROACHING'       # obstacle approaching the path
STATE_PERSON = 'PERSON'          # person-like object detected (risk < stop)
STATE_STOP_ = 'STOP'             # stop required
STATE_INVALID = 'INVALID'        # image missing / stale / degenerate


def row_to_distance(v):
    """Ground distance (from the camera, along the optical axis direction
    projected on the floor) for an image row. inf above the horizon."""
    ang = CAM_PITCH + math.atan((v - CY) / FOCAL)
    if ang <= 0.01:
        return math.inf
    return CAM_HEIGHT / math.tan(ang)


ROW_DIST = np.array([row_to_distance(v) for v in range(IMG_H)])
COL_BEARING = np.arctan((np.arange(IMG_W) - CX) / FOCAL)   # +right in image
NEAREST_VISIBLE = ROW_DIST[IMG_H - 1]                      # ~1.2 m blind zone


def check_image(img):
    """Return (ok, reason). Rejects degenerate frames such as a uniform image."""
    if img is None:
        return False, 'no image'
    if img.shape[0] != IMG_H or img.shape[1] != IMG_W:
        return False, f'unexpected size {img.shape[1]}x{img.shape[0]}'
    std = float(img.std())
    if std < 3.0:
        return False, f'uniform image (std={std:.2f})'
    return True, ''


class RiskEstimator:
    def __init__(self):
        self.floor_ref = None       # per-row floor colour (IMG_H x 3), learned
        self.bg_ref = None          # background colour
        self.step_thr = 14.0        # max colour jump between adjacent rows
        self.ref_thr = 30.0         # max distance to learned floor colour
        self.in_stop = False        # STOP hysteresis state

    # -- appearance models ---------------------------------------------------
    def _update_models(self, img, boundary):
        top = img[:8].reshape(-1, 3).astype(np.float32)
        bg = np.median(top, axis=0)
        self.bg_ref = bg if self.bg_ref is None else 0.9 * self.bg_ref + 0.1 * bg
        # learn floor colour per row from columns whose boundary is above it
        h = img.shape[0]
        rows = np.arange(h)[:, None]
        floor_mask = rows > (boundary[None, :] + 4)
        f = img.astype(np.float32)
        new = np.full((h, 3), np.nan, np.float32)
        for v in range(int(CY) + 10, h):
            m = floor_mask[v]
            if m.sum() > 40:
                new[v] = np.median(f[v][m], axis=0)
        if self.floor_ref is None:
            self.floor_ref = new
        else:
            ok = ~np.isnan(new[:, 0])
            old_ok = ~np.isnan(self.floor_ref[:, 0])
            both = ok & old_ok
            self.floor_ref[both] = 0.9 * self.floor_ref[both] + 0.1 * new[both]
            self.floor_ref[ok & ~old_ok] = new[ok & ~old_ok]

    def _bootstrap(self, img):
        """First frame: assume the bottom 12 rows near the image centre are
        floor and that floor colour varies smoothly with the row."""
        f = img.astype(np.float32)
        seed = np.median(f[IMG_H - 12:, IMG_W // 2 - 80: IMG_W // 2 + 80].reshape(-1, 3), axis=0)
        self.floor_ref = np.full((IMG_H, 3), np.nan, np.float32)
        self.floor_ref[IMG_H - 12:] = seed

    # -- main ----------------------------------------------------------------
    def boundary(self, img):
        f = cv2.GaussianBlur(img, (5, 5), 0).astype(np.float32)
        h, w = f.shape[:2]
        ref = self.floor_ref
        known = ~np.isnan(ref[:, 0])
        step = np.abs(f[1:] - f[:-1]).sum(axis=2)                  # (h-1, w)
        step6 = np.abs(f[6:] - f[:-6]).sum(axis=2)                 # (h-6, w)
        bad = np.zeros((h, w), bool)
        bad[:-1] |= step > self.step_thr
        bad[:-6] |= step6 > 2.0 * self.step_thr                    # soft edges
        # absolute check only where a floor model exists for the row
        dref = np.abs(f[known] - ref[known][:, None, :]).sum(axis=2)
        bad[known] |= dref > self.ref_thr
        # bottom row must look like floor (blocked camera / object touching it)
        horizon = int(np.searchsorted(-ROW_DIST, -MAX_RANGE))       # first row with dist<=MAX
        bad[:horizon] = True
        # boundary = lowest row of the first "bad" pixel scanning up from bottom
        bound = np.full(w, -1, np.int32)
        flipped = bad[::-1]                                         # row 0 = bottom
        any_bad = flipped.any(axis=0)
        first = flipped.argmax(axis=0)
        bound[any_bad] = (h - 1) - first[any_bad]
        return bound, f

    def process(self, img_rgb):
        ok, reason = check_image(img_rgb)
        if not ok:
            return {'state': STATE_INVALID, 'risk': 100.0, 'reason': reason,
                    'objects': [], 'sectors': [0.0] * N_SECTORS}
        if self.floor_ref is None:
            self._bootstrap(img_rgb)
        bound, fimg = self.boundary(img_rgb)
        self._update_models(img_rgb, bound)

        h = IMG_H
        dist = np.where(bound >= h - 1, NEAREST_VISIBLE * 0.9,      # blocked at bottom row
                        ROW_DIST[np.clip(bound, 0, h - 1)])
        dist = np.where(dist > MAX_RANGE, math.inf, dist)
        lateral = np.tan(COL_BEARING) * np.where(np.isinf(dist), 0, dist)

        # column type: does the background (sky) start right above the boundary?
        vb = np.clip(bound - 4, 0, h - 1)
        above = fimg[vb, np.arange(IMG_W)]
        edge_col = np.abs(above - self.bg_ref[None, :]).sum(axis=1) < 12
        objects = self._group(img_rgb, fimg, bound, dist, lateral, edge_col)
        sectors = []
        edges = np.linspace(0, IMG_W, N_SECTORS + 1).astype(int)
        for i in range(N_SECTORS):
            d = dist[edges[i]:edges[i + 1]]
            fin = d[np.isfinite(d)]
            sectors.append(float(fin.min()) if fin.size else MAX_RANGE)

        risk = max([o['risk'] for o in objects], default=0.0)
        # a person only changes the state when in the path or closer than 3 m
        persons = [o for o in objects if o['label'] == 'person'
                   and ((o['in_path'] and o['distance'] < PERSON_FAR) or o['distance'] < 3.0)]
        real = [o for o in objects if o['label'] != 'floor_edge']
        top = max(objects, key=lambda o: o['risk']) if objects else None
        stop_relevant = top is not None and (top['in_path'] or top['distance'] < 1.25)
        thr = STATE_STOP_EXIT if self.in_stop else STATE_STOP
        self.in_stop = bool(risk >= thr and stop_relevant)
        if self.in_stop:
            state = STATE_STOP_
        elif persons:
            state = STATE_PERSON
        elif risk >= STATE_APPROACH:
            state = STATE_NEAR          # includes high risk from objects beside the path
        elif real:
            state = STATE_FAR
        else:
            state = STATE_CLEAR
        in_path = [o for o in objects if o['in_path']]
        return {'state': state, 'risk': round(float(risk), 1), 'reason': '',
                'objects': objects, 'sectors': [round(s, 2) for s in sectors],
                'nearest_in_path': round(min([o['distance'] for o in in_path], default=MAX_RANGE), 2),
                'boundary': bound}

    def _side_depth(self, dist, s, e, d):
        """Median free-space distance just left/right of columns s..e
        (None at the image border)."""
        w = len(dist)
        left = dist[max(0, s - 14):s - 3] if s - 3 >= 8 else None
        right = dist[e + 4:min(w, e + 15)] if e + 4 <= w - 8 else None
        med = lambda a: None if a is None or a.size == 0 else float(np.median(np.where(np.isinf(a), 99.0, a)))
        return med(left), med(right)

    def _top_row(self, fimg, s, e, yb, dist, d):
        """Top of an object standing on the floor between columns s..e.
        Each row above the floor contact is compared with the background
        just LEFT/RIGHT of the object, using only side strips that are
        farther away than the object (real background, not the same or a
        nearer object).  Works in front of walls, furniture or sky; falls
        back to the global background colour when no such strip exists."""
        w = fimg.shape[1]
        cols = fimg[:yb + 1, s:e + 1]                                         # (yb+1, n, 3)
        dl, dr = self._side_depth(dist, s, e, d)
        sides = []
        if dl is not None and dl > d + 0.3:
            sides.append(np.median(fimg[:yb + 1, max(0, s - 14):s - 3], axis=1))
        if dr is not None and dr > d + 0.3:
            sides.append(np.median(fimg[:yb + 1, e + 4:min(w, e + 15)], axis=1))
        if not sides:
            sides = [np.tile(self.bg_ref, (yb + 1, 1))]
        # a pixel belongs to the object if it differs from EVERY background
        # reference; a row is "object" if >= 25 % of its columns do (legs, gaps)
        differs = np.all([np.abs(cols - sd[:, None, :]).sum(axis=2) > 30 for sd in sides], axis=0)
        is_obj = differs.mean(axis=1) >= 0.25
        yt, gap = yb, 0
        for v in range(yb, -1, -1):
            if is_obj[v]:
                yt, gap = v, 0
            else:
                gap += 1
                if gap > 4:
                    break
        return yt

    def _group(self, img, fimg, bound, dist, lateral, edge_col):
        objs = []
        w = IMG_W
        valid = np.isfinite(dist)
        u = 0
        while u < w:
            if not valid[u]:
                u += 1
                continue
            s = u
            while u + 1 < w and valid[u + 1] and edge_col[u + 1] == edge_col[s] \
                    and abs(dist[u + 1] - dist[u]) < 0.25 + 0.08 * dist[u]:
                u += 1
            e = u
            u += 1
            if e - s < 6:
                continue
            seg_d = dist[s:e + 1]
            d = float(np.percentile(seg_d, 20))
            yb = int(np.median(bound[s:e + 1]))
            yt = self._top_row(fimg, s, e, yb, dist, d)
            width_m = (e - s) / FOCAL * d
            height_m = (yb - yt) / FOCAL * d
            lat = lateral[s:e + 1]
            lat_min = 0.0 if (lat.min() < 0 < lat.max()) else float(np.abs(lat).min())
            # what lies at the far floor edge (boundary row near the horizon, flat)?
            is_floor_edge = bool(np.mean(edge_col[s:e + 1]) > 0.5)
            label = 'floor_edge' if is_floor_edge else 'obstacle'
            # person-like: tall (1.15-2.1 m) and narrow at the floor contact
            # (legs, 0.04-0.75 m).  Shape heuristic, NOT a trained classifier.
            # ... and free-standing (excludes wall segments and doors, which are
            # continuous surfaces or touch the image border)
            dl, dr = self._side_depth(dist, s, e, d)
            # (both sides inside the image, depth jump on at least one side: a
            #  shadow can make one side look continuous)
            free_standing = dl is not None and dr is not None and (abs(dl - d) > 0.3 or abs(dr - d) > 0.3)
            if not is_floor_edge and free_standing and 1.15 <= height_m <= 2.1 and 0.04 <= width_m <= 0.75 \
                    and height_m / max(width_m, 1e-3) >= 1.9:
                label = 'person'
            # confidence: boundary contrast and support width (heuristic, 0-1)
            v0 = min(yb + 3, IMG_H - 1)
            v1 = max(yb - 3, 0)
            contrast = float(np.abs(fimg[v0, s:e + 1] - fimg[v1, s:e + 1]).sum(axis=1).mean())
            conf = min(1.0, 0.35 + 0.4 * min(1.0, (e - s) / 60.0) + 0.25 * min(1.0, contrast / 80.0))
            if is_floor_edge:
                conf *= 0.6
            path = 1.0 if lat_min < CORRIDOR_HALF else max(0.0, 1.0 - (lat_min - CORRIDOR_HALF) / 0.6)
            if label == 'person':
                prox = np.clip((PERSON_FAR - d) / (PERSON_FAR - PERSON_STOP), 0, 1)
            else:
                prox = np.clip((OBST_FAR - d) / (OBST_FAR - OBST_STOP), 0, 1)
            risk = 100.0 * prox * (0.3 + 0.7 * path) * (0.7 + 0.3 * conf)
            if prox >= 1.0 and path >= 1.0:
                risk = 100.0
            # lateral position of the object's edges (ROS convention: +left)
            lat_left = float(-math.tan(COL_BEARING[s]) * d)
            lat_right = float(-math.tan(COL_BEARING[e]) * d)
            objs.append({
                'label': label, 'confidence': round(conf, 2),
                'lat_left': round(lat_left, 2), 'lat_right': round(lat_right, 2),
                'bbox': [int(s), int(yt), int(e), int(yb)],
                'distance': round(d, 2), 'lateral': round(lat_min, 2),
                'width_m': round(width_m, 2), 'height_m': round(height_m, 2),
                'in_path': bool(lat_min < CORRIDOR_HALF), 'risk': round(float(risk), 1),
            })
        return objs


STATE_COLORS = {STATE_CLEAR: (80, 200, 80), STATE_FAR: (80, 200, 80), STATE_NEAR: (0, 200, 255),
                STATE_PERSON: (255, 120, 0), STATE_STOP_: (0, 0, 255), STATE_INVALID: (180, 0, 180)}


def draw(img_bgr, res, extra=''):
    out = img_bgr.copy()
    b = res.get('boundary')
    if b is not None:
        pts = np.stack([np.arange(IMG_W), np.clip(b, 0, IMG_H - 1)], 1).astype(np.int32)
        cv2.polylines(out, [pts], False, (0, 255, 0), 1)
    for o in res['objects']:
        x1, y1, x2, y2 = o['bbox']
        col = (255, 120, 0) if o['label'] == 'person' else ((160, 160, 160) if o['label'] == 'floor_edge' else (0, 0, 255))
        cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)
        txt = f"{o['label']} ~{o['distance']:.1f}m r{o['risk']:.0f} c{o['confidence']:.2f}"
        cv2.putText(out, txt, (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3)
        cv2.putText(out, txt, (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
    sc = STATE_COLORS.get(res['state'], (255, 255, 255))
    cv2.rectangle(out, (0, 0), (IMG_W, 26), (25, 25, 25), -1)
    cv2.putText(out, f"{res['state']}  risk {res['risk']:.0f}/100 {extra}", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, sc, 2)
    return out
