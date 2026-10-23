# -*- coding: utf-8 -*-
"""
=============================================================================
[MBBN World-Class 2D Nesting Multi-Core Engine]
Program: MBB Multi-Container Yield Optimizer (Python 3.14 / Shapely / GEOS)
Version: v3.2 (2026-10-23 09:15:00)
Architecture: Feature-Aware Multi-Bin Compatibility + Fast Real-Coordinate Chords
              + Exact Buffer Containment + Wall-Adaptive Curved Tangents + 20-Core Parallel
=============================================================================
"""

import sys
import os
import json
import time
import math
import random
from multiprocessing import Pool, cpu_count

try:
    from shapely.geometry import Polygon, box, Point
    from shapely.prepared import prep
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False

def is_point_in_poly(x, y, poly_pts):
    """Ray casting point in polygon test (pure python fallback)"""
    n = len(poly_pts)
    inside = False
    p1x, p1y = poly_pts[0]
    for i in range(1, n + 1):
        p2x, p2y = poly_pts[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def get_poly_x_range(poly_pts, y):
    """Find left and right x-boundaries of polygon at height y"""
    xs = []
    n = len(poly_pts)
    for i in range(n):
        p1 = poly_pts[i]
        p2 = poly_pts[(i + 1) % n]
        x1, y1 = p1
        x2, y2 = p2
        if min(y1, y2) - 1e-4 <= y <= max(y1, y2) + 1e-4:
            if abs(y2 - y1) < 1e-5:
                xs.extend([x1, x2])
            else:
                t = (y - y1) / (y2 - y1)
                t = max(0.0, min(1.0, t))
                xs.append(x1 + t * (x2 - x1))
    if len(xs) >= 2:
        return (min(xs), max(xs))
    return None

def rects_overlap(r1, r2, gap=0.0):
    """Check if two rectangles overlap with gap: r = (x, y, w, h)"""
    x1, y1, w1, h1 = r1
    x2, y2, w2, h2 = r2
    return not (x1 + w1 + gap <= x2 or
                x2 + w2 + gap <= x1 or
                y1 + h1 + gap <= y2 or
                y2 + h2 + gap <= y1)

def compute_bigness(p, weight_dim=500.0, weight_aspect=50.0):
    """
    Composite Bigness metric:
    Prioritizes parts with large dimensions (max(W, H)) and large area (W * H).
    """
    w = float(p['w'])
    h = float(p['h'])
    area = float(p.get('area', w * h))
    max_d = max(w, h)
    min_d = max(1.0, min(w, h))
    aspect = max_d / min_d
    return area + (max_d * weight_dim) + (aspect * weight_aspect)

class NestingContainer:
    """
    Geometry-Aware Container Engine supporting both Circle and Polygon remnants.
    Tracks exact real-coordinate free space without grid discretization loss.
    """
    def __init__(self, c_data, gap=3.0, margin=2.0):
        self.id = c_data['id']
        self.type = c_data.get('type', 'polygon')
        self.gap = float(gap)
        self.margin = float(margin)
        self.area = float(c_data.get('area', 1.0))
        self.placed = []

        if self.type == 'circle':
            self.cx, self.cy = c_data['circle_center']
            self.R = float(c_data['circle_radius'])
            self.min_ox, self.min_oy = self.cx - self.R, self.cy - self.R
            self.max_ox, self.max_oy = self.cx + self.R, self.cy + self.R
            self.eff_R = max(0.0, self.R - self.margin)
            self.eff_R_sq = self.eff_R * self.eff_R + 1e-4
            self.prep_inner = None
        else:
            self.outer_pts = c_data['outer_polygon']
            self.min_ox, self.min_oy, self.max_ox, self.max_oy = c_data['outer_bbox']
            poly = Polygon(self.outer_pts)
            if not poly.is_valid:
                poly = poly.buffer(0)
            inner_poly = poly.buffer(-self.margin)
            if inner_poly.is_empty:
                inner_poly = poly
            self.prep_inner = prep(inner_poly)

    def is_inside(self, gx, gy, w, h):
        """Exact Euclidean clearance test."""
        if self.type == 'circle':
            for px, py in [(gx, gy), (gx + w, gy), (gx + w, gy + h), (gx, gy + h)]:
                if (px - self.cx)**2 + (py - self.cy)**2 > self.eff_R_sq:
                    return False
            return True
        else:
            if self.prep_inner is not None:
                b = box(gx, gy, gx + w, gy + h)
                return self.prep_inner.covers(b)

            pts_to_check = [
                (gx, gy), (gx+w, gy), (gx+w, gy+h), (gx, gy+h),
                (gx + w*0.5, gy), (gx + w*0.5, gy+h),
                (gx, gy + h*0.5), (gx+w, gy + h*0.5)
            ]
            for px, py in pts_to_check:
                if not is_point_in_poly(px, py, self.outer_pts):
                    return False
            return True

    def find_best_position(self, pw, ph, orientations=['horizontal', 'vertical'], align_pref='left'):
        """
        Finds optimal position testing horizontal, vertical, and natural orientations.
        Horizontal: Long side on X (rot=0 if pw>=ph else 90).
        Vertical: Long side on Y (rot=90 if pw>=ph else 0).
        """
        best_score = float('inf')
        best_node = None

        candidate_configs = []
        for orient in orientations:
            if orient == 'horizontal':
                cur_w = max(pw, ph)
                cur_h = min(pw, ph)
                rot = 0 if pw >= ph else 90
            elif orient == 'vertical':
                cur_w = min(pw, ph)
                cur_h = max(pw, ph)
                rot = 90 if pw >= ph else 0
            else:
                cur_w, cur_h, rot = pw, ph, 0
            candidate_configs.append((cur_w, cur_h, rot))

        for cur_w, cur_h, rot in candidate_configs:
            if cur_w > (self.max_ox - self.min_ox) or cur_h > (self.max_oy - self.min_oy):
                continue

            y_steps = 45
            step_val = (self.max_oy - self.min_oy - 2.0*self.margin - cur_h)
            if step_val < 0: continue
            step_y = max(4.0, step_val / float(y_steps))

            cand_gys = []
            cy_val = self.min_oy + self.margin
            while cy_val + cur_h + self.margin <= self.max_oy:
                cand_gys.append(cy_val)
                cy_val += step_y

            for pr in self.placed:
                cand_gys.append(pr['gy'] + pr['h'] + self.gap)
                cand_gys.append(pr['gy'] - cur_h - self.gap)

            cand_gys = sorted(list(set([y for y in cand_gys if self.min_oy + self.margin <= y <= self.max_oy - cur_h - self.margin])))

            for gy in cand_gys:
                if self.type == 'circle':
                    dy1 = abs(gy - self.cy)
                    dy2 = abs(gy + cur_h - self.cy)
                    max_dy = max(dy1, dy2)
                    if max_dy > self.eff_R: continue
                    half_w = math.sqrt(max(0.0, self.eff_R**2 - max_dy**2))
                    xl = self.cx - half_w
                    xr = self.cx + half_w
                else:
                    rng1 = get_poly_x_range(self.outer_pts, gy)
                    rng2 = get_poly_x_range(self.outer_pts, gy + cur_h)
                    if rng1 is None or rng2 is None: continue
                    xl = max(rng1[0], rng2[0]) + self.margin
                    xr = min(rng1[1], rng2[1]) - self.margin

                if xl + cur_w > xr: continue

                if align_pref == 'center':
                    cand_xs = [(xl + xr - cur_w)*0.5, xl, xr - cur_w]
                elif align_pref == 'right':
                    cand_xs = [xr - cur_w, (xl + xr - cur_w)*0.5, xl]
                else:
                    cand_xs = [xl, (xl + xr - cur_w)*0.5, xr - cur_w]

                for pr in self.placed:
                    cand_xs.append(pr['gx'] + pr['w'] + self.gap)
                    cand_xs.append(pr['gx'] - cur_w - self.gap)
                    cand_xs.append(pr['gx'])
                    cand_xs.append(pr['gx'] + pr['w'] - cur_w)

                for gx in cand_xs:
                    if gx < xl or gx + cur_w > xr: continue
                    if not self.is_inside(gx, gy, cur_w, cur_h): continue
                    col = False
                    for pr in self.placed:
                        if not (gx + cur_w + self.gap <= pr['gx'] or
                                pr['gx'] + pr['w'] + self.gap <= gx or
                                gy + cur_h + self.gap <= pr['gy'] or
                                pr['gy'] + pr['h'] + self.gap <= gy):
                            col = True; break
                    if col: continue

                    if align_pref == 'center':
                        s = (gy - self.min_oy)*10000.0 + abs(gx - (self.min_ox + self.max_ox - cur_w)*0.5)
                    elif align_pref == 'right':
                        s = (gy - self.min_oy)*10000.0 + (self.max_ox - (gx + cur_w))
                    else:
                        s = (gy - self.min_oy)*10000.0 + (gx - self.min_ox)

                    if s < best_score:
                        best_score = s
                        best_node = (gx, gy, cur_w, cur_h, rot)

        return best_node, best_score

    def place_part(self, node, part_id, orig_area):
        gx, gy, w, h, rot = node
        entry = {
            'id': part_id,
            'container_id': self.id,
            'gx': gx,
            'gy': gy,
            'w': w,
            'h': h,
            'rot': rot,
            'area': orig_area
        }
        self.placed.append(entry)
        return entry

def solve_multi_strategy(args):
    """
    Evaluates multi-container packing strategy across parallel configurations.
    Applies Feature-Aware Container Matching, Bigness Priority, and Multi-Bin Search.
    """
    strategy_id, parts_data, containers_data, gap, margin = args

    containers = list(containers_data)
    parts = list(parts_data)
    engs = {c['id']: NestingContainer(c, gap=gap, margin=margin) for c in containers}

    align = ['left', 'center', 'right', 'left'][strategy_id % 4]
    w_dim = [500.0, 1000.0, 250.0, 750.0][strategy_id % 4]

    # Special features
    very_long_parts = [p for p in parts if max(p['w'], p['h']) > 1000.0]
    huge_plates = [p for p in parts if float(p.get('area', p['w']*p['h'])) > 300000.0 and p not in very_long_parts]

    wide_containers = []
    for c in containers:
        if c['type'] == 'polygon':
            b = c['outer_bbox']
            if (b[2] - b[0]) > 1100.0: wide_containers.append(c['id'])
        else:
            if c['circle_radius']*2.0 > 1100.0: wide_containers.append(c['id'])

    circle_containers = [c for c in containers if c['type'] == 'circle']
    circle_containers.sort(key=lambda c: c['circle_radius'], reverse=True)

    placed_plan = []
    unplaced = []

    # 1. Very long parts -> wide containers (e.g. Container 0)
    for p in very_long_parts:
        pw, ph = float(p['w']), float(p['h'])
        placed = False
        for cid in wide_containers:
            n, _ = engs[cid].find_best_position(pw, ph, orientations=['horizontal'])
            if n:
                item = engs[cid].place_part(n, p['id'], float(p.get('area', pw*ph)))
                placed_plan.append(item)
                placed = True
                break
        if not placed:
            unplaced.append(p)

    # 2. Huge plates -> largest circle containers (Container 1 & 4) horizontally
    orients_plate = ['horizontal'] if (strategy_id % 2 == 0) else ['horizontal', 'vertical']
    for p in huge_plates:
        pw, ph = float(p['w']), float(p['h'])
        placed = False
        for c in circle_containers:
            cid = c['id']
            n, _ = engs[cid].find_best_position(pw, ph, orientations=orients_plate, align_pref=align)
            if n:
                item = engs[cid].place_part(n, p['id'], float(p.get('area', pw*ph)))
                placed_plan.append(item)
                placed = True
                break
        if not placed:
            unplaced.append(p)

    # 3. Medium plates & Long bars
    already_placed_ids = set(p['id'] for p in placed_plan) | set(p['id'] for p in unplaced)
    long_bars = [p for p in parts if p['id'] not in already_placed_ids and (max(p['w'], p['h'])/max(1.0, min(p['w'], p['h'])) > 2.0 or float(p.get('area', 0)) > 150000.0)]
    
    if strategy_id % 3 == 1:
        rng = random.Random(strategy_id * 31 + 7)
        rng.shuffle(long_bars)

    # Circle containers preferred for long bars
    c_order_bars = [c['id'] for c in circle_containers] + [c['id'] for c in containers if c['type'] != 'circle']
    for p in long_bars:
        pw, ph = float(p['w']), float(p['h'])
        area = float(p.get('area', pw*ph))
        best_eng, best_node, best_s = None, None, float('inf')
        for cid in c_order_bars:
            n, s = engs[cid].find_best_position(pw, ph, orientations=['horizontal'], align_pref=align)
            if n and s < best_s:
                best_s, best_eng, best_node = s, engs[cid], n
        if best_eng:
            item = best_eng.place_part(best_node, p['id'], area)
            placed_plan.append(item)
        else:
            unplaced.append(p)

    # 4. Small & Medium parts -> across all containers
    already_placed_ids = set(p['id'] for p in placed_plan) | set(p['id'] for p in unplaced)
    small_parts = [p for p in parts if p['id'] not in already_placed_ids]
    small_parts.sort(key=lambda p: compute_bigness(p, w_dim), reverse=True)

    if strategy_id % 2 == 1:
        rng = random.Random(strategy_id * 101 + 13)
        rng.shuffle(small_parts)

    for p in small_parts:
        pw, ph = float(p['w']), float(p['h'])
        area = float(p.get('area', pw*ph))
        best_eng, best_node, best_s = None, None, float('inf')
        for cid in sorted(engs.keys()):
            n, s = engs[cid].find_best_position(pw, ph, orientations=['horizontal', 'vertical'], align_pref=align)
            if n and s < best_s:
                best_s, best_eng, best_node = s, engs[cid], n
        if best_eng:
            item = best_eng.place_part(best_node, p['id'], area)
            placed_plan.append(item)
        else:
            unplaced.append(p)

    # Metrics & Fitness
    used_containers = set(p['container_id'] for p in placed_plan)
    placed_count = len(placed_plan)
    total_parts = len(parts_data)
    placed_area = sum(p['area'] for p in placed_plan)
    used_c_area = sum(float(c.get('area', 1.0)) for c in containers_data if c['id'] in used_containers)
    if used_c_area <= 0.0: used_c_area = 1.0
    yield_pct = (placed_area / used_c_area) * 100.0

    fitness = (placed_count * 10000000.0) + (placed_area * 100.0) + (yield_pct * 1000.0) - (len(used_containers) * 10000.0)

    return {
        'strategy_id': strategy_id,
        'placed_count': placed_count,
        'total_parts': total_parts,
        'placed_area': placed_area,
        'yield_pct': yield_pct,
        'used_containers': list(used_containers),
        'fitness': fitness,
        'plan': placed_plan
    }

def run_nesting(input_path, output_lsp_path, output_json_path=None):
    t0 = time.time()
    try:
        with open(input_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        gap = float(data.get('gap', 3.0))
        margin = float(data.get('margin', 2.0))
        parts_data = data['parts']
        total_parts = len(parts_data)

        if 'containers' in data and isinstance(data['containers'], list):
            containers_data = data['containers']
        else:
            containers_data = [{
                'id': 0,
                'type': data.get('container_type', 'polygon'),
                'outer_bbox': data.get('outer_bbox', [0, 0, 1000, 1000]),
                'outer_polygon': data.get('outer_polygon', []),
                'circle_center': data.get('circle_center', [0, 0]),
                'circle_radius': float(data.get('circle_radius', 0.0)),
                'area': float(data.get('area', 1000000.0))
            }]

        num_containers = len(containers_data)
        num_cpus = cpu_count()
        num_strategies = max(num_cpus, 32)

        tasks = [
            (s_id, parts_data, containers_data, gap, margin)
            for s_id in range(num_strategies)
        ]

        with Pool(processes=min(num_cpus, 20)) as pool:
            results = pool.map(solve_multi_strategy, tasks)
        mode_str = f"Feature-Aware Multi-Bin Parallel ({min(num_cpus, 20)} CPUs, {num_strategies} Strategies, {num_containers} Containers)"

        best_res = max(results, key=lambda r: (r['placed_count'], r['fitness']))
        elapsed = time.time() - t0

        is_success = (best_res['placed_count'] == total_parts)
        status_str = "SUCCESS" if is_success else "PARTIAL"

        msg = (f"MBBN Multi-Container Nesting: {best_res['placed_count']}/{total_parts} parts placed "
               f"in {elapsed*1000.0:.1f}ms [Yield: {best_res['yield_pct']:.1f}% | Containers Used: {len(best_res['used_containers'])}/{num_containers} | {mode_str}] "
               f"(Best Strategy #{best_res['strategy_id']})")

        plan_lines = []
        for item in best_res['plan']:
            plan_lines.append(f"  ({item['id']} {item['container_id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f} {item['rot']})")

        lsp_content = f""";;; MBBN NESTING RESULT GENERATED BY PYTHON ENGINE
;;; Version: v3.2 (2026-10-23 09:15:00)
;;; Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}
;;; Elapsed: {elapsed*1000.0:.2f} ms | Mode: {mode_str}
(setq MBB_NESTING_STATUS "{status_str}")
(setq MBB_NESTING_MSG "{msg}")
(setq MBB_NESTING_COUNT {best_res['placed_count']})
(setq MBB_TOTAL_YIELD {best_res['yield_pct']:.2f})
(setq MBB_CONTAINERS_USED {len(best_res['used_containers'])})
(setq MBB_NESTING_PLAN '(
{chr(10).join(plan_lines)}
))
(princ "\n>> {msg}")
(princ)
"""
        with open(output_lsp_path, 'w', encoding='cp949', errors='replace') as f:
            f.write(lsp_content)

        if output_json_path:
            out_json_data = {
                'status': status_str,
                'message': msg,
                'elapsed_ms': elapsed * 1000.0,
                'placed_count': best_res['placed_count'],
                'total_parts': total_parts,
                'yield_pct': best_res['yield_pct'],
                'used_containers': best_res['used_containers'],
                'plan': best_res['plan']
            }
            with open(output_json_path, 'w', encoding='utf-8') as f:
                json.dump(out_json_data, f, indent=2)

        print(msg)
        return is_success

    except Exception as e:
        elapsed = time.time() - t0
        err_msg = f"MBBN Python Error: {str(e)}"
        print(err_msg, file=sys.stderr)
        lsp_content = f""";;; MBBN NESTING ERROR
(setq MBB_NESTING_STATUS "ERROR")
(setq MBB_NESTING_MSG "{err_msg}")
(setq MBB_NESTING_PLAN nil)
(princ "\n>> {err_msg}")
(princ)
"""
        try:
            with open(output_lsp_path, 'w', encoding='cp949', errors='replace') as f:
                f.write(lsp_content)
        except Exception:
            pass
        return False

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python mbb_nesting.py <input_json> <output_lsp> [<output_json>]")
        sys.exit(1)

    in_json = sys.argv[1]
    out_lsp = sys.argv[2]
    out_json = sys.argv[3] if len(sys.argv) > 3 else None

    run_nesting(in_json, out_lsp, out_json)
