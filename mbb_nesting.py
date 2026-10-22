# -*- coding: utf-8 -*-
"""
=============================================================================
[MBBN World-Class 2D Nesting Multi-Core Engine]
Program: MBB Multi-Container Yield Optimizer (Python 3.14 / Shapely / GEOS)
Version: v3.1 (2026-10-23 08:45:00)
Architecture: Jukka Jylänki MaxRects (BSSF/BAF/BLSF/CP) + Global Multi-Bin Best-Fit
              + Exact Buffer Containment + Wall-Adaptive Curved Tangents + 20-Core Massively Parallel
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
    Large parts are nested first, while small parts act as cavity fillers.
    """
    w = float(p['w'])
    h = float(p['h'])
    area = float(p.get('area', w * h))
    max_d = max(w, h)
    min_d = max(1.0, min(w, h))
    aspect = max_d / min_d
    return area + (max_d * weight_dim) + (aspect * weight_aspect)

class MaxRectsContainer:
    """
    High-performance 2D MaxRects container engine for Circle or Polygon shapes.
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
            self.prep_inner = None
            self.eff_R = max(0.0, self.R - self.margin)
            self.eff_R_sq = self.eff_R * self.eff_R + 1e-4
        else:
            self.outer_pts = c_data['outer_polygon']
            self.min_ox, self.min_oy, self.max_ox, self.max_oy = c_data['outer_bbox']
            self.prep_inner = None
            if HAS_SHAPELY:
                poly = Polygon(self.outer_pts)
                if not poly.is_valid:
                    poly = poly.buffer(0)
                inner_poly = poly.buffer(-self.margin)
                if inner_poly.is_empty:
                    inner_poly = poly
                self.prep_inner = prep(inner_poly)

        init_x = self.min_ox + self.margin
        init_y = self.min_oy + self.margin
        init_w = max(0.0, (self.max_ox - self.min_ox) - 2.0 * self.margin)
        init_h = max(0.0, (self.max_oy - self.min_oy) - 2.0 * self.margin)
        self.free_rects = [[init_x, init_y, init_w, init_h]]

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

    def prune_free_list(self):
        """Removes free rectangles that are completely subsumed by another."""
        i = 0
        while i < len(self.free_rects):
            r1 = self.free_rects[i]
            j = i + 1
            while j < len(self.free_rects):
                r2 = self.free_rects[j]
                if (r1[0] >= r2[0] and r1[1] >= r2[1] and
                    r1[0] + r1[2] <= r2[0] + r2[2] and
                    r1[1] + r1[3] <= r2[1] + r2[3]):
                    self.free_rects.pop(i)
                    i -= 1
                    break
                if (r2[0] >= r1[0] and r2[1] >= r1[1] and
                    r2[0] + r2[2] <= r1[0] + r1[2] and
                    r2[1] + r2[3] <= r1[1] + r1[3]):
                    self.free_rects.pop(j)
                    continue
                j += 1
            i += 1

    def split_free_rect(self, free_rect, placed_rect):
        """Splits free_rect against placed_rect with inter-part gap."""
        fx, fy, fw, fh = free_rect
        px, py, pw, ph = placed_rect
        gx = px - self.gap
        gy = py - self.gap
        gw = pw + 2.0 * self.gap
        gh = ph + 2.0 * self.gap

        if (gx >= fx + fw or gx + gw <= fx or
            gy >= fy + fh or gy + gh <= fy):
            return [free_rect]

        new_rects = []
        if gx > fx and gx < fx + fw: new_rects.append([fx, fy, gx - fx, fh])
        if gx + gw < fx + fw and gx + gw > fx: new_rects.append([gx + gw, fy, fx + fw - (gx + gw), fh])
        if gy > fy and gy < fy + fh: new_rects.append([fx, fy, fw, gy - fy])
        if gy + gh < fy + fh and gy + gh > fy: new_rects.append([fx, gy + gh, fw, fy + fh - (gy + gh)])
        return new_rects

    def find_best_position(self, pw, ph, method='BL', rot_policy='free', align_pref='left'):
        """
        Evaluates candidate placement using Jukka Jylänki's MaxRects heuristics.
        Supports BSSF, BAF, BLSF, CP, BL.
        """
        aspect = max(pw, ph) / max(1.0, min(pw, ph))

        if rot_policy == 90:
            candidate_rots = [(ph, pw, 90)] if aspect > 1.25 else [(ph, pw, 90), (pw, ph, 0)]
        elif rot_policy == 0:
            candidate_rots = [(pw, ph, 0)] if aspect > 1.25 else [(pw, ph, 0), (ph, pw, 90)]
        else: # 'free'
            candidate_rots = [(ph, pw, 90), (pw, ph, 0)] if aspect > 1.25 else [(pw, ph, 0), (ph, pw, 90)]

        best_score1 = float('inf')
        best_score2 = float('inf')
        best_node = None

        for cur_w, cur_h, rot in candidate_rots:
            if cur_w > (self.max_ox - self.min_ox) or cur_h > (self.max_oy - self.min_oy):
                continue

            # 1. Circle-Specific Tangent Chord Search (Wall-Adaptive)
            if self.type == 'circle':
                y_steps = 80
                step_val = (2.0 * self.eff_R) / y_steps
                for yi in range(y_steps + 1):
                    gy_val = (self.cy - self.eff_R) + yi * step_val
                    if gy_val + cur_h > self.cy + self.eff_R: break
                    dy1 = abs(gy_val - self.cy)
                    dy2 = abs(gy_val + cur_h - self.cy)
                    max_dy = max(dy1, dy2)
                    if max_dy > self.eff_R: continue
                    half_w = math.sqrt(max(0.0, self.eff_R**2 - max_dy**2))
                    xl = self.cx - half_w
                    xr = self.cx + half_w
                    if xl + cur_w > xr: continue

                    if align_pref == 'center':
                        cand_xs = [(xl + xr - cur_w)*0.5, xl, xr - cur_w]
                    elif align_pref == 'right':
                        cand_xs = [xr - cur_w, (xl + xr - cur_w)*0.5, xl]
                    else: # 'left'
                        cand_xs = [xl, (xl + xr - cur_w)*0.5, xr - cur_w]

                    for pr in self.placed:
                        cand_xs.append(pr['gx'] + pr['w'] + self.gap)
                        cand_xs.append(pr['gx'] - cur_w - self.gap)

                    for gx_val in cand_xs:
                        if gx_val < xl or gx_val + cur_w > xr: continue
                        if not self.is_inside(gx_val, gy_val, cur_w, cur_h): continue
                        col = False
                        for pr in self.placed:
                            if not (gx_val + cur_w + self.gap <= pr['gx'] or
                                    pr['gx'] + pr['w'] + self.gap <= gx_val or
                                    gy_val + cur_h + self.gap <= pr['gy'] or
                                    pr['gy'] + pr['h'] + self.gap <= gy_val):
                                col = True; break
                        if col: continue

                        if align_pref == 'center':
                            s1 = (gy_val - self.min_oy)*10000.0 + abs(gx_val - (self.cx - cur_w*0.5))
                        elif align_pref == 'right':
                            s1 = (gy_val - self.min_oy)*10000.0 + (self.max_ox - (gx_val + cur_w))
                        else:
                            s1 = (gy_val - self.min_oy)*10000.0 + (gx_val - self.min_ox)
                        s2 = 0.0

                        if s1 < best_score1:
                            best_score1 = s1
                            best_score2 = s2
                            best_node = (gx_val, gy_val, cur_w, cur_h, rot)

            # 2. MaxRects Free Rectangles Search (Polygon & Box Containers)
            else:
                for fr in self.free_rects:
                    if fr[2] >= cur_w and fr[3] >= cur_h:
                        if align_pref == 'center':
                            cand_anchors = [(fr[0] + (fr[2] - cur_w)*0.5, fr[1]), (fr[0], fr[1]), (fr[0] + fr[2] - cur_w, fr[1])]
                        elif align_pref == 'right':
                            cand_anchors = [(fr[0] + fr[2] - cur_w, fr[1]), (fr[0], fr[1]), (fr[0] + (fr[2] - cur_w)*0.5, fr[1])]
                        else:
                            cand_anchors = [(fr[0], fr[1]), (fr[0] + (fr[2] - cur_w)*0.5, fr[1]), (fr[0] + fr[2] - cur_w, fr[1])]

                        cand_anchors.append((fr[0], fr[1] + fr[3] - cur_h))

                        for gx, gy in cand_anchors:
                            if not self.is_inside(gx, gy, cur_w, cur_h): continue
                            col = False
                            for pr in self.placed:
                                if not (gx + cur_w + self.gap <= pr['gx'] or
                                        pr['gx'] + pr['w'] + self.gap <= gx or
                                        gy + cur_h + self.gap <= pr['gy'] or
                                        pr['gy'] + pr['h'] + self.gap <= gy):
                                    col = True; break
                            if col: continue

                            rem_w = fr[2] - cur_w
                            rem_h = fr[3] - cur_h
                            if method == 'BSSF':
                                s1 = min(rem_w, rem_h)
                                s2 = (gy - self.min_oy)*1000.0 + (gx - self.min_ox)
                            elif method == 'BAF':
                                s1 = (fr[2]*fr[3]) - (cur_w*cur_h)
                                s2 = min(rem_w, rem_h)
                            elif method == 'CP':
                                contact = 0.0
                                if abs(gx - self.min_ox - self.margin) < 2.0: contact += cur_h
                                if abs(gy - self.min_oy - self.margin) < 2.0: contact += cur_w
                                s1 = -contact
                                s2 = (gy - self.min_oy)*1000.0 + (gx - self.min_ox)
                            else: # BL
                                s1 = (gy - self.min_oy)*10000.0 + (gx - self.min_ox)
                                s2 = 0.0

                            if s1 < best_score1 or (s1 == best_score1 and s2 < best_score2):
                                best_score1 = s1
                                best_score2 = s2
                                best_node = (gx, gy, cur_w, cur_h, rot)

        return best_node, best_score1

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
        placed_rect = [gx, gy, w, h]
        new_free = []
        for fr in self.free_rects:
            splits = self.split_free_rect(fr, placed_rect)
            new_free.extend(splits)
        self.free_rects = new_free
        self.prune_free_list()
        return entry

    def rebuild_free_list(self):
        init_x = self.min_ox + self.margin
        init_y = self.min_oy + self.margin
        init_w = max(0.0, (self.max_ox - self.min_ox) - 2.0 * self.margin)
        init_h = max(0.0, (self.max_oy - self.min_oy) - 2.0 * self.margin)
        self.free_rects = [[init_x, init_y, init_w, init_h]]
        for pr in self.placed:
            placed_rect = [pr['gx'], pr['gy'], pr['w'], pr['h']]
            new_free = []
            for fr in self.free_rects:
                splits = self.split_free_rect(fr, placed_rect)
                new_free.extend(splits)
            self.free_rects = new_free
            self.prune_free_list()

def solve_multi_strategy(args):
    """
    Evaluates multi-container packing strategy across 64 combinations.
    Combines Jukka Jylänki MaxRects heuristics, Global Multi-Bin Best-Fit,
    Bigness Priority, and Backtracking Ejection Chain.
    """
    strategy_id, parts_data, containers_data, gap, margin = args

    # Clone containers & parts
    c_raw_list = list(containers_data)
    parts = list(parts_data)

    # Strategy Parameters
    heuristic_methods = ['BL', 'BSSF', 'BAF', 'CP']
    chosen_method = heuristic_methods[strategy_id % len(heuristic_methods)]

    strat_type = (strategy_id // len(heuristic_methods)) % 16

    if strat_type in [0, 1, 2, 3]:
        # [Archetype 1..4]: Left-Hug Horizontal Priority (Bigness Descending)
        w_dim = [500.0, 1000.0, 250.0, 750.0][strat_type]
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=w_dim), reverse=True)
        rot_policy = 90
        align_pref = 'left'
        mode = 'global_best_fit' if (strat_type % 2 == 0) else 'sequential'
        if mode == 'sequential':
            c_raw_list.sort(key=lambda c: float(c.get('area', 0.0)), reverse=True)

    elif strat_type in [4, 5, 6]:
        # [Archetype 5..7]: Right-Hug Horizontal Priority
        w_dim = [500.0, 1000.0, 300.0][strat_type - 4]
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=w_dim), reverse=True)
        rot_policy = 90
        align_pref = 'right'
        mode = 'global_best_fit'

    elif strat_type in [7, 8, 9]:
        # [Archetype 8..10]: Center-Stack Alignment
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=500.0), reverse=True)
        rot_policy = 90 if (strat_type != 9) else 'free'
        align_pref = 'center'
        mode = 'global_best_fit'

    elif strat_type in [10, 11]:
        # [Archetype 11..12]: 2-Tier Hierarchical Infill
        b_scores = [compute_bigness(p, 500.0) for p in parts]
        median_b = sorted(b_scores)[len(b_scores)//2] if b_scores else 0.0
        tier1 = [p for p in parts if compute_bigness(p, 500.0) >= median_b]
        tier2 = [p for p in parts if compute_bigness(p, 500.0) < median_b]
        tier1.sort(key=lambda p: compute_bigness(p, 500.0), reverse=True)
        tier2.sort(key=lambda p: compute_bigness(p, 500.0), reverse=True)
        parts = tier1 + tier2
        rot_policy = 90
        align_pref = 'left'
        mode = 'global_best_fit'

    else:
        # [Archetypes 13..16]: Stochastic Local Search
        rng = random.Random(strategy_id * 1013 + 57)
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=500.0), reverse=True)
        for i in range(len(parts) - 1):
            if rng.random() < 0.35:
                parts[i], parts[i + 1] = parts[i + 1], parts[i]
        rot_policy = 90 if (strategy_id % 2 == 0) else 'free'
        align_pref = 'left' if (strategy_id % 3 == 0) else 'center'
        mode = 'global_best_fit'

    # Build container engines
    engines = [MaxRectsContainer(c, gap=gap, margin=margin) for c in c_raw_list]
    total_plan = []
    unplaced = []

    if mode == 'global_best_fit':
        for p in parts:
            pw = float(p['w'])
            ph = float(p['h'])
            p_area = float(p.get('area', pw * ph))

            best_engine = None
            best_node = None
            global_best_score = float('inf')

            for eng in engines:
                node, score = eng.find_best_position(pw, ph, method=chosen_method, rot_policy=rot_policy, align_pref=align_pref)
                if node is not None:
                    fill_factor = 0.95 if len(eng.placed) > 0 else 1.0
                    adj_score = score * fill_factor
                    if adj_score < global_best_score:
                        global_best_score = adj_score
                        best_engine = eng
                        best_node = node

            if best_engine is not None:
                plan_item = best_engine.place_part(best_node, p['id'], p_area)
                total_plan.append(plan_item)
            else:
                unplaced.append(p)

    else: # sequential
        remaining = list(parts)
        for eng in engines:
            if not remaining: break
            still_unplaced = []
            for p in remaining:
                pw = float(p['w'])
                ph = float(p['h'])
                p_area = float(p.get('area', pw * ph))
                node, score = eng.find_best_position(pw, ph, method=chosen_method, rot_policy=rot_policy, align_pref=align_pref)
                if node is not None:
                    plan_item = eng.place_part(node, p['id'], p_area)
                    total_plan.append(plan_item)
                else:
                    still_unplaced.append(p)
            remaining = still_unplaced
        unplaced = remaining

    # Backtracking / Ejection Chain Repacking
    if unplaced and len(total_plan) > 0 and (strategy_id % 2 == 0):
        unplaced.sort(key=lambda p: compute_bigness(p), reverse=True)
        for u in list(unplaced):
            u_w, u_h = float(u['w']), float(u['h'])
            u_area = float(u.get('area', u_w * u_h))
            placed_for_u = False
            for eng in engines:
                small_placed = [pr for pr in eng.placed if pr['area'] < u_area * 0.7]
                if not small_placed: continue
                small_placed.sort(key=lambda pr: pr['area'])
                for cand_pop in small_placed[:2]:
                    eng.placed.remove(cand_pop)
                    eng.rebuild_free_list()
                    node, score = eng.find_best_position(u_w, u_h, method=chosen_method, rot_policy=rot_policy, align_pref=align_pref)
                    if node is not None:
                        new_item = eng.place_part(node, u['id'], u_area)
                        total_plan = [pr for pr in total_plan if pr['id'] != cand_pop['id']] + [new_item]
                        unplaced.remove(u)
                        unplaced.append({'id': cand_pop['id'], 'w': cand_pop['w'], 'h': cand_pop['h'], 'area': cand_pop['area']})
                        placed_for_u = True
                        break
                    else:
                        eng.placed.append(cand_pop)
                        eng.rebuild_free_list()
                if placed_for_u: break

    # Metrics & Fitness
    used_containers = set(p['container_id'] for p in total_plan)
    placed_count = len(total_plan)
    total_parts = len(parts_data)
    placed_area = sum(p['area'] for p in total_plan)
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
        'plan': total_plan
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
        num_strategies = max(num_cpus * 3, 64)

        tasks = [
            (s_id, parts_data, containers_data, gap, margin)
            for s_id in range(num_strategies)
        ]

        # Multi-Core Multiprocessing Execution
        with Pool(processes=min(num_cpus, 20)) as pool:
            results = pool.map(solve_multi_strategy, tasks)
        mode_str = f"MaxRects Multi-Core Multiprocessing ({min(num_cpus, 20)} CPUs, {num_strategies} Strategies, {num_containers} Containers)"

        best_res = max(results, key=lambda r: (r['placed_count'], r['fitness']))
        elapsed = time.time() - t0

        is_success = (best_res['placed_count'] == total_parts)
        status_str = "SUCCESS" if is_success else "PARTIAL"

        msg = (f"MBBN MaxRects Global Nesting: {best_res['placed_count']}/{total_parts} parts placed "
               f"in {elapsed*1000.0:.1f}ms [Yield: {best_res['yield_pct']:.1f}% | Containers Used: {len(best_res['used_containers'])}/{num_containers} | {mode_str}] "
               f"(Best Strategy #{best_res['strategy_id']})")

        plan_lines = []
        for item in best_res['plan']:
            plan_lines.append(f"  ({item['id']} {item['container_id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f} {item['rot']})")

        lsp_content = f""";;; MBBN MAXRECTS GLOBAL NESTING RESULT GENERATED BY PYTHON ENGINE
;;; Version: v3.1 (2026-10-23 08:45:00)
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
