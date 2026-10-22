# -*- coding: utf-8 -*-
"""
=============================================================================
[MBBN Ultra-Fast 2D Nesting Multi-Core Engine]
Program: MBB Nesting Optimizer (Python 3.14 Multi-Core / Shapely Engine)
Version: v2.4 (2026-10-22 21:27:00)
Architecture: Hybrid (Direct Fast Path + 20-Core Multiprocessing BLF Anchor)
=============================================================================
"""

import sys
import os
import json
import time
import math
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
        p1x, p2x = p2x, p2y
    return inside

def is_box_inside_poly(gx, gy, w, h, poly_pts, margin=0.0, prep_poly=None):
    """Check if rectangle [gx, gy, gx+w, gy+h] with margin is completely inside poly"""
    x1, y1 = gx - margin, gy - margin
    x2, y2 = gx + w + margin, gy + h + margin

    if HAS_SHAPELY and prep_poly is not None:
        b = box(x1, y1, x2, y2)
        return prep_poly.contains(b)

    # Fallback sampling
    test_pts = [
        (x1, y1), (x2, y1), (x2, y2), (x1, y2),
        ((x1+x2)*0.5, y1), ((x1+x2)*0.5, y2),
        (x1, (y1+y2)*0.5), (x2, (y1+y2)*0.5),
        ((x1+x2)*0.5, (y1+y2)*0.5)
    ]
    for px, py in test_pts:
        if not is_point_in_poly(px, py, poly_pts):
            return False
    return True

def rects_overlap(r1, r2, gap=0.0):
    """Check if two rectangles overlap with gap: r = (x, y, w, h)"""
    x1, y1, w1, h1 = r1
    x2, y2, w2, h2 = r2
    return not (x1 + w1 + gap <= x2 or
                x1 >= x2 + w2 + gap or
                y1 + h1 + gap <= y2 or
                y1 >= y2 + h2 + gap)

def solve_nesting_strategy(args):
    """
    Evaluates a specific sorting and anchor placement strategy.
    Can be run in parallel or sequentially.
    """
    strategy_id, parts_data, outer_pts, outer_bbox, gap, margin = args

    min_ox, min_oy, max_ox, max_oy = outer_bbox

    prep_poly = None
    if HAS_SHAPELY:
        poly = Polygon(outer_pts)
        if not poly.is_valid:
            poly = poly.buffer(0)
        prep_poly = prep(poly)

    parts = list(parts_data)
    if strategy_id == 0:
        parts.sort(key=lambda p: p['w'] * p['h'], reverse=True)
        rot_prefs = [0, 90]
    elif strategy_id == 1:
        parts.sort(key=lambda p: p['w'] * p['h'], reverse=True)
        rot_prefs = [90, 0]
    elif strategy_id == 2:
        parts.sort(key=lambda p: max(p['w'], p['h']), reverse=True)
        rot_prefs = [0, 90]
    elif strategy_id == 3:
        parts.sort(key=lambda p: max(p['w'], p['h']), reverse=True)
        rot_prefs = [90, 0]
    elif strategy_id == 4:
        parts.sort(key=lambda p: p['h'], reverse=True)
        rot_prefs = [0, 90]
    elif strategy_id == 5:
        parts.sort(key=lambda p: p['w'], reverse=True)
        rot_prefs = [0, 90]
    elif strategy_id == 6:
        parts.sort(key=lambda p: 2 * (p['w'] + p['h']), reverse=True)
        rot_prefs = [0, 90]
    elif strategy_id == 7:
        parts.sort(key=lambda p: p['w'] * p['h'], reverse=True)
        rot_prefs = [0]
    elif strategy_id == 8:
        parts.sort(key=lambda p: p['w'] * p['h'], reverse=True)
        rot_prefs = [90]
    else:
        import random
        rng = random.Random(strategy_id * 101 + 13)
        rng.shuffle(parts)
        rot_prefs = [0, 90] if (strategy_id % 2 == 0) else [90, 0]

    placed = []
    plan = []

    # Seed anchors
    anchors = [(min_ox + margin, min_oy + margin)]

    # Add side and step anchors
    y_step = max(15.0, (max_oy - min_oy) / 40.0)
    cy = min_oy + margin
    while cy <= max_oy - margin:
        anchors.append((min_ox + margin, cy))
        cy += y_step

    for p in parts:
        pid = p['id']
        pw = float(p['w'])
        ph = float(p['h'])

        best_anchor = None
        best_rot = 0
        best_w, best_h = pw, ph
        best_score = float('inf')

        for rot in rot_prefs:
            cur_w, cur_h = (ph, pw) if rot == 90 else (pw, ph)
            if cur_w > (max_ox - min_ox) or cur_h > (max_oy - min_oy):
                continue

            for ax, ay in anchors:
                if ax + cur_w + margin > max_ox or ay + cur_h + margin > max_oy:
                    continue

                cur_r = (ax, ay, cur_w, cur_h)

                overlap = False
                for pr in placed:
                    if rects_overlap(cur_r, pr, gap):
                        overlap = True
                        break
                if overlap:
                    continue

                if not is_box_inside_poly(ax, ay, cur_w, cur_h, outer_pts, margin, prep_poly):
                    continue

                # BLF score
                if strategy_id % 3 == 0:
                    score = ay * 10000.0 + ax
                elif strategy_id % 3 == 1:
                    score = ax * 10000.0 + ay
                else:
                    score = (ay - min_oy)**2 + (ax - min_ox)**2

                if score < best_score:
                    best_score = score
                    best_anchor = (ax, ay)
                    best_rot = rot
                    best_w, best_h = cur_w, cur_h

        if best_anchor is not None:
            gx, gy = best_anchor
            placed.append((gx, gy, best_w, best_h))
            plan.append({
                'id': pid,
                'gx': gx,
                'gy': gy,
                'w': best_w,
                'h': best_h,
                'rot': best_rot
            })
            new_anchors = [
                (gx + best_w + gap, gy),
                (gx, gy + best_h + gap),
                (gx + best_w + gap, gy + best_h + gap),
                (min_ox + margin, gy + best_h + gap)
            ]
            for na in new_anchors:
                if na[0] <= max_ox and na[1] <= max_oy and na not in anchors:
                    anchors.append(na)
            anchors.sort(key=lambda a: a[1] * 1000.0 + a[0])

    total_parts = len(parts_data)
    placed_count = len(plan)
    if placed_count == total_parts:
        p_min_x = min(r[0] for r in placed)
        p_max_x = max(r[0] + r[2] for r in placed)
        p_min_y = min(r[1] for r in placed)
        p_max_y = max(r[1] + r[3] for r in placed)
        footprint_area = (p_max_x - p_min_x) * (p_max_y - p_min_y)
        fitness = 1000000.0 - footprint_area
    else:
        fitness = placed_count * 1000.0

    return {
        'strategy_id': strategy_id,
        'placed_count': placed_count,
        'total_parts': total_parts,
        'fitness': fitness,
        'plan': plan
    }

def run_nesting(input_path, output_lsp_path, output_json_path=None):
    t0 = time.time()
    try:
        with open(input_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        outer_pts = data['outer_polygon']
        outer_bbox = data['outer_bbox']
        gap = float(data.get('gap', 3.0))
        margin = float(data.get('margin', 2.0))
        parts_data = data['parts']
        total_parts = len(parts_data)

        num_cpus = cpu_count()
        num_strategies = max(num_cpus, 20)

        tasks = [
            (s_id, parts_data, outer_pts, outer_bbox, gap, margin)
            for s_id in range(num_strategies)
        ]

        # Adaptive execution: If small batch (<= 10 parts), run sequentially for instant ~30ms response
        # If larger batch, engage full multiprocessing pool across all cores
        if total_parts <= 10:
            results = [solve_nesting_strategy(t) for t in tasks]
            mode_str = f"Direct Fast-Path (20 Strategies, {total_parts} parts)"
        else:
            with Pool(processes=min(num_cpus, 20)) as pool:
                results = pool.map(solve_nesting_strategy, tasks)
            mode_str = f"Multi-Core Multiprocessing ({num_cpus} CPUs, {total_parts} parts)"

        best_res = max(results, key=lambda r: (r['placed_count'], r['fitness']))
        elapsed = time.time() - t0

        is_success = (best_res['placed_count'] == total_parts)
        status_str = "SUCCESS" if is_success else "PARTIAL"

        msg = (f"MBBN Nesting: {best_res['placed_count']}/{total_parts} parts placed "
               f"in {elapsed*1000.0:.1f}ms [{mode_str}] (Best Strategy #{best_res['strategy_id']})")

        plan_lines = []
        for item in best_res['plan']:
            plan_lines.append(f"  ({item['id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f} {item['rot']})")

        lsp_content = f""";;; MBBN NESTING RESULT GENERATED BY PYTHON ENGINE
;;; Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}
;;; Elapsed: {elapsed*1000.0:.2f} ms | Mode: {mode_str}
(setq MBB_NESTING_STATUS "{status_str}")
(setq MBB_NESTING_MSG "{msg}")
(setq MBB_NESTING_COUNT {best_res['placed_count']})
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
