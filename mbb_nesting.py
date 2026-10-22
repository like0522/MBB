# -*- coding: utf-8 -*-
"""
=============================================================================
[MBBN Ultra-Fast 2D Nesting Multi-Core Engine]
Program: MBB Nesting Optimizer (Python 3.14 Multi-Core / Shapely Engine)
Version: v2.7 (2026-10-22 22:25:00)
Architecture: Wall-Adaptive + Multi-Policy Rotation & Alignment Engine
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
        p1x, p1y = p2x, p2y
    return inside

def get_poly_x_range(poly, y):
    """Find left and right x-boundaries of polygon at height y"""
    xs = []
    n = len(poly)
    for i in range(n):
        p1 = poly[i]
        p2 = poly[(i + 1) % n]
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

def solve_nesting_strategy(args):
    """
    Evaluates a specific sorting, rotation policy, and alignment strategy.
    Supports Polygon containers and dedicated Circle/Pipe containers.
    """
    strategy_id, parts_data, outer_data, gap, margin = args

    is_circle = (outer_data.get('container_type') == 'circle')
    if is_circle:
        cx, cy = outer_data['circle_center']
        R = float(outer_data['circle_radius'])
        min_ox, min_oy = cx - R, cy - R
        max_ox, max_oy = cx + R, cy + R
        prep_poly = None

        def x_range_fn(y):
            dy = abs(y - cy)
            if dy > R:
                return None
            dx = math.sqrt(max(0.0, R * R - dy * dy))
            return (cx - dx, cx + dx)

        def is_inside_fn(gx, gy, w, h):
            m = margin
            for px, py in [(gx-m, gy-m), (gx+w+m, gy-m), (gx+w+m, gy+h+m), (gx-m, gy+h+m)]:
                if (px - cx) ** 2 + (py - cy) ** 2 > R * R + 1e-4:
                    return False
            return True
    else:
        outer_pts = outer_data['outer_polygon']
        min_ox, min_oy, max_ox, max_oy = outer_data['outer_bbox']
        prep_poly = None
        if HAS_SHAPELY:
            poly_obj = Polygon(outer_pts)
            if not poly_obj.is_valid:
                poly_obj = poly_obj.buffer(0)
            prep_poly = prep(poly_obj)

        def x_range_fn(y):
            return get_poly_x_range(outer_pts, y)

        def is_inside_fn(gx, gy, w, h):
            m = margin
            if HAS_SHAPELY and prep_poly is not None:
                b = box(gx - m, gy - m, gx + w + m, gy + h + m)
                return prep_poly.contains(b)

            pts_to_check = [
                (gx-m, gy-m), (gx+w+m, gy-m), (gx+w+m, gy+h+m), (gx-m, gy+h+m),
                (gx + w*0.5, gy-m), (gx + w*0.5, gy+h+m),
                (gx-m, gy + h*0.5), (gx+w+m, gy + h*0.5)
            ]
            for px, py in pts_to_check:
                if not is_point_in_poly(px, py, outer_pts):
                    return False
            return True

    def get_left_fit_x(gy, h):
        """Find max left and min right envelope over height range [gy-margin, gy+h+margin]"""
        max_left = -float('inf')
        min_right = float('inf')
        steps = 8
        for i in range(steps + 1):
            y = (gy - margin) + (i / float(steps)) * (h + 2 * margin)
            rng = x_range_fn(y)
            if rng is None:
                return None, None
            if rng[0] > max_left:
                max_left = rng[0]
            if rng[1] < min_right:
                min_right = rng[1]
        return max_left + margin, min_right - margin

    # Sorting & Rotation Policy configuration
    parts = list(parts_data)
    rot_policy = 'mixed'
    align_pref = 'left'

    if strategy_id == 0:
        # Area descending, prefer horizontal (90 deg) for long parts
        parts.sort(key=lambda p: p['w'] * p['h'], reverse=True)
        rot_policy = 90
        align_pref = 'left'
    elif strategy_id == 1:
        # Area descending, prefer vertical (0 deg) for long parts
        parts.sort(key=lambda p: p['w'] * p['h'], reverse=True)
        rot_policy = 0
        align_pref = 'left'
    elif strategy_id == 2:
        # Area descending, center align, 90 deg policy
        parts.sort(key=lambda p: p['w'] * p['h'], reverse=True)
        rot_policy = 90
        align_pref = 'center'
    elif strategy_id == 3:
        # Area ascending (small parts first!), 90 deg policy
        parts.sort(key=lambda p: p['w'] * p['h'])
        rot_policy = 90
        align_pref = 'left'
    elif strategy_id == 4:
        # Area ascending (small parts first!), 0 deg policy
        parts.sort(key=lambda p: p['w'] * p['h'])
        rot_policy = 0
        align_pref = 'left'
    elif strategy_id == 5:
        # Max dimension descending, 90 deg policy
        parts.sort(key=lambda p: max(p['w'], p['h']), reverse=True)
        rot_policy = 90
        align_pref = 'left'
    elif strategy_id == 6:
        # Max dimension descending, 0 deg policy
        parts.sort(key=lambda p: max(p['w'], p['h']), reverse=True)
        rot_policy = 0
        align_pref = 'left'
    elif strategy_id == 7:
        # Perimeter descending, mixed policy
        parts.sort(key=lambda p: 2 * (p['w'] + p['h']), reverse=True)
        rot_policy = 'mixed'
        align_pref = 'center'
    elif strategy_id == 8:
        # Height descending, 90 deg policy
        parts.sort(key=lambda p: p['h'], reverse=True)
        rot_policy = 90
        align_pref = 'left'
    elif strategy_id == 9:
        # Width descending, 0 deg policy
        parts.sort(key=lambda p: p['w'], reverse=True)
        rot_policy = 0
        align_pref = 'left'
    else:
        # Randomized permutation multi-start
        import random
        rng_rand = random.Random(strategy_id * 101 + 19)
        rng_rand.shuffle(parts)
        if strategy_id % 3 == 0:
            rot_policy = 90
        elif strategy_id % 3 == 1:
            rot_policy = 0
        else:
            rot_policy = 'mixed'
        align_pref = 'center' if (strategy_id % 2 == 0) else 'left'

    placed = []
    plan = []

    for p in parts:
        pid = p['id']
        pw = float(p['w'])
        ph = float(p['h'])
        aspect = max(pw, ph) / max(1.0, min(pw, ph))

        best_anchor = None
        best_rot = 0
        best_w, best_h = pw, ph
        best_score = float('inf')

        # Determine valid rotations for this part based on rot_policy
        if rot_policy == 90:
            # If part is elongated (aspect > 1.2), test 90 only. If square, test [90, 0]
            valid_rots = [90] if aspect > 1.2 else [90, 0]
        elif rot_policy == 0:
            valid_rots = [0] if aspect > 1.2 else [0, 90]
        else:
            valid_rots = [0, 90] if (strategy_id % 2 == 0) else [90, 0]

        # Filter rotations that exceed container dimensions
        filtered_rots = []
        for rot in valid_rots:
            cur_w, cur_h = (ph, pw) if rot == 90 else (pw, ph)
            if cur_w <= (max_ox - min_ox) and cur_h <= (max_oy - min_oy):
                filtered_rots.append(rot)
        if not filtered_rots:
            # Fallback to any rotation that fits
            for rot in [0, 90]:
                cur_w, cur_h = (ph, pw) if rot == 90 else (pw, ph)
                if cur_w <= (max_ox - min_ox) and cur_h <= (max_oy - min_oy):
                    filtered_rots.append(rot)

        for rot in filtered_rots:
            cur_w, cur_h = (ph, pw) if rot == 90 else (pw, ph)

            # Generate candidate heights gy
            candidate_gys = []
            step_y = max(4.0, (max_oy - min_oy) / 100.0)
            cy_val = min_oy + margin
            while cy_val + cur_h + margin <= max_oy:
                candidate_gys.append(cy_val)
                cy_val += step_y

            for pr in placed:
                top_y = pr[1] + pr[3] + gap
                bot_y = pr[1] - cur_h - gap
                if top_y + cur_h + margin <= max_oy:
                    candidate_gys.append(top_y)
                if bot_y >= min_oy + margin:
                    candidate_gys.append(bot_y)
                candidate_gys.append(pr[1])

            candidate_gys = sorted(list(set(candidate_gys)))

            for gy in candidate_gys:
                xl, xr = get_left_fit_x(gy, cur_h)
                if xl is None or (xl + cur_w > xr):
                    continue

                # Candidate X coordinates:
                cand_xs = []
                if align_pref == 'center':
                    cand_xs.append((xl + xr - cur_w) * 0.5)
                    cand_xs.append(xl)
                    cand_xs.append(xr - cur_w)
                else:
                    cand_xs.append(xl)
                    cand_xs.append((xl + xr - cur_w) * 0.5)
                    cand_xs.append(xr - cur_w)

                # Relative to already placed parts
                for pr in placed:
                    cand_xs.append(pr[0] + pr[2] + gap)   # Step right
                    cand_xs.append(pr[0] - cur_w - gap)   # Step left
                    cand_xs.append(pr[0])                  # Flush left
                    cand_xs.append(pr[0] + pr[2] - cur_w)  # Flush right

                # Filter and deduplicate cand_xs
                cand_xs = [x for x in set(cand_xs) if xl <= x <= xr - cur_w]

                for gx in cand_xs:
                    cur_r = (gx, gy, cur_w, cur_h)
                    collides = False
                    for pr in placed:
                        if rects_overlap(cur_r, pr, gap):
                            collides = True
                            break
                    if collides:
                        continue

                    if not is_inside_fn(gx, gy, cur_w, cur_h):
                        continue

                    # Score calculation
                    if strategy_id % 2 == 0:
                        score = (gy - min_oy) * 10000.0 + (gx - min_ox)
                    else:
                        score = (gy - min_oy) * 10000.0 + abs(gx - (min_ox + max_ox) * 0.5)

                    if score < best_score:
                        best_score = score
                        best_anchor = (gx, gy)
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

        container_type = data.get('container_type', 'polygon')
        gap = float(data.get('gap', 3.0))
        margin = float(data.get('margin', 2.0))
        parts_data = data['parts']
        total_parts = len(parts_data)

        outer_data = {
            'container_type': container_type,
            'outer_bbox': data.get('outer_bbox', [0, 0, 1000, 1000]),
            'outer_polygon': data.get('outer_polygon', []),
            'circle_center': data.get('circle_center', [0, 0]),
            'circle_radius': float(data.get('circle_radius', 0.0))
        }

        num_cpus = cpu_count()
        num_strategies = max(num_cpus, 32)

        tasks = [
            (s_id, parts_data, outer_data, gap, margin)
            for s_id in range(num_strategies)
        ]

        # Multiprocessing across 20 cores (or direct if <= 4 parts)
        if total_parts <= 4:
            results = [solve_nesting_strategy(t) for t in tasks]
            mode_str = f"Direct Fast-Path ({num_strategies} Strategies, {total_parts} parts)"
        else:
            with Pool(processes=min(num_cpus, 20)) as pool:
                results = pool.map(solve_nesting_strategy, tasks)
            mode_str = f"Multi-Core Multiprocessing ({num_cpus} CPUs, {num_strategies} Strategies, {total_parts} parts)"

        best_res = max(results, key=lambda r: (r['placed_count'], r['fitness']))
        elapsed = time.time() - t0

        is_success = (best_res['placed_count'] == total_parts)
        status_str = "SUCCESS" if is_success else "PARTIAL"

        shape_label = f"Circle/Pipe (Dia {outer_data['circle_radius']*2:.1f})" if container_type == 'circle' else "Curved/Polygon"
        msg = (f"MBBN Nesting: {best_res['placed_count']}/{total_parts} parts placed "
               f"in {elapsed*1000.0:.1f}ms [{shape_label} | {mode_str}] (Best Strategy #{best_res['strategy_id']})")

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
                'container_type': container_type,
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
