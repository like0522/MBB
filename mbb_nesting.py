# -*- coding: utf-8 -*-
"""
=============================================================================
[MBBN Ultra-Fast 2D Nesting Multi-Core Engine]
Program: MBB Multi-Container Yield Optimizer (Python 3.14 / Shapely / GEOS)
Version: v2.8 (2026-10-23 07:03:00)
Architecture: Multi-Container Multi-Bin Yield Maximization + Wall-Adaptive
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

def pack_parts_into_single_container(parts, container, gap, margin, rot_policy, align_pref):
    """
    Packs as many parts as possible into one container (Circle or Polygon).
    Returns (placed_plan, unplaced_parts).
    """
    is_circle = (container.get('type') == 'circle')
    if is_circle:
        cx, cy = container['circle_center']
        R = float(container['circle_radius'])
        min_ox, min_oy = cx - R, cy - R
        max_ox, max_oy = cx + R, cy + R
        prep_poly = None

        def x_range_fn(y):
            dy = abs(y - cy)
            if dy > R: return None
            dx = math.sqrt(max(0.0, R * R - dy * dy))
            return (cx - dx, cx + dx)

        def is_inside_fn(gx, gy, w, h):
            m = margin
            for px, py in [(gx-m, gy-m), (gx+w+m, gy-m), (gx+w+m, gy+h+m), (gx-m, gy+h+m)]:
                if (px - cx) ** 2 + (py - cy) ** 2 > R * R + 1e-4:
                    return False
            return True
    else:
        outer_pts = container['outer_polygon']
        min_ox, min_oy, max_ox, max_oy = container['outer_bbox']
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
        max_left = -float('inf')
        min_right = float('inf')
        steps = 8
        for i in range(steps + 1):
            y = (gy - margin) + (i / float(steps)) * (h + 2 * margin)
            rng = x_range_fn(y)
            if rng is None: return None, None
            if rng[0] > max_left: max_left = rng[0]
            if rng[1] < min_right: min_right = rng[1]
        return max_left + margin, min_right - margin

    placed = []
    placed_plan = []
    unplaced = []

    for p in parts:
        pid = p['id']
        pw = float(p['w'])
        ph = float(p['h'])
        aspect = max(pw, ph) / max(1.0, min(pw, ph))

        if rot_policy == 90:
            valid_rots = [90] if aspect > 1.2 else [90, 0]
        elif rot_policy == 0:
            valid_rots = [0] if aspect > 1.2 else [0, 90]
        else:
            valid_rots = [0, 90]

        filtered_rots = []
        for rot in valid_rots:
            cur_w, cur_h = (ph, pw) if rot == 90 else (pw, ph)
            if cur_w <= (max_ox - min_ox) and cur_h <= (max_oy - min_oy):
                filtered_rots.append(rot)

        if not filtered_rots:
            for rot in [0, 90]:
                cur_w, cur_h = (ph, pw) if rot == 90 else (pw, ph)
                if cur_w <= (max_ox - min_ox) and cur_h <= (max_oy - min_oy):
                    filtered_rots.append(rot)

        best_anchor = None
        best_rot = 0
        best_w, best_h = pw, ph
        best_score = float('inf')

        for rot in filtered_rots:
            cur_w, cur_h = (ph, pw) if rot == 90 else (pw, ph)
            cand_gys = []
            step_y = max(4.0, (max_oy - min_oy) / 80.0)
            cy_val = min_oy + margin
            while cy_val + cur_h + margin <= max_oy:
                cand_gys.append(cy_val)
                cy_val += step_y

            for pr in placed:
                cand_gys.append(pr[1] + pr[3] + gap)
                cand_gys.append(pr[1] - cur_h - gap)
                cand_gys.append(pr[1])

            cand_gys = sorted(list(set([y for y in cand_gys if min_oy + margin <= y <= max_oy - cur_h - margin])))

            for gy in cand_gys:
                xl, xr = get_left_fit_x(gy, cur_h)
                if xl is None or (xl + cur_w > xr):
                    continue

                cand_xs = []
                if align_pref == 'center':
                    cand_xs.append((xl + xr - cur_w) * 0.5)
                    cand_xs.append(xl)
                    cand_xs.append(xr - cur_w)
                else:
                    cand_xs.append(xl)
                    cand_xs.append((xl + xr - cur_w) * 0.5)
                    cand_xs.append(xr - cur_w)

                for pr in placed:
                    cand_xs.append(pr[0] + pr[2] + gap)
                    cand_xs.append(pr[0] - cur_w - gap)
                    cand_xs.append(pr[0])
                    cand_xs.append(pr[0] + pr[2] - cur_w)

                cand_xs = [x for x in set(cand_xs) if xl <= x <= xr - cur_w]

                for gx in cand_xs:
                    cur_r = (gx, gy, cur_w, cur_h)
                    if any(rects_overlap(cur_r, pr, gap) for pr in placed):
                        continue
                    if not is_inside_fn(gx, gy, cur_w, cur_h):
                        continue

                    score = (gy - min_oy) * 10000.0 + (gx - min_ox)
                    if score < best_score:
                        best_score = score
                        best_anchor = (gx, gy)
                        best_rot = rot
                        best_w, best_h = cur_w, cur_h

        if best_anchor is not None:
            gx, gy = best_anchor
            placed.append((gx, gy, best_w, best_h))
            placed_plan.append({
                'id': pid,
                'container_id': container['id'],
                'gx': gx,
                'gy': gy,
                'w': best_w,
                'h': best_h,
                'rot': best_rot,
                'area': float(p.get('area', best_w * best_h))
            })
        else:
            unplaced.append(p)

    return placed_plan, unplaced

def solve_multi_strategy(args):
    """
    Evaluates multi-container packing strategy.
    Tries different parts orders, container orders, and rotation policies.
    """
    strategy_id, parts_data, containers_data, gap, margin = args

    # Clone containers
    c_list = list(containers_data)
    parts = list(parts_data)

    # 1. Container ordering policy
    if strategy_id % 4 == 0:
        # Original order
        pass
    elif strategy_id % 4 == 1:
        # Largest container area first
        c_list.sort(key=lambda c: float(c.get('area', 0.0)), reverse=True)
    elif strategy_id % 4 == 2:
        # Smallest container area first
        c_list.sort(key=lambda c: float(c.get('area', 0.0)))
    else:
        # Circular containers first
        c_list.sort(key=lambda c: (0 if c.get('type') == 'circle' else 1))

    # 2. Parts sorting policy
    strat_type = strategy_id // 4
    if strat_type == 0:
        parts.sort(key=lambda p: float(p.get('area', p['w'] * p['h'])), reverse=True)
        rot_policy = 90
        align_pref = 'left'
    elif strat_type == 1:
        parts.sort(key=lambda p: float(p.get('area', p['w'] * p['h'])), reverse=True)
        rot_policy = 0
        align_pref = 'left'
    elif strat_type == 2:
        parts.sort(key=lambda p: float(p.get('area', p['w'] * p['h'])), reverse=True)
        rot_policy = 'mixed'
        align_pref = 'center'
    elif strat_type == 3:
        # Long parts first (aspect ratio)
        parts.sort(key=lambda p: max(p['w'], p['h']) / max(1.0, min(p['w'], p['h'])), reverse=True)
        rot_policy = 90
        align_pref = 'left'
    elif strat_type == 4:
        # Max dimension descending
        parts.sort(key=lambda p: max(p['w'], p['h']), reverse=True)
        rot_policy = 90
        align_pref = 'left'
    elif strat_type == 5:
        # Area ascending (small parts first)
        parts.sort(key=lambda p: float(p.get('area', p['w'] * p['h'])))
        rot_policy = 90
        align_pref = 'left'
    elif strat_type == 6:
        # Perimeter descending
        parts.sort(key=lambda p: 2 * (p['w'] + p['h']), reverse=True)
        rot_policy = 'mixed'
        align_pref = 'center'
    else:
        # Randomized shuffle
        import random
        rng = random.Random(strategy_id * 107 + 23)
        rng.shuffle(parts)
        rng.shuffle(c_list)
        rot_policy = 90 if (strategy_id % 2 == 0) else 0
        align_pref = 'left' if (strategy_id % 3 == 0) else 'center'

    # Multi-container sequential bin packing
    total_plan = []
    remaining_parts = list(parts)
    used_containers = set()

    for container in c_list:
        if not remaining_parts:
            break
        c_plan, unplaced = pack_parts_into_single_container(remaining_parts, container, gap, margin, rot_policy, align_pref)
        if c_plan:
            total_plan.extend(c_plan)
            used_containers.add(container['id'])
            remaining_parts = unplaced

    # Metrics calculation
    placed_count = len(total_plan)
    total_parts = len(parts_data)
    placed_area = sum(p['area'] for p in total_plan)
    used_c_area = sum(float(c.get('area', 1.0)) for c in containers_data if c['id'] in used_containers)
    if used_c_area <= 0.0:
        used_c_area = 1.0
    yield_pct = (placed_area / used_c_area) * 100.0

    # Fitness: High placed count > High placed area > High yield %
    fitness = (placed_count * 10000000.0) + (placed_area * 10.0) + (yield_pct * 100.0) - (used_c_area * 0.1)

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

        # Standardize containers list (support single or multi container)
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
        num_strategies = max(num_cpus, 36)

        tasks = [
            (s_id, parts_data, containers_data, gap, margin)
            for s_id in range(num_strategies)
        ]

        if total_parts <= 4 and num_containers == 1:
            results = [solve_multi_strategy(t) for t in tasks]
            mode_str = f"Direct Fast-Path ({num_strategies} Strategies, {total_parts} parts)"
        else:
            with Pool(processes=min(num_cpus, 20)) as pool:
                results = pool.map(solve_multi_strategy, tasks)
            mode_str = f"Multi-Core Multiprocessing ({num_cpus} CPUs, {num_strategies} Strategies, {num_containers} Containers)"

        best_res = max(results, key=lambda r: (r['placed_count'], r['fitness']))
        elapsed = time.time() - t0

        is_success = (best_res['placed_count'] == total_parts)
        status_str = "SUCCESS" if is_success else "PARTIAL"

        msg = (f"MBBN Multi-Container Nesting: {best_res['placed_count']}/{total_parts} parts placed "
               f"in {elapsed*1000.0:.1f}ms [Yield: {best_res['yield_pct']:.1f}% | Containers Used: {len(best_res['used_containers'])}/{num_containers} | {mode_str}] "
               f"(Best Strategy #{best_res['strategy_id']})")

        # Generate AutoLISP output file
        # Format of MBB_NESTING_PLAN: ( (part_id container_id gx gy w h rot) ... )
        plan_lines = []
        for item in best_res['plan']:
            plan_lines.append(f"  ({item['id']} {item['container_id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f} {item['rot']})")

        lsp_content = f""";;; MBBN MULTI-CONTAINER NESTING RESULT GENERATED BY PYTHON ENGINE
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
