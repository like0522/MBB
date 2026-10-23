# -*- coding: utf-8 -*-
"""
=============================================================================
[MBBN Ultra-Fast 2D Nesting Multi-Core Engine]
Program: MBB Multi-Container Yield Optimizer (Python 3.14 / Shapely / GEOS)
Version: v3.0 (2026-10-23 08:25:00)
Architecture: Composite Bigness Priority + Multi-Container Backtracking & Trial-and-Error Yield Maximizer
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

def pack_parts_into_single_container(parts, container, gap, margin, rot_policy, align_pref, enable_backtrack=False):
    """
    Packs as many parts as possible into one container (Circle or Polygon).
    Supports wall-hugging anchors, rotation policies, and local backtracking.
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

    def get_fit_bounds(gy, h):
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

    def find_best_anchor_for_part(p, cur_placed):
        pw = float(p['w'])
        ph = float(p['h'])
        aspect = max(pw, ph) / max(1.0, min(pw, ph))

        if rot_policy == 90:
            valid_rots = [90] if aspect > 1.25 else [90, 0]
        elif rot_policy == 0:
            valid_rots = [0] if aspect > 1.25 else [0, 90]
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

            for pr in cur_placed:
                cand_gys.append(pr[1] + pr[3] + gap)
                cand_gys.append(pr[1] - cur_h - gap)
                cand_gys.append(pr[1])

            cand_gys = sorted(list(set([y for y in cand_gys if min_oy + margin <= y <= max_oy - cur_h - margin])))

            for gy in cand_gys:
                xl, xr = get_fit_bounds(gy, cur_h)
                if xl is None or (xl + cur_w > xr):
                    continue

                cand_xs = []
                if align_pref == 'center':
                    cand_xs.append((xl + xr - cur_w) * 0.5)
                    cand_xs.append(xl)
                    cand_xs.append(xr - cur_w)
                elif align_pref == 'right':
                    cand_xs.append(xr - cur_w)
                    cand_xs.append((xl + xr - cur_w) * 0.5)
                    cand_xs.append(xl)
                else: # 'left' or 'wall_hug'
                    cand_xs.append(xl)
                    cand_xs.append((xl + xr - cur_w) * 0.5)
                    cand_xs.append(xr - cur_w)

                for pr in cur_placed:
                    cand_xs.append(pr[0] + pr[2] + gap)
                    cand_xs.append(pr[0] - cur_w - gap)
                    cand_xs.append(pr[0])
                    cand_xs.append(pr[0] + pr[2] - cur_w)

                cand_xs = [x for x in set(cand_xs) if xl <= x <= xr - cur_w]

                for gx in cand_xs:
                    cur_r = (gx, gy, cur_w, cur_h)
                    if any(rects_overlap(cur_r, pr, gap) for pr in cur_placed):
                        continue
                    if not is_inside_fn(gx, gy, cur_w, cur_h):
                        continue

                    # Wall-Hug / Compactness score:
                    # Penalizes height (gy) and x-span, rewards proximity to container boundary
                    score = (gy - min_oy) * 10000.0 + (gx - min_ox)
                    if score < best_score:
                        best_score = score
                        best_anchor = (gx, gy)
                        best_rot = rot
                        best_w, best_h = cur_w, cur_h

        return best_anchor, best_rot, best_w, best_h

    placed = []
    placed_plan = []
    unplaced = []

    for p in parts:
        pid = p['id']
        best_anchor, best_rot, best_w, best_h = find_best_anchor_for_part(p, placed)

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
            # Local Backtracking (넣어보고 지우고 재시도):
            # If a part fails to fit, try unplacing the last placed part, placing current, and re-placing the old one.
            backtrack_success = False
            if enable_backtrack and len(placed) > 0:
                last_rect = placed.pop()
                last_plan = placed_plan.pop()
                last_p = [orig_p for orig_p in parts if orig_p['id'] == last_plan['id']][0]

                # Try placing current part first
                anchor1, rot1, w1, h1 = find_best_anchor_for_part(p, placed)
                if anchor1 is not None:
                    placed.append((anchor1[0], anchor1[1], w1, h1))
                    # Try re-placing last part
                    anchor2, rot2, w2, h2 = find_best_anchor_for_part(last_p, placed)
                    if anchor2 is not None:
                        # Both fit! Commit backtrack!
                        placed_plan.append({
                            'id': pid,
                            'container_id': container['id'],
                            'gx': anchor1[0],
                            'gy': anchor1[1],
                            'w': w1,
                            'h': h1,
                            'rot': rot1,
                            'area': float(p.get('area', w1 * h1))
                        })
                        placed.append((anchor2[0], anchor2[1], w2, h2))
                        placed_plan.append({
                            'id': last_p['id'],
                            'container_id': container['id'],
                            'gx': anchor2[0],
                            'gy': anchor2[1],
                            'w': w2,
                            'h': h2,
                            'rot': rot2,
                            'area': float(last_p.get('area', w2 * h2))
                        })
                        backtrack_success = True
                    else:
                        # Re-placement failed, undo
                        placed.pop()

                if not backtrack_success:
                    # Restore previous state
                    placed.append(last_rect)
                    placed_plan.append(last_plan)

            if not backtrack_success:
                unplaced.append(p)

    return placed_plan, unplaced

def repack_ejection_chain(plan, unplaced, containers_data, gap, margin):
    """
    Trial-and-Error Ejection Chain (넣어보고 지우고 리패킹):
    If large parts remain unplaced, attempts to eject 1~2 small placed parts
    to accommodate the larger unplaced part, then re-packs the ejected parts into remaining cavities.
    """
    if not unplaced:
        return plan, unplaced

    # Sort unplaced by Bigness descending (largest unplaced parts first)
    unplaced_sorted = sorted(unplaced, key=lambda p: compute_bigness(p), reverse=True)
    c_map = {c['id']: c for c in containers_data}

    for target_u in unplaced_sorted:
        target_u_bigness = compute_bigness(target_u)
        # Try each container
        for cid, container in c_map.items():
            c_placed_plans = [p for p in plan if p['container_id'] == cid]
            if not c_placed_plans:
                continue

            # Identify small candidate parts in this container
            small_candidates = [p for p in c_placed_plans if p['area'] < float(target_u.get('area', target_u['w'] * target_u['h']))]
            small_candidates.sort(key=lambda p: p['area'])

            # Try ejecting the smallest 1 or 2 parts
            for k in [1, min(2, len(small_candidates))]:
                if k == 0: continue
                ejected_sub = small_candidates[:k]
                ejected_ids = set(p['id'] for p in ejected_sub)

                # Simulated container state without ejected
                sim_plan = [p for p in plan if p['id'] not in ejected_ids]
                sim_c_plan = [p for p in sim_plan if p['container_id'] == cid]
                sim_placed_rects = [(p['gx'], p['gy'], p['w'], p['h']) for p in sim_c_plan]

                # Test if target_u fits in container with remaining parts
                # Using pack_parts_into_single_container on remaining
                c_other_parts = [orig_p for orig_p in unplaced if orig_p['id'] == target_u['id']]
                test_plan, test_unplaced = pack_parts_into_single_container(
                    c_other_parts, container, gap, margin, rot_policy='mixed', align_pref='left', enable_backtrack=True
                )

                # If target_u could fit together with remaining parts in container:
                # We check full replanning of this container
                c_kept_orig = []
                for p in sim_c_plan:
                    c_kept_orig.append({'id': p['id'], 'w': p['w'], 'h': p['h'], 'area': p['area']})
                test_all_parts = [target_u] + c_kept_orig
                new_c_plan, new_c_unplaced = pack_parts_into_single_container(
                    test_all_parts, container, gap, margin, rot_policy='mixed', align_pref='left', enable_backtrack=True
                )

                if any(p['id'] == target_u['id'] for p in new_c_plan):
                    # target_u successfully placed! Now try re-placing ejected parts into ANY container
                    current_sim_plan = [p for p in sim_plan if p['container_id'] != cid] + new_c_plan
                    re_ejected_candidates = [p for p in unplaced if p['id'] in ejected_ids] + [
                        orig_p for orig_p in c_kept_orig if any(u['id'] == orig_p['id'] for u in new_c_unplaced)
                    ]

                    # Re-pack across all containers
                    all_unplaced_now = [p for p in unplaced if p['id'] != target_u['id']] + re_ejected_candidates
                    # If net placed count improved, accept this swap!
                    if len(current_sim_plan) >= len(plan):
                        new_remaining_unplaced = [p for p in all_unplaced_now if not any(placed_p['id'] == p['id'] for placed_p in current_sim_plan)]
                        return current_sim_plan, new_remaining_unplaced

    return plan, unplaced

def solve_multi_strategy(args):
    """
    Evaluates multi-container packing strategy across 64 combinations.
    Implements Composite Bigness Priority, Container Sorting,
    Wall-Hugging Anchors, and Multi-Iteration Backtracking & Repacking.
    """
    strategy_id, parts_data, containers_data, gap, margin = args

    # Clone containers & parts
    c_list = list(containers_data)
    parts = list(parts_data)

    # 1. Container ordering policy (4 variations)
    c_order_type = strategy_id % 4
    if c_order_type == 0:
        # Largest container area first (대형 컨테이너 우선)
        c_list.sort(key=lambda c: float(c.get('area', 0.0)), reverse=True)
    elif c_order_type == 1:
        # Smallest container area first (잔재 우선 소진 Best-Fit)
        c_list.sort(key=lambda c: float(c.get('area', 0.0)))
    elif c_order_type == 2:
        # Circular containers first
        c_list.sort(key=lambda c: (0 if c.get('type') == 'circle' else 1, -float(c.get('area', 0.0))))
    else:
        # Original CAD selection order
        pass

    # 2. Parts sorting & priority policy (16 distinct archetypes)
    strat_type = (strategy_id // 4) % 16
    enable_backtrack = (strategy_id % 2 == 0)

    if strat_type == 0:
        # [Archetype 1]: Balanced Bigness (User specification: 길이 + 넓이 동시 대형 우선)
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=500.0, weight_aspect=50.0), reverse=True)
        rot_policy = 90
        align_pref = 'left'

    elif strat_type == 1:
        # [Archetype 2]: Heavy Length Bigness (장척물/긴 막대 단품 절대 우선)
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=1000.0, weight_aspect=100.0), reverse=True)
        rot_policy = 90
        align_pref = 'left'

    elif strat_type == 2:
        # [Archetype 3]: Balanced Bigness + Center-Stack Alignment (대칭 밀착)
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=500.0, weight_aspect=50.0), reverse=True)
        rot_policy = 90
        align_pref = 'center'

    elif strat_type == 3:
        # [Archetype 4]: Balanced Bigness + Free Adaptive Rotation (0도 및 90도 자유 선택)
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=500.0, weight_aspect=50.0), reverse=True)
        rot_policy = 'mixed'
        align_pref = 'left'

    elif strat_type == 4:
        # [Archetype 5]: Area Primary, Max-Dimension Secondary
        parts.sort(key=lambda p: (float(p.get('area', p['w'] * p['h'])), max(p['w'], p['h'])), reverse=True)
        rot_policy = 90
        align_pref = 'left'

    elif strat_type == 5:
        # [Archetype 6]: Max-Dimension Primary, Area Secondary
        parts.sort(key=lambda p: (max(p['w'], p['h']), float(p.get('area', p['w'] * p['h']))), reverse=True)
        rot_policy = 90
        align_pref = 'center'

    elif strat_type == 6:
        # [Archetype 7]: 2-Tier Hierarchical (대형 그룹 먼저 배치 -> 소형 그룹 캐비티 채우기)
        bigness_vals = [compute_bigness(p, weight_dim=500.0) for p in parts]
        median_b = sorted(bigness_vals)[len(bigness_vals) // 2] if bigness_vals else 0.0
        tier1 = [p for p in parts if compute_bigness(p, weight_dim=500.0) >= median_b]
        tier2 = [p for p in parts if compute_bigness(p, weight_dim=500.0) < median_b]
        tier1.sort(key=lambda p: compute_bigness(p, weight_dim=500.0), reverse=True)
        tier2.sort(key=lambda p: compute_bigness(p, weight_dim=500.0), reverse=True)
        parts = tier1 + tier2
        rot_policy = 'mixed'
        align_pref = 'left'

    elif strat_type == 7:
        # [Archetype 8]: Diagonal / Perimeter Descending
        parts.sort(key=lambda p: math.hypot(p['w'], p['h']), reverse=True)
        rot_policy = 90
        align_pref = 'left'

    elif strat_type == 8:
        # [Archetype 9]: Aspect-Ratio & Long Bars First
        parts.sort(key=lambda p: (max(p['w'], p['h']) / max(1.0, min(p['w'], p['h'])), float(p.get('area', 0.0))), reverse=True)
        rot_policy = 90
        align_pref = 'left'

    elif strat_type == 9:
        # [Archetype 10]: Heavy Length Bigness + Vertical Bias (rot=0)
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=800.0), reverse=True)
        rot_policy = 0
        align_pref = 'left'

    elif strat_type == 10:
        # [Archetype 11]: Balanced Bigness + Right-Wall Hugging
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=500.0), reverse=True)
        rot_policy = 90
        align_pref = 'right'

    elif strat_type == 11:
        # [Archetype 12]: Bigness with moderate dimension weight
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=250.0), reverse=True)
        rot_policy = 'mixed'
        align_pref = 'center'

    else:
        # [Archetypes 13..16]: Stochastic Local Search (Bigness 기반 국소 셔플 및 시행착오)
        rng = random.Random(strategy_id * 1013 + 57)
        # Base sort by Bigness
        parts.sort(key=lambda p: compute_bigness(p, weight_dim=500.0), reverse=True)
        # Permute within small sliding windows (넣어보고 지우고 탐색)
        for i in range(len(parts) - 1):
            if rng.random() < 0.35:
                parts[i], parts[i + 1] = parts[i + 1], parts[i]
        rot_policy = 90 if (strategy_id % 2 == 0) else 'mixed'
        align_pref = 'center' if (strategy_id % 3 == 0) else 'left'

    # Multi-container sequential bin packing
    total_plan = []
    remaining_parts = list(parts)
    used_containers = set()

    for container in c_list:
        if not remaining_parts:
            break
        c_plan, unplaced = pack_parts_into_single_container(
            remaining_parts, container, gap, margin, rot_policy, align_pref, enable_backtrack=enable_backtrack
        )
        if c_plan:
            total_plan.extend(c_plan)
            used_containers.add(container['id'])
            remaining_parts = unplaced

    # 3. Post-Packing Ejection Chain & Repacking ("넣어보고 지우고" 전역 리패킹)
    if remaining_parts and len(total_plan) > 0 and (strategy_id % 3 == 0):
        total_plan, remaining_parts = repack_ejection_chain(
            total_plan, remaining_parts, containers_data, gap, margin
        )

    # 4. Metrics & Fitness Evaluation
    placed_count = len(total_plan)
    total_parts = len(parts_data)
    placed_area = sum(p['area'] for p in total_plan)
    used_c_area = sum(float(c.get('area', 1.0)) for c in containers_data if c['id'] in used_containers)
    if used_c_area <= 0.0:
        used_c_area = 1.0
    yield_pct = (placed_area / used_c_area) * 100.0

    # Fitness Formula: Placed Count (10M) > Placed Area (100) > Yield % (1000) - Containers Used (10K)
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
        num_strategies = max(num_cpus * 3, 64)

        tasks = [
            (s_id, parts_data, containers_data, gap, margin)
            for s_id in range(num_strategies)
        ]

        # Multi-Core Multiprocessing Execution
        with Pool(processes=min(num_cpus, 20)) as pool:
            results = pool.map(solve_multi_strategy, tasks)
        mode_str = f"Multi-Core Multiprocessing ({min(num_cpus, 20)} CPUs, {num_strategies} Strategies, {num_containers} Containers)"

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
;;; Version: v3.0 (2026-10-23 08:25:00)
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
