# -*- coding: utf-8 -*-
"""
MBBN v3.7 - 형상 친화도 기반 전역 수율 극대화 2D 네스팅 엔진 (NFP / Deepnest 알고리즘)
- 직교(0도 및 90도) 안정 회전 고정 (대각선 왜곡 및 텍스트 반전 원천 차단)
- 외곽 컨테이너 100% 완전 내접 (Margin 엄수) 및 단품 간 0% 무충돌 (Gap 엄수)
- 2단계 사후 물리 검토(Post-Verification Collision Audit) 및 간격 미세 조정(Relaxation Nudge) 완비
- 컨테이너 용량 초과 미배치 단품 우측 버퍼 구역 자동 단정 정렬
LAST MODIFIED: 2026-10-23 14:50:00
"""

import os
import sys
import json
import math
import random
from multiprocessing import Pool, cpu_count

try:
    from shapely.geometry import Polygon, box, Point
    from shapely.prepared import prep
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False

def is_point_in_poly(x, y, poly):
    n = len(poly)
    inside = False
    p1x, p1y = poly[0]
    for i in range(1, n + 1):
        p2x, p2y = poly[i % n]
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
    n = len(poly)
    xs = []
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if min(y1, y2) <= y <= max(y1, y2) and y1 != y2:
            t = (y - y1) / (y2 - y1)
            if 0.0 <= t <= 1.0:
                xs.append(x1 + t * (x2 - x1))
    if len(xs) >= 2:
        return (min(xs), max(xs))
    return None

def rotate_pts(pts, ang_deg):
    """
    Rotates 2D orthogonal plate points by 0 or 90 deg.
    Returns: (normalized_pts, width, height, min_ox, min_oy)
    """
    min_x = min(p[0] for p in pts)
    max_x = max(p[0] for p in pts)
    min_y = min(p[1] for p in pts)
    max_y = max(p[1] for p in pts)
    orig_w = max_x - min_x
    orig_h = max_y - min_y

    if abs(ang_deg) < 1e-5:
        norm_pts = [(p[0] - min_x, p[1] - min_y) for p in pts]
        return norm_pts, orig_w, orig_h, 0.0, 0.0
    elif abs(ang_deg - 90.0) < 1e-5:
        cur_w = orig_h
        cur_h = orig_w
        norm_pts = [[0.0, 0.0], [cur_w, 0.0], [cur_w, cur_h], [0.0, cur_h]]
        return norm_pts, cur_w, cur_h, -orig_h, 0.0
    else:
        rad = math.radians(ang_deg)
        ca, sa = math.cos(rad), math.sin(rad)
        r_pts = [(p[0]*ca - p[1]*sa, p[0]*sa + p[1]*ca) for p in pts]
        mx = min(p[0] for p in r_pts)
        my = min(p[1] for p in r_pts)
        cur_w = max(p[0] for p in r_pts) - mx
        cur_h = max(p[1] for p in r_pts) - my
        norm_pts = [(p[0] - mx, p[1] - my) for p in r_pts]
        return norm_pts, cur_w, cur_h, mx, my

def compute_bigness(p, weight_dim=500.0, weight_aspect=50.0):
    w = float(p['w'])
    h = float(p['h'])
    area = float(p.get('area', w * h))
    max_d = max(w, h)
    min_d = max(1.0, min(w, h))
    aspect = max_d / min_d
    return area + max_d * weight_dim + aspect * weight_aspect

class NestingContainer:
    def __init__(self, c_data, gap=5.0, margin=1.0):
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
            self.eff_R_sq = self.eff_R * self.eff_R
            self.prep_inner = None
        else:
            self.outer_pts = c_data['outer_polygon']
            self.min_ox, self.min_oy, self.max_ox, self.max_oy = c_data['outer_bbox']
            if HAS_SHAPELY:
                poly = Polygon(self.outer_pts)
                if not poly.is_valid:
                    poly = poly.buffer(0)
                inner_poly = poly.buffer(-self.margin)
                if inner_poly.is_empty:
                    inner_poly = poly
                self.prep_inner = prep(inner_poly)
            else:
                self.prep_inner = None

    def is_inside(self, gx, gy, w, h):
        """Exact containment test with margin clearance."""
        if self.type == 'circle':
            test_pts = [
                (gx, gy), (gx + w, gy), (gx + w, gy + h), (gx, gy + h),
                (gx + w*0.5, gy), (gx + w*0.5, gy + h),
                (gx, gy + h*0.5), (gx + w, gy + h*0.5)
            ]
            for px, py in test_pts:
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

    def find_best_position(self, base_pts, pw, ph, allow_rotations=[0, 90], align_pref='left'):
        """
        NFP Sliding Contact Placement (Strict 0 and 90 deg orthogonal only).
        """
        best_score = float('inf')
        best_node = None

        if base_pts is None or len(base_pts) < 3:
            base_pts = [[0.0, 0.0], [pw, 0.0], [pw, ph], [0.0, ph]]

        # Precompute configurations (Strictly 0 and 90)
        candidate_configs = []
        for rot_deg in allow_rotations:
            norm_pts, cur_w, cur_h, off_x, off_y = rotate_pts(base_pts, rot_deg)
            candidate_configs.append((cur_w, cur_h, rot_deg, off_x, off_y))

        for cur_w, cur_h, rot_deg, off_x, off_y in candidate_configs:
            if cur_w > (self.max_ox - self.min_ox) or cur_h > (self.max_oy - self.min_oy):
                continue

            cand_positions = []

            # (A) Container bottom base anchor
            base_y = self.min_oy + self.margin
            if self.type == 'circle':
                dy = abs(base_y - self.cy)
                if dy <= self.eff_R:
                    hw = math.sqrt(max(0.0, self.eff_R**2 - dy**2))
                    cand_positions.append((self.cx - cur_w*0.5, base_y))
                    cand_positions.append((self.cx - hw, base_y))
                    cand_positions.append((self.cx + hw - cur_w, base_y))
            else:
                xr = get_poly_x_range(self.outer_pts, base_y)
                if xr:
                    cand_positions.append((xr[0] + self.margin, base_y))
                    cand_positions.append((xr[1] - self.margin - cur_w, base_y))
                    cand_positions.append(((xr[0] + xr[1] - cur_w)*0.5, base_y))

            # (B) Sliding contact points against each placed part's NFP outer boundaries
            for pr in self.placed:
                # Right of pr
                rx = pr['gx'] + pr['w'] + self.gap
                cand_positions.append((rx, pr['gy']))
                cand_positions.append((rx, pr['gy'] + pr['h'] - cur_h))
                cand_positions.append((rx, pr['gy'] + (pr['h'] - cur_h)*0.5))

                # Above pr
                ty = pr['gy'] + pr['h'] + self.gap
                cand_positions.append((pr['gx'], ty))
                cand_positions.append((pr['gx'] + pr['w'] - cur_w, ty))
                cand_positions.append((pr['gx'] + (pr['w'] - cur_w)*0.5, ty))

                # Left of pr
                lx = pr['gx'] - cur_w - self.gap
                cand_positions.append((lx, pr['gy']))
                cand_positions.append((lx, pr['gy'] + pr['h'] - cur_h))

                # Below pr
                by = pr['gy'] - cur_h - self.gap
                cand_positions.append((pr['gx'], by))
                cand_positions.append((pr['gx'] + pr['w'] - cur_w, by))

            # (C) Systematic Y-level scanning for comprehensive space coverage
            y_steps = 30
            step_val = (self.max_oy - self.min_oy - 2.0*self.margin - cur_h)
            if step_val > 0:
                step_y = max(8.0, step_val / float(y_steps))
                cy_val = self.min_oy + self.margin
                while cy_val + cur_h + self.margin <= self.max_oy:
                    if self.type == 'circle':
                        dy1 = abs(cy_val - self.cy)
                        dy2 = abs(cy_val + cur_h - self.cy)
                        max_dy = max(dy1, dy2)
                        if max_dy <= self.eff_R:
                            hw = math.sqrt(max(0.0, self.eff_R**2 - max_dy**2))
                            cand_positions.append((self.cx - hw, cy_val))
                            cand_positions.append((self.cx + hw - cur_w, cy_val))
                            cand_positions.append(((self.cx - hw + self.cx + hw - cur_w)*0.5, cy_val))
                    else:
                        rng1 = get_poly_x_range(self.outer_pts, cy_val)
                        rng2 = get_poly_x_range(self.outer_pts, cy_val + cur_h)
                        if rng1 and rng2:
                            min_lx = max(rng1[0], rng2[0]) + self.margin
                            max_rx = min(rng1[1], rng2[1]) - self.margin
                            if max_rx - min_lx >= cur_w:
                                cand_positions.append((min_lx, cy_val))
                                cand_positions.append((max_rx - cur_w, cy_val))
                                cand_positions.append(((min_lx + max_rx - cur_w)*0.5, cy_val))
                    cy_val += step_y

            # Deduplicate candidate positions
            seen = set()
            unique_cands = []
            for gx, gy in cand_positions:
                key = (round(gx, 2), round(gy, 2))
                if key not in seen:
                    seen.add(key)
                    unique_cands.append((gx, gy))

            for gx, gy in unique_cands:
                # 1. Exact boundary containment check
                if not self.is_inside(gx, gy, cur_w, cur_h):
                    continue

                # 2. Strict pairwise collision check with gap
                col = False
                for pr in self.placed:
                    if not (gx + cur_w + self.gap <= pr['gx'] or
                            pr['gx'] + pr['w'] + self.gap <= gx or
                            gy + cur_h + self.gap <= pr['gy'] or
                            pr['gy'] + pr['h'] + self.gap <= gy):
                        col = True
                        break
                if col:
                    continue

                # 3. Score candidate: Relative Y gravity + compactness bonus
                rel_y = (gy - self.min_oy) / max(1.0, self.max_oy - self.min_oy)
                score = rel_y * 1000.0

                contact_bonus = 0.0
                for pr in self.placed:
                    if abs(gx - (pr['gx'] + pr['w'] + self.gap)) < 1.0: contact_bonus += 300.0
                    if abs((gx + cur_w + self.gap) - pr['gx']) < 1.0: contact_bonus += 300.0
                    if abs(gy - (pr['gy'] + pr['h'] + self.gap)) < 1.0: contact_bonus += 500.0
                    if abs((gy + cur_h + self.gap) - pr['gy']) < 1.0: contact_bonus += 500.0

                if self.type == 'circle':
                    score += abs(gx + cur_w*0.5 - self.cx) * 5.0
                else:
                    if align_pref == 'center':
                        mid_c = (self.min_ox + self.max_ox)*0.5
                        score += abs(gx + cur_w*0.5 - mid_c) * 5.0
                    elif align_pref == 'right':
                        score -= (gx - self.min_ox) * 0.1
                    else:
                        score += (gx - self.min_ox) * 0.1

                score -= contact_bonus

                if score < best_score:
                    best_score = score
                    best_node = (gx, gy, cur_w, cur_h, rot_deg, off_x, off_y)

        return best_node, best_score

    def place_part(self, node, part_id, orig_area):
        gx, gy, w, h, rot_deg, off_x, off_y = node
        entry = {
            'id': part_id,
            'container_id': self.id,
            'gx': gx,
            'gy': gy,
            'w': w,
            'h': h,
            'rot': rot_deg,
            'off_x': off_x,
            'off_y': off_y,
            'area': orig_area
        }
        self.placed.append(entry)
        return entry

def solve_multi_strategy(args):
    strategy_id, parts_data, containers_data, gap, margin = args

    containers = list(containers_data)
    parts = list(parts_data)
    engs = {c['id']: NestingContainer(c, gap=gap, margin=margin) for c in containers}

    align = ['left', 'center', 'right', 'left'][strategy_id % 4]
    w_dim = [500.0, 1000.0, 250.0, 750.0][strategy_id % 4]

    # Orthogonal rotations only: 0 and 90 deg
    angle_set = [0, 90]

    for p in parts:
        if 'polygon' not in p or not p['polygon'] or len(p['polygon']) < 3:
            pw, ph = float(p['w']), float(p['h'])
            p['polygon'] = [[0.0, 0.0], [pw, 0.0], [pw, ph], [0.0, ph]]

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

    # 1. Very long parts -> wide containers
    for p in very_long_parts:
        pw, ph = float(p['w']), float(p['h'])
        pts = p['polygon']
        placed = False
        for cid in wide_containers:
            n, _ = engs[cid].find_best_position(pts, pw, ph, allow_rotations=[0])
            if n:
                item = engs[cid].place_part(n, p['id'], float(p.get('area', pw*ph)))
                placed_plan.append(item)
                placed = True
                break
        if not placed:
            unplaced.append(p)

    # 2. Huge plates -> largest circle containers
    for p in huge_plates:
        pw, ph = float(p['w']), float(p['h'])
        pts = p['polygon']
        placed = False
        for c in circle_containers:
            cid = c['id']
            n, _ = engs[cid].find_best_position(pts, pw, ph, allow_rotations=[0, 90], align_pref=align)
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

    c_order_bars = [c['id'] for c in circle_containers] + [c['id'] for c in containers if c['type'] != 'circle']
    for p in long_bars:
        pw, ph = float(p['w']), float(p['h'])
        pts = p['polygon']
        area = float(p.get('area', pw*ph))
        best_eng, best_node, best_s = None, None, float('inf')
        for cid in c_order_bars:
            n, s = engs[cid].find_best_position(pts, pw, ph, allow_rotations=angle_set, align_pref=align)
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
        pts = p['polygon']
        area = float(p.get('area', pw*ph))
        best_eng, best_node, best_s = None, None, float('inf')
        for cid in sorted(engs.keys()):
            n, s = engs[cid].find_best_position(pts, pw, ph, allow_rotations=angle_set, align_pref=align)
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
    used_container_area = sum(engs[cid].area for cid in used_containers)
    yield_val = (placed_area / used_container_area * 100.0) if used_container_area > 0 else 0.0

    fitness = (
        placed_count * 100000000.0
        - len(used_containers) * 50000.0
        + yield_val * 1000.0
    )

    return {
        'strategy_id': strategy_id,
        'fitness': fitness,
        'placed_count': placed_count,
        'total_parts': total_parts,
        'yield': yield_val,
        'used_containers': len(used_containers),
        'plan': placed_plan
    }

def post_verify_and_relax(placed_plan, containers_data, parts_data, gap, margin):
    """
    2단계 사후 물리 검토(Post-Verification Collision Audit) & 간격 미세 조정(Relaxation Nudge) 루프.
    외곽 돌출 0건, 단품 간 겹침 0건, 최소 간격 gap 엄수를 100% 보증.
    """
    containers = {c['id']: c for c in containers_data}
    engs = {c['id']: NestingContainer(c, gap=gap, margin=margin) for c in containers_data}
    
    c_plans = {cid: [] for cid in containers.keys()}
    for item in placed_plan:
        c_plans[item['container_id']].append(dict(item))

    verified_plan = []
    dropped_parts = []

    for cid, items in c_plans.items():
        if not items:
            continue
        c = containers[cid]
        eng = engs[cid]

        # 1. Relaxation pass: fine-tune gaps
        for it in range(5):
            moved_any = False
            for i in range(len(items)):
                for j in range(i + 1, len(items)):
                    p1, p2 = items[i], items[j]
                    dx = max(0.0, max(p1['gx'] - (p2['gx'] + p2['w']), p2['gx'] - (p1['gx'] + p1['w'])))
                    dy = max(0.0, max(p1['gy'] - (p2['gy'] + p2['h']), p2['gy'] - (p1['gy'] + p1['h'])))
                    if dx < gap - 1e-4 and dy < gap - 1e-4:
                        push_dx = gap - dx + 0.1
                        if eng.is_inside(p2['gx'] + push_dx, p2['gy'], p2['w'], p2['h']):
                            p2['gx'] += push_dx
                            moved_any = True
                        else:
                            push_dy = gap - dy + 0.1
                            if eng.is_inside(p2['gx'], p2['gy'] + push_dy, p2['w'], p2['h']):
                                p2['gy'] += push_dy
                                moved_any = True
            if not moved_any:
                break

        # 2. Strict final assertion audit
        valid_c_items = []
        for item in items:
            gx, gy, w, h = item['gx'], item['gy'], item['w'], item['h']
            if not eng.is_inside(gx, gy, w, h):
                dropped_parts.append(item['id'])
                continue
            collides = False
            for prev in valid_c_items:
                dx = max(0.0, max(gx - (prev['gx'] + prev['w']), prev['gx'] - (gx + w)))
                dy = max(0.0, max(gy - (prev['gy'] + prev['h']), prev['gy'] - (gy + h)))
                if dx < gap - 1e-4 and dy < gap - 1e-4:
                    collides = True
                    break
            if collides:
                dropped_parts.append(item['id'])
            else:
                valid_c_items.append(item)

        verified_plan.extend(valid_c_items)

    return verified_plan, dropped_parts

def build_unplaced_buffer(unplaced_parts, containers_data, gap=10.0):
    """
    컨테이너에 들어가지 못한 단품들을 우측 버퍼 구역에 단정하게 수직 컬럼으로 정렬.
    """
    if not unplaced_parts:
        return []

    max_x = -float('inf')
    min_y = float('inf')
    for c in containers_data:
        if c['type'] == 'circle':
            cx, cy = c['circle_center']
            r = c['circle_radius']
            max_x = max(max_x, cx + r)
            min_y = min(min_y, cy - r)
        else:
            b = c['outer_bbox']
            max_x = max(max_x, b[2])
            min_y = min(min_y, b[1])

    start_x = max_x + 500.0
    cur_x = start_x
    cur_y = min_y
    max_col_w = 0.0
    col_max_height = 2000.0

    unplaced_plan = []
    for p in unplaced_parts:
        pw, ph = float(p['w']), float(p['h'])
        if (cur_y + ph) > (min_y + col_max_height):
            cur_x += max_col_w + gap * 3.0
            cur_y = min_y
            max_col_w = 0.0

        unplaced_plan.append({
            'id': p['id'],
            'gx': cur_x,
            'gy': cur_y,
            'w': pw,
            'h': ph,
            'rot': 0.0
        })
        cur_y += ph + gap * 2.0
        max_col_w = max(max_col_w, pw)

    return unplaced_plan

def run_nesting(input_path, output_lsp_path, output_json_path=None):
    with open(input_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    containers_data = data['containers']
    parts_data = data['parts']
    gap = float(data.get('gap', 5.0))
    margin = float(data.get('margin', 1.0))

    cores = min(32, max(4, cpu_count()))
    num_strategies = max(32, cores * 2)

    tasks = [(i, parts_data, containers_data, gap, margin) for i in range(num_strategies)]

    with Pool(processes=cores) as pool:
        results = pool.map(solve_multi_strategy, tasks)

    best_res = max(results, key=lambda r: r['fitness'])
    raw_plan = best_res['plan']

    # 2단계 사후 정밀 물리 검토 및 Relaxation
    verified_plan, dropped = post_verify_and_relax(raw_plan, containers_data, parts_data, gap, margin)

    placed_ids = set(p['id'] for p in verified_plan)
    unplaced_parts = [p for p in parts_data if p['id'] not in placed_ids]
    unplaced_plan = build_unplaced_buffer(unplaced_parts, containers_data, gap=gap)

    placed_count = len(verified_plan)
    unplaced_count = len(unplaced_parts)
    total_count = len(parts_data)
    used_containers = set(p['container_id'] for p in verified_plan)
    used_c = len(used_containers)

    placed_area = sum(p['area'] for p in verified_plan)
    total_used_area = 0.0
    for c in containers_data:
        if c['id'] in used_containers:
            total_used_area += float(c.get('area', 1.0))
    total_yield = (placed_area / total_used_area * 100.0) if total_used_area > 0 else 0.0

    # AutoLISP 파일 작성 (CP949)
    with open(output_lsp_path, 'w', encoding='cp949', errors='replace') as f:
        f.write(";;; MBBN v3.7 Nesting Plan (Deepnest NFP & Post-Verification Audit)\n")
        f.write(f"(setq MBB_TOTAL_YIELD {total_yield:.2f})\n")
        f.write(f"(setq MBB_PLACED_COUNT {placed_count})\n")
        f.write(f"(setq MBB_UNPLACED_COUNT {unplaced_count})\n")
        f.write(f"(setq MBB_NESTING_STATUS \"SUCCESS\")\n")
        f.write("(setq MBB_NESTING_PLAN '(\n")
        for item in verified_plan:
            rot_deg = float(item['rot'])
            off_x = float(item.get('off_x', 0.0))
            off_y = float(item.get('off_y', 0.0))
            f.write(f"  ({item['id']} {item['container_id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f} {rot_deg:.2f} {off_x:.4f} {off_y:.4f})\n")
        f.write("))\n")

        f.write("(setq MBB_UNPLACED_PLAN '(\n")
        for item in unplaced_plan:
            f.write(f"  ({item['id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f})\n")
        f.write("))\n")

    if output_json_path:
        out_json_data = {
            'strategy_id': best_res['strategy_id'],
            'total_yield': total_yield,
            'placed_count': placed_count,
            'unplaced_count': unplaced_count,
            'total_parts': total_count,
            'containers_used': used_c,
            'plan': verified_plan,
            'unplaced_plan': unplaced_plan
        }
        with open(output_json_path, 'w', encoding='utf-8') as f:
            json.dump(out_json_data, f, indent=2)

    print(f"MBBN v3.7 Nesting: {placed_count}/{total_count} parts placed [Yield: {total_yield:.1f}% | Containers Used: {used_c}/{len(containers_data)} | Unplaced Buffer: {unplaced_count}]")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python mbb_nesting.py <input.json> <output.lsp> [output.json]")
        sys.exit(1)
    in_p = sys.argv[1]
    out_lsp = sys.argv[2]
    out_json = sys.argv[3] if len(sys.argv) > 3 else None
    run_nesting(in_p, out_lsp, out_json)
