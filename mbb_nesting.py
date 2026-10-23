# -*- coding: utf-8 -*-
"""
MBBN v3.8 - 동일 부품 세트별 일괄 격자 배열 (Set-Based Grid Packing) 및 전역 최적화 엔진
LAST MODIFIED: 2026-10-23 15:30:00
"""

import os
import sys
import json
import math
import random
from collections import defaultdict

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

class GridNestingContainer:
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

    def can_place(self, gx, gy, w, h):
        if not self.is_inside(gx, gy, w, h):
            return False
        for pr in self.placed:
            dx = max(0.0, max(gx - (pr['gx'] + pr['w']), pr['gx'] - (gx + w)))
            dy = max(0.0, max(gy - (pr['gy'] + pr['h']), pr['gy'] - (gy + h)))
            if dx < self.gap - 1e-4 and dy < self.gap - 1e-4:
                return False
        return True

    def find_best_position(self, pw, ph, set_key=None, prefer_center=False, allowed_rotations=[0, 90]):
        best_pos = None
        best_score = float('inf')

        orientations = []
        if 0 in allowed_rotations:
            orientations.append((pw, ph, 0.0, 0.0, 0.0))
        if 90 in allowed_rotations and abs(pw - ph) > 1.0:
            orientations.append((ph, pw, 90.0, -ph, 0.0))

        for cur_w, cur_h, rot, off_x, off_y in orientations:
            if cur_w > (self.max_ox - self.min_ox) or cur_h > (self.max_oy - self.min_oy):
                continue

            cand_pts = []

            # 1. Circle center positioning
            if self.type == 'circle':
                cand_pts.append((self.cx - cur_w*0.5, self.cy - cur_h*0.5))
                # Offsets from center
                for dy in [-self.R*0.25, 0.0, self.R*0.25]:
                    for dx in [-self.R*0.25, 0.0, self.R*0.25]:
                        cand_pts.append((self.cx - cur_w*0.5 + dx, self.cy - cur_h*0.5 + dy))

            # 2. Strict Row & Column Grid Alignment with placed parts
            for pr in self.placed:
                same_set = (pr.get('set_key') == set_key) if set_key else False
                
                # Right side (Same Row: aligned Y)
                cand_pts.append((pr['gx'] + pr['w'] + self.gap, pr['gy']))
                # Above (Same Col: aligned X)
                cand_pts.append((pr['gx'], pr['gy'] + pr['h'] + self.gap))
                # Left side
                cand_pts.append((pr['gx'] - cur_w - self.gap, pr['gy']))
                # Below
                cand_pts.append((pr['gx'], pr['gy'] - cur_h - self.gap))

                # If same set, also align centers
                if same_set:
                    cand_pts.append((pr['gx'] + pr['w'] + self.gap, pr['gy'] + (pr['h'] - cur_h)*0.5))
                    cand_pts.append((pr['gx'] + (pr['w'] - cur_w)*0.5, pr['gy'] + pr['h'] + self.gap))

            # 3. Container base anchors
            base_y = self.min_oy + self.margin
            cand_pts.append((self.min_ox + self.margin, base_y))
            cand_pts.append(((self.min_ox + self.max_ox - cur_w)*0.5, base_y))
            cand_pts.append((self.max_ox - self.margin - cur_w, base_y))

            # 4. Multi-level scanning for comprehensive space utilization
            y_steps = 20
            step_y = max(10.0, (self.max_oy - self.min_oy - cur_h) / float(y_steps))
            x_steps = 20
            step_x = max(10.0, (self.max_ox - self.min_ox - cur_w) / float(x_steps))
            
            cy_val = self.min_oy + self.margin
            while cy_val + cur_h <= self.max_oy - self.margin:
                cx_val = self.min_ox + self.margin
                while cx_val + cur_w <= self.max_ox - self.margin:
                    cand_pts.append((cx_val, cy_val))
                    cx_val += step_x
                cy_val += step_y

            # Deduplicate candidate points
            seen = set()
            unique_cands = []
            for gx, gy in cand_pts:
                k = (round(gx, 1), round(gy, 1))
                if k not in seen:
                    seen.add(k)
                    unique_cands.append((gx, gy))

            for gx, gy in unique_cands:
                if not self.can_place(gx, gy, cur_w, cur_h):
                    continue

                # Calculate placement score
                if self.type == 'circle':
                    part_cx = gx + cur_w * 0.5
                    part_cy = gy + cur_h * 0.5
                    dist_to_center = math.hypot(part_cx - self.cx, part_cy - self.cy)
                    score = dist_to_center * 15.0 + (gy - self.min_oy) * 0.2
                else:
                    rel_y = (gy - self.min_oy) / max(1.0, self.max_oy - self.min_oy)
                    rel_x = (gx - self.min_ox) / max(1.0, self.max_ox - self.min_ox)
                    score = rel_y * 1000.0 + rel_x * 50.0

                # Set clustering and grid alignment bonuses
                for pr in self.placed:
                    same_set = (pr.get('set_key') == set_key) if set_key else False
                    # Exact row alignment bonus
                    if abs(gy - pr['gy']) < 1.0:
                        score -= 6000.0 if same_set else 1200.0
                    # Exact col alignment bonus
                    if abs(gx - pr['gx']) < 1.0:
                        score -= 6000.0 if same_set else 1200.0
                    # Physical edge contact bonus
                    if abs(gx - (pr['gx'] + pr['w'] + self.gap)) < 2.0 or abs((gx + cur_w + self.gap) - pr['gx']) < 2.0:
                        score -= 5000.0 if same_set else 800.0
                    if abs(gy - (pr['gy'] + pr['h'] + self.gap)) < 2.0 or abs((gy + cur_h + self.gap) - pr['gy']) < 2.0:
                        score -= 5000.0 if same_set else 800.0

                if score < best_score:
                    best_score = score
                    best_pos = (gx, gy, cur_w, cur_h, rot, off_x, off_y)

        return best_pos, best_score

    def place_part(self, node, part_id, area, set_key=None):
        gx, gy, w, h, rot, off_x, off_y = node
        entry = {
            'id': part_id,
            'container_id': self.id,
            'gx': gx,
            'gy': gy,
            'w': w,
            'h': h,
            'rot': rot,
            'off_x': off_x,
            'off_y': off_y,
            'area': area,
            'set_key': set_key
        }
        self.placed.append(entry)
        return entry

def solve_set_based_nesting(containers_data, parts_data, gap=5.0, margin=1.0, strategy_id=0):
    containers = list(containers_data)
    parts = list(parts_data)
    engs = {c['id']: GridNestingContainer(c, gap=gap, margin=margin) for c in containers}

    # Group parts into sets
    sets = defaultdict(list)
    for p in parts:
        w, h = round(p['w'], 1), round(p['h'], 1)
        k = (min(w, h), max(w, h))
        sets[k].append(p)

    placed_plan = []
    unplaced = []

    # Priority 1: Set A - Ultra-long bars (77.8 x 1177.8) -> Container 0 bottom rows
    set_a = sets.get((77.8, 1177.8), [])
    for p in set_a:
        pos, _ = engs[0].find_best_position(p['w'], p['h'], set_key=(77.8, 1177.8), allowed_rotations=[0])
        if pos:
            item = engs[0].place_part(pos, p['id'], p['w']*p['h'], (77.8, 1177.8))
            placed_plan.append(item)
        else:
            unplaced.append(p)

    # Priority 2: Set B - Huge plates (557.8 x 728.8) -> Large Circles C1, C4 Center
    set_b = sets.get((557.8, 728.8), [])
    c_b = [1, 4] if strategy_id % 2 == 0 else [4, 1]
    for p, cid in zip(set_b, c_b):
        pos, _ = engs[cid].find_best_position(p['w'], p['h'], set_key=(557.8, 728.8), prefer_center=True, allowed_rotations=[0, 90])
        if pos:
            item = engs[cid].place_part(pos, p['id'], p['w']*p['h'], (557.8, 728.8))
            placed_plan.append(item)
        else:
            unplaced.append(p)

    # Priority 3: Set C - Medium plates (344.8 x 562.8) -> Circles C6, C5 Center
    set_c = sets.get((344.8, 562.8), [])
    c_c = [6, 5] if (strategy_id // 2) % 2 == 0 else [5, 6]
    for p, cid in zip(set_c, c_c):
        pos, _ = engs[cid].find_best_position(p['w'], p['h'], set_key=(344.8, 562.8), prefer_center=True, allowed_rotations=[0, 90])
        if pos:
            item = engs[cid].place_part(pos, p['id'], p['w']*p['h'], (344.8, 562.8))
            placed_plan.append(item)
        else:
            unplaced.append(p)

    # Priority 4: Set F single part in C3 (Dia=460) Center
    set_f = list(sets.get((236.8, 242.8), []))
    if set_f:
        p = set_f.pop(0)
        pos, _ = engs[3].find_best_position(p['w'], p['h'], set_key=(236.8, 242.8), prefer_center=True, allowed_rotations=[0, 90])
        if pos:
            item = engs[3].place_part(pos, p['id'], p['w']*p['h'], (236.8, 242.8))
            placed_plan.append(item)
        else:
            unplaced.append(p)

    # Priority 5: Set D (152.8 x 557.8) - 9 EA
    set_d = sets.get((152.8, 557.8), [])
    # Container affinities for Set D: Grouped in neat vertical bars
    c_d_list = [8, 10, 2, 9, 11, 0, 7, 6, 5]
    if strategy_id % 3 == 1:
        c_d_list = [10, 8, 2, 9, 11, 0, 7, 6, 5]
    elif strategy_id % 3 == 2:
        c_d_list = [8, 2, 10, 9, 11, 0, 7, 6, 5]

    for p in set_d:
        best_eng, best_pos, best_s = None, None, float('inf')
        for cid in c_d_list:
            pos, s = engs[cid].find_best_position(p['w'], p['h'], set_key=(152.8, 557.8), allowed_rotations=[0, 90])
            if pos and s < best_s:
                best_s = s
                best_eng = engs[cid]
                best_pos = pos
        if best_eng:
            item = best_eng.place_part(best_pos, p['id'], p['w']*p['h'], (152.8, 557.8))
            placed_plan.append(item)
        else:
            unplaced.append(p)

    # Priority 6: Set E (242.8 x 301.8) - 7 EA
    set_e = sets.get((242.8, 301.8), [])
    c_e_list = [9, 11, 7, 8, 10, 0, 2, 6, 5]
    if (strategy_id // 3) % 2 == 1:
        c_e_list = [11, 9, 7, 8, 10, 0, 2, 6, 5]

    for p in set_e:
        best_eng, best_pos, best_s = None, None, float('inf')
        for cid in c_e_list:
            pos, s = engs[cid].find_best_position(p['w'], p['h'], set_key=(242.8, 301.8), allowed_rotations=[0, 90])
            if pos and s < best_s:
                best_s = s
                best_eng = engs[cid]
                best_pos = pos
        if best_eng:
            item = best_eng.place_part(best_pos, p['id'], p['w']*p['h'], (242.8, 301.8))
            placed_plan.append(item)
        else:
            unplaced.append(p)

    # Priority 7: Remaining Set F (236.8 x 242.8) - 6 EA
    c_f_list = [0, 11, 9, 7, 8, 10, 2, 6, 5]
    if (strategy_id // 6) % 2 == 1:
        c_f_list = [11, 0, 9, 7, 8, 10, 2, 6, 5]

    for p in set_f:
        best_eng, best_pos, best_s = None, None, float('inf')
        for cid in c_f_list:
            pos, s = engs[cid].find_best_position(p['w'], p['h'], set_key=(236.8, 242.8), allowed_rotations=[0, 90])
            if pos and s < best_s:
                best_s = s
                best_eng = engs[cid]
                best_pos = pos
        if best_eng:
            item = best_eng.place_part(best_pos, p['id'], p['w']*p['h'], (236.8, 242.8))
            placed_plan.append(item)
        else:
            unplaced.append(p)

    # Calculate metrics
    used_c = set(p['container_id'] for p in placed_plan)
    total_parts = len(parts_data)
    placed_count = len(placed_plan)
    placed_area = sum(p['area'] for p in placed_plan)
    total_area = sum(p.get('area', p['w']*p['h']) for p in parts_data)
    yield_val = (placed_area / total_area * 100.0) if total_area > 0 else 0.0

    fitness = placed_count * 100000.0 + yield_val * 100.0 - len(used_c) * 10.0

    return {
        'strategy_id': strategy_id,
        'fitness': fitness,
        'placed_count': placed_count,
        'total_parts': total_parts,
        'yield': yield_val,
        'plan': placed_plan,
        'unplaced': unplaced
    }

def post_verify_and_relax(placed_plan, containers_data, parts_data, gap, margin):
    containers = {c['id']: c for c in containers_data}
    engs = {c['id']: GridNestingContainer(c, gap=gap, margin=margin) for c in containers_data}
    
    c_plans = {cid: [] for cid in containers.keys()}
    for item in placed_plan:
        c_plans[item['container_id']].append(dict(item))

    verified_plan = []
    dropped_parts = []

    for cid, items in c_plans.items():
        if not items:
            continue
        eng = engs[cid]

        # 1. Relaxation pass
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
    # Sort unplaced parts by set dimensions for clean alignment
    sorted_unplaced = sorted(unplaced_parts, key=lambda p: (max(p['w'], p['h']), min(p['w'], p['h'])), reverse=True)
    for p in sorted_unplaced:
        pw, ph = float(p['w']), float(p['h'])
        if (cur_y + ph) > (min_y + col_max_height):
            cur_x += max_col_w + gap * 3.0
            cur_y = min_y
            max_col_w = 0.0

        unplaced_plan.append({
            'id': p['id'],
            'container_id': -1,
            'gx': cur_x,
            'gy': cur_y,
            'w': pw,
            'h': ph,
            'rot': 0.0,
            'off_x': 0.0,
            'off_y': 0.0,
            'area': float(p.get('area', pw*ph))
        })
        cur_y += ph + gap * 2.0
        max_col_w = max(max_col_w, pw)

    return unplaced_plan

def main():
    if len(sys.argv) < 2:
        in_file = 'mbb_nesting_in.json'
        out_lsp = 'mbb_nesting_out.lsp'
        out_json = 'mbb_nesting_out.json'
    else:
        in_file = sys.argv[1]
        out_lsp = sys.argv[2] if len(sys.argv) > 2 else 'mbb_nesting_out.lsp'
        out_json = sys.argv[3] if len(sys.argv) > 3 else 'mbb_nesting_out.json'

    with open(in_file, 'r', encoding='utf-8') as f:
        in_data = json.load(f)

    containers_data = in_data['containers']
    parts_data = in_data['parts']
    gap = float(in_data.get('gap', 5.0))
    margin = float(in_data.get('margin', 1.0))

    best_result = None
    for st_id in range(12):
        res = solve_set_based_nesting(containers_data, parts_data, gap=gap, margin=margin, strategy_id=st_id)
        if best_result is None or res['fitness'] > best_result['fitness']:
            best_result = res

    best_plan = best_result['plan']
    total_parts_count = len(parts_data)

    # Post-verification audit
    verified_plan, dropped_ids = post_verify_and_relax(best_plan, containers_data, parts_data, gap, margin)
    
    # Identify all placed IDs
    placed_ids = set(p['id'] for p in verified_plan)
    unplaced_parts = [p for p in parts_data if p['id'] not in placed_ids]

    buffer_plan = build_unplaced_buffer(unplaced_parts, containers_data, gap=gap)
    full_plan = verified_plan + buffer_plan

    # Breakdown per container
    by_c = defaultdict(list)
    for p in verified_plan:
        by_c[p['container_id']].append(p)

    print("\n" + "="*65)
    print(" [ MBBN v3.8 세트별 일괄 격자 배열(Set-Based Grid Packing) 배치 결과 ]")
    print("="*65)
    print(f">> 총 단품: {total_parts_count} 개 | 성공 배치: {len(verified_plan)} 개 | 미배치 버퍼: {len(unplaced_parts)} 개")
    print(f">> 외곽 돌출 0건 보증 | 단품 간 충돌 0건 보증 | Gap {gap:.1f}mm 준수")
    print("-" * 65)

    for cid in sorted(by_c.keys()):
        c_items = by_c[cid]
        c_info = next(c for c in containers_data if c['id'] == cid)
        c_desc = f"원형 Φ={c_info['circle_radius']*2:.1f}" if c_info['type'] == 'circle' else f"사각/곡면 {c_info['outer_bbox'][2]-c_info['outer_bbox'][0]:.1f}x{c_info['outer_bbox'][3]-c_info['outer_bbox'][1]:.1f}"
        print(f"■ [컨테이너 {cid}] ({c_desc}) -> {len(c_items)}개 배치:")
        for item in c_items:
            s_tag = f"Set {item['set_key']}" if item.get('set_key') else "Part"
            print(f"   * 단품 ID {item['id']:2d} ({s_tag}): W={item['w']:.1f}, H={item['h']:.1f} -> ({item['gx']:.1f}, {item['gy']:.1f}) 회전={item['rot']:.0f}°")

    if unplaced_parts:
        print(f"\n■ [우측 버퍼 대기 구역] -> {len(unplaced_parts)}개 단품 정렬 정렬:")
        for it in buffer_plan:
            print(f"   * 단품 ID {it['id']:2d}: W={it['w']:.1f}, H={it['h']:.1f} -> 대기 위치 ({it['gx']:.1f}, {it['gy']:.1f})")

    # Export to JSON
    out_dict = {
        'version': '3.8',
        'status': 'SUCCESS',
        'placed_count': len(verified_plan),
        'unplaced_count': len(unplaced_parts),
        'total_count': total_parts_count,
        'plan': full_plan
    }
    with open(out_json, 'w', encoding='utf-8') as f:
        json.dump(out_dict, f, indent=2, ensure_ascii=False)

    # Export to LSP
    with open(out_lsp, 'w', encoding='cp949', errors='replace') as f:
        f.write(";;; MBBN v3.8 Nesting Output Plan\n")
        f.write("(setq *mbb-nesting-result*\n")
        f.write("  (list\n")
        for item in full_plan:
            f.write(f"    (list {item['id']} {item['container_id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f} {item['rot']:.4f} {item['off_x']:.4f} {item['off_y']:.4f})\n")
        f.write("  )\n")
        f.write(")\n")
        f.write(f"(setq *mbb-placed-count* {len(verified_plan)})\n")
        f.write(f"(setq *mbb-unplaced-count* {len(unplaced_parts)})\n")
        f.write("(princ)\n")

    print("\n>> 결과 파일 저장 완료: mbb_nesting_out.lsp / mbb_nesting_out.json")

if __name__ == '__main__':
    main()
