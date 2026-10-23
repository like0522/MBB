# -*- coding: utf-8 -*-
"""
;;; ==========================================================================
;;; MBBN v4.3 - Generational Tournament Competitive Evolutionary Nesting Engine
;;; ==========================================================================
;;; PROGRAM    : MBBN Nesting & Array Solver (다세대 경쟁 진화 수율 극대화 엔진)
;;; AUTHOR     : Antigravity CAD/AutoLISP Senior Engineer
;;; LAST MODIFIED: 2026-10-23 20:55:00
;;;
;;; [VERSION HISTORY & ROLLBACK LOG]
;;; - v4.3 (2026-10-23 20:55:00) : 다세대 토너먼트 경쟁 진화(Competitive Evolutionary) 엔진 탑재
;;;                                 1. 세대별 토너먼트 경쟁 (Generation Population Tournament Selection)
;;;                                 2. 챔피언 유전자 기반 변이 & 국소 개선 (Mutation: 순서 교환, 회전 토글, 적층 변이)
;;;                                 3. 차세대 재경쟁으로 수율 한계 지속 돌파 (Elitism & Title Defense)
;;;                                 4. 원형/다각형 내 동일 세트 단품 평행 적층 (Parallel Strip Stacking) 완벽 지원
;;;                                 5. 100% 무간섭 (Gap 5.0mm) & 무돌출 (Margin 1.0mm) 물리 엔진 검증
;;; - v4.2 (2026-10-23 20:25:00) : Actcut 100-Pass Multi-Strategy Engine
;;; - v4.1 (2026-10-23 19:53:00) : Actcut Core Reverse-Engineering Integration
;;; - v3.0 (2026-10-23 08:25:00) : Legacy Backtracking Engine
;;; ==========================================================================
"""

import os
import sys
import json
import math
import random
import time
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


class EvolutionContainer:
    """
    High-Performance Container Solver for Generational Evolutionary Nesting:
    - Inner-Fit Polygon (Margin)
    - No-Fit Polygon clearance (Gap)
    - Sliding Contact & Boundary Anchors
    - Gravity-based compaction (doFloat)
    """
    def __init__(self, c_info, gap=5.0, margin=1.0):
        self.id = c_info['id']
        self.type = c_info.get('type', 'polygon')
        self.gap = float(gap)
        self.margin = float(margin)
        self.area = float(c_info.get('area', 1.0))
        self.placed = []
        self.placed_area = 0.0

        if self.type == 'circle':
            self.cx, self.cy = c_info['circle_center']
            self.R = float(c_info['circle_radius'])
            self.min_ox = self.cx - self.R
            self.min_oy = self.cy - self.R
            self.max_ox = self.cx + self.R
            self.max_oy = self.cy + self.R
            self.eff_R = max(0.0, self.R - self.margin)
            self.eff_R_sq = self.eff_R * self.eff_R
            self.prep_inner = None
        else:
            self.outer_pts = c_info['outer_polygon']
            self.min_ox, self.min_oy, self.max_ox, self.max_oy = c_info['outer_bbox']
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
            corners = [
                (gx, gy),
                (gx + w, gy),
                (gx, gy + h),
                (gx + w, gy + h)
            ]
            for x, y in corners:
                dx = x - self.cx
                dy = y - self.cy
                if (dx * dx + dy * dy) > (self.eff_R_sq + 1e-4):
                    return False
            return True
        else:
            if gx < (self.min_ox + self.margin - 1e-4) or gy < (self.min_oy + self.margin - 1e-4) or \
               (gx + w) > (self.max_ox - self.margin + 1e-4) or (gy + h) > (self.max_oy - self.margin + 1e-4):
                return False
            if self.prep_inner:
                return self.prep_inner.contains(box(gx, gy, gx + w, gy + h))
            else:
                for x, y in [(gx, gy), (gx + w, gy), (gx, gy + h), (gx + w, gy + h)]:
                    if not is_point_in_poly(x, y, self.outer_pts):
                        return False
                return True

    def can_place(self, gx, gy, w, h):
        for p in self.placed:
            if not (gx + w + self.gap <= p['gx'] + 1e-4 or
                    gx >= p['gx'] + p['w'] + self.gap - 1e-4 or
                    gy + h + self.gap <= p['gy'] + 1e-4 or
                    gy >= p['gy'] + p['h'] + self.gap - 1e-4):
                return False
        return self.is_inside(gx, gy, w, h)

    def find_best_position(self, pw, ph, allowed_rotations=[0, 90], prefer_center=False):
        rem_area = self.area - self.placed_area
        if pw * ph > rem_area:
            return None
        if min(pw, ph) > (self.max_ox - self.min_ox) or min(pw, ph) > (self.max_oy - self.min_oy):
            return None
        if self.type == 'circle' and (pw * pw + ph * ph) > (4.0 * self.eff_R_sq):
            return None

        orientations = []
        if 0 in allowed_rotations:
            orientations.append((pw, ph, 0.0, 0.0, 0.0))
        if 90 in allowed_rotations and abs(pw - ph) > 1.0:
            orientations.append((ph, pw, 90.0, -ph, 0.0))

        best_cand = None
        best_score = float('inf')

        for cur_w, cur_h, rot, off_x, off_y in orientations:
            if cur_w > (self.max_ox - self.min_ox) or cur_h > (self.max_oy - self.min_oy):
                continue

            cands = []
            if self.type == 'circle':
                cands.append((self.cx - cur_w * 0.5, self.cy - cur_h * 0.5))
                for frac in [0.2, 0.35, 0.5]:
                    for ang_deg in range(0, 360, 45):
                        rad = math.radians(ang_deg)
                        cands.append((self.cx - cur_w * 0.5 + self.R * frac * math.cos(rad),
                                      self.cy - cur_h * 0.5 + self.R * frac * math.sin(rad)))

            for p in self.placed:
                cands.append((p['gx'] + p['w'] + self.gap, p['gy']))
                cands.append((p['gx'], p['gy'] + p['h'] + self.gap))
                cands.append((p['gx'] - cur_w - self.gap, p['gy']))
                cands.append((p['gx'], p['gy'] - cur_h - self.gap))
                cands.append((p['gx'] + p['w'] + self.gap, p['gy'] + (p['h'] - cur_h)))
                cands.append((p['gx'] + (p['w'] - cur_w), p['gy'] + p['h'] + self.gap))
                cands.append((p['gx'] + p['w'] + self.gap, p['gy'] + p['h'] + self.gap))

            cands.append((self.min_ox + self.margin, self.min_oy + self.margin))
            cands.append(((self.min_ox + self.max_ox - cur_w) * 0.5, self.min_oy + self.margin))
            cands.append((self.max_ox - self.margin - cur_w, self.min_oy + self.margin))

            if len(self.placed) < 3:
                scan_res = 8
                step_x = (self.max_ox - self.min_ox - cur_w) / max(1, scan_res)
                step_y = (self.max_oy - self.min_oy - cur_h) / max(1, scan_res)
                for sy in range(scan_res + 1):
                    cy = self.min_oy + self.margin + sy * step_y
                    for sx in range(scan_res + 1):
                        cx = self.min_ox + self.margin + sx * step_x
                        cands.append((cx, cy))

            for gx, gy in cands:
                if self.can_place(gx, gy, cur_w, cur_h):
                    if prefer_center:
                        dist_c = math.hypot(gx + cur_w * 0.5 - (self.min_ox + self.max_ox) * 0.5,
                                            gy + cur_h * 0.5 - (self.min_oy + self.max_oy) * 0.5)
                        score = dist_c
                    else:
                        score = (gy - self.min_oy) * 1.5 + (gx - self.min_ox)
                    if score < best_score:
                        best_score = score
                        best_cand = (gx, gy, cur_w, cur_h, rot, off_x, off_y)

        return best_cand

    def do_float(self, p_entry):
        """Gravity sliding toward bottom-left boundaries."""
        gx, gy, w, h = p_entry['gx'], p_entry['gy'], p_entry['w'], p_entry['h']
        self.placed.remove(p_entry)
        for step in [20.0, 5.0, 1.0]:
            while gy - step >= (self.min_oy + self.margin):
                if self.can_place(gx, gy - step, w, h):
                    gy -= step
                else:
                    break
        for step in [20.0, 5.0, 1.0]:
            while gx - step >= (self.min_ox + self.margin):
                if self.can_place(gx - step, gy, w, h):
                    gx -= step
                else:
                    break
        p_entry['gx'] = gx
        p_entry['gy'] = gy
        self.placed.append(p_entry)


def evaluate_individual_genome(genome, containers_data, gap=5.0, margin=1.0):
    """Evaluates one placement genome and returns its physical layout and fitness score."""
    c_map = {c['id']: EvolutionContainer(c, gap, margin) for c in containers_data}
    c_ids = list(genome['c_order'])
    parts_order = list(genome['parts_order'])
    use_strip = genome.get('use_strip', True)
    rot_prefs = genome.get('rot_prefs', {})

    placed_plan = []
    remaining = list(parts_order)

    # 1. Parallel Strip Stacking for identical part clusters (Horizontal & Vertical)
    if use_strip:
        clusters = defaultdict(list)
        for p in remaining:
            w, h = round(p['w'], 1), round(p['h'], 1)
            clusters[(min(w, h), max(w, h))].append(p)

        for k in sorted(clusters.keys(), key=lambda k: k[0] * k[1] + k[1] * 300.0, reverse=True):
            p_list = clusters[k]
            if len(p_list) < 2:
                continue
            pw, ph = p_list[0]['w'], p_list[0]['h']

            for cid in c_ids:
                cnt = c_map[cid]
                if cnt.placed:
                    continue

                stack_succeeded = False
                # Try Horizontal Stack (pw x ph along Y)
                for num_bars in range(len(p_list), 1, -1):
                    total_h = num_bars * ph + (num_bars - 1) * cnt.gap
                    if cnt.type == 'circle':
                        start_x = cnt.cx - pw * 0.5
                        start_y = cnt.cy - total_h * 0.5
                    else:
                        start_x = cnt.min_ox + cnt.margin
                        start_y = cnt.min_oy + cnt.margin

                    all_fit = True
                    stack_entries = []
                    for bi in range(num_bars):
                        cur_y = start_y + bi * (ph + cnt.gap)
                        if not cnt.can_place(start_x, cur_y, pw, ph):
                            all_fit = False
                            break
                        stack_entries.append((p_list[bi], start_x, cur_y, pw, ph, 0.0, 0.0, 0.0))

                    if all_fit:
                        for b_part, sx, sy, sw, sh, srot, ox, oy in stack_entries:
                            e = {'id': b_part['id'], 'container_id': cid, 'gx': sx, 'gy': sy, 'w': sw, 'h': sh, 'rot': srot, 'off_x': ox, 'off_y': oy, 'area': sw * sh}
                            cnt.placed.append(e)
                            cnt.placed_area += e['area']
                            placed_plan.append(e)
                            remaining.remove(b_part)
                        stack_succeeded = True
                        break

                if stack_succeeded:
                    break

    # 2. Adaptive Unitary Placement with Sliding Contact for remaining parts
    for p in remaining:
        pw, ph = p['w'], p['h']
        allowed_rots = rot_prefs.get(p['id'], [0, 90])
        best_cnt, best_pos, best_s = None, None, float('inf')
        prefer_c = (pw * ph > 200000.0)

        for cid in c_ids:
            cnt = c_map[cid]
            p_c = prefer_c and (cnt.type == 'circle')
            pos = cnt.find_best_position(pw, ph, allowed_rotations=allowed_rots, prefer_center=p_c)
            if pos:
                gx, gy, cur_w, cur_h, rot, ox, oy = pos
                s = (gy - cnt.min_oy) * 1.2 + (gx - cnt.min_ox)
                if s < best_s:
                    best_s = s
                    best_cnt = cnt
                    best_pos = pos

        if best_cnt and best_pos:
            gx, gy, cur_w, cur_h, rot, ox, oy = best_pos
            entry = {
                'id': p['id'],
                'container_id': best_cnt.id,
                'gx': gx, 'gy': gy,
                'w': cur_w, 'h': cur_h,
                'rot': rot, 'off_x': ox, 'off_y': oy,
                'area': cur_w * cur_h
            }
            best_cnt.placed.append(entry)
            best_cnt.placed_area += entry['area']
            best_cnt.do_float(entry)
            placed_plan.append(entry)

    placed_count = len(placed_plan)
    total_area = sum(it['area'] for it in placed_plan)
    # Balanced Fitness: Area (Material Yield) + Count Multiplier
    score = total_area + placed_count * 100000.0

    return {
        'score': score,
        'placed_count': placed_count,
        'total_area': total_area,
        'plan': placed_plan,
        'genome': genome
    }


def solve_competitive_evolution(containers_data, parts_data, gap=5.0, margin=1.0, pop_size=15, num_generations=25):
    """
    Generational Tournament Competitive Evolutionary Nesting Solver:
    - Generates diverse population seeds.
    - Evaluates population in tournament competition.
    - Preserves reigning champion (Elitism).
    - Mutates top elites (part swap, container shuffle, rotation toggle).
    - Competes across generations until global champion emerges.
    """
    t0 = time.time()
    print("\n" + "=" * 70)
    print(" [ MBBN v4.3 다세대 경쟁 진화(Competitive Evolutionary) 네스팅 엔진 가동 ]")
    print("=" * 70)
    print(f">> 탐색 모드: 세대당 {pop_size}개체 x {num_generations}세대 토너먼트 경쟁 (총 {pop_size * num_generations}회 대진)")
    print("-" * 70, flush=True)

    # 1. Initial Population Seeding
    population = []
    # Seed A: Bigness priority with strip stack
    p_bigness = sorted(parts_data, key=lambda p: (p['w'] * p['h'] + max(p['w'], p['h']) * 350.0), reverse=True)
    population.append({
        'parts_order': p_bigness,
        'c_order': [c['id'] for c in containers_data],
        'use_strip': True,
        'rot_prefs': {p['id']: [0, 90] for p in parts_data}
    })
    # Seed B: Area priority without strip stack
    p_area = sorted(parts_data, key=lambda p: p['w'] * p['h'], reverse=True)
    population.append({
        'parts_order': p_area,
        'c_order': [c['id'] for c in containers_data],
        'use_strip': False,
        'rot_prefs': {p['id']: [0, 90] for p in parts_data}
    })
    # Seed C: Container area descending priority
    c_area_desc = sorted([c['id'] for c in containers_data], key=lambda cid: next(c['area'] for c in containers_data if c['id'] == cid), reverse=True)
    population.append({
        'parts_order': p_bigness,
        'c_order': c_area_desc,
        'use_strip': True,
        'rot_prefs': {p['id']: [0, 90] for p in parts_data}
    })
    # Fill remaining population with diversified stochastic permutations
    for i in range(pop_size - len(population)):
        p_shuffled = list(p_bigness)
        random.seed(i * 23 + 11)
        for _ in range(4):
            i1 = random.randint(0, len(p_shuffled) - 1)
            i2 = random.randint(0, len(p_shuffled) - 1)
            p_shuffled[i1], p_shuffled[i2] = p_shuffled[i2], p_shuffled[i1]
        c_ord = list(c['id'] for c in containers_data)
        if random.random() < 0.5:
            random.shuffle(c_ord)
        population.append({
            'parts_order': p_shuffled,
            'c_order': c_ord,
            'use_strip': random.choice([True, False]),
            'rot_prefs': {p['id']: [0, 90] for p in parts_data}
        })

    reigning_champion = None
    champion_gen = 0

    # 2. Generational Tournament Loop
    for gen in range(num_generations):
        eval_results = []
        for ind in population:
            res = evaluate_individual_genome(ind, containers_data, gap=gap, margin=margin)
            eval_results.append(res)

        # Sort by fitness score descending (토너먼트 순위 결정)
        eval_results.sort(key=lambda r: r['score'], reverse=True)
        gen_champion = eval_results[0]

        # Champion Title Defense & Renewal
        if reigning_champion is None or gen_champion['score'] > reigning_champion['score']:
            reigning_champion = gen_champion
            champion_gen = gen + 1
            print(f">> [Gen {gen+1:2d}/{num_generations}] ★ 챔피언 타이틀 경신! 안착: {reigning_champion['placed_count']:2d}/{len(parts_data)}개 | 면적: {reigning_champion['total_area']:.1f} mm² (Score: {reigning_champion['score']:.0f})", flush=True)

        # 3. Reproduction, Elitism & Genetic Mutation for Next Generation
        new_pop = [reigning_champion['genome']] # Elitism (최고 챔피언 영구 보존)
        top_elites = [r['genome'] for r in eval_results[:4]]

        while len(new_pop) < pop_size:
            parent = random.choice(top_elites)
            child_parts = list(parent['parts_order'])
            child_c = list(parent['c_order'])
            child_strip = parent['use_strip']
            child_rots = dict(parent['rot_prefs'])

            # Mutation A: Part sequence swap
            if random.random() < 0.65:
                idx1 = random.randint(0, len(child_parts) - 1)
                idx2 = random.randint(0, len(child_parts) - 1)
                child_parts[idx1], child_parts[idx2] = child_parts[idx2], child_parts[idx1]

            # Mutation B: Container order shuffle
            if random.random() < 0.35:
                random.shuffle(child_c)

            # Mutation C: Strip stack toggle
            if random.random() < 0.25:
                child_strip = not child_strip

            # Mutation D: Rotation preference toggle
            if random.random() < 0.3:
                rand_id = random.choice(child_parts)['id']
                child_rots[rand_id] = random.choice([[0], [90], [0, 90]])

            new_pop.append({
                'parts_order': child_parts,
                'c_order': child_c,
                'use_strip': child_strip,
                'rot_prefs': child_rots
            })

        population = new_pop

    elapsed = time.time() - t0
    print("-" * 70)
    print(f">> 다세대 진화 경쟁 완결! (소요 시간: {elapsed:.2f}초)")
    print(f">> [최종 선정 절대 챔피언] Gen {champion_gen} 출신 | 안착 {reigning_champion['placed_count']}개 / 총 면적 {reigning_champion['total_area']:.1f} mm²", flush=True)

    return reigning_champion


def post_verify_and_relax(plan, containers_data, parts_data, gap, margin):
    """Independent audit to guarantee 0mm overlap and 0mm protrusion."""
    c_map = {c['id']: c for c in containers_data}
    verified_plan = []
    dropped_ids = []

    by_c = defaultdict(list)
    for it in plan:
        by_c[it['container_id']].append(it)

    for cid, items in by_c.items():
        c_info = c_map[cid]
        c_type = c_info.get('type', 'polygon')
        accepted = []

        if c_type == 'circle':
            cx, cy = c_info['circle_center']
            R = float(c_info['circle_radius'])
            max_dist_sq = max(0.0, R - margin) ** 2

            for it in items:
                gx, gy, w, h = it['gx'], it['gy'], it['w'], it['h']
                corners = [(gx, gy), (gx + w, gy), (gx, gy + h), (gx + w, gy + h)]
                breach = any(((x - cx)**2 + (y - cy)**2) > max_dist_sq + 1e-4 for x, y in corners)
                if breach:
                    dropped_ids.append(it['id'])
                    continue

                collision = False
                for ac in accepted:
                    if not (gx + w + gap <= ac['gx'] + 1e-4 or
                            gx >= ac['gx'] + ac['w'] + gap - 1e-4 or
                            gy + h + gap <= ac['gy'] + 1e-4 or
                            gy >= ac['gy'] + ac['h'] + gap - 1e-4):
                        collision = True
                        break

                if not collision:
                    accepted.append(it)
                    verified_plan.append(it)
                else:
                    dropped_ids.append(it['id'])
        else:
            poly = Polygon(c_info['outer_polygon'])
            if not poly.is_valid:
                poly = poly.buffer(0)
            inner_poly = poly.buffer(-margin)
            if inner_poly.is_empty:
                inner_poly = poly
            prep_inner = prep(inner_poly)

            for it in items:
                gx, gy, w, h = it['gx'], it['gy'], it['w'], it['h']
                b = box(gx, gy, gx + w, gy + h)
                if not prep_inner.contains(b):
                    dropped_ids.append(it['id'])
                    continue

                collision = False
                for ac in accepted:
                    if not (gx + w + gap <= ac['gx'] + 1e-4 or
                            gx >= ac['gx'] + ac['w'] + gap - 1e-4 or
                            gy + h + gap <= ac['gy'] + 1e-4 or
                            gy >= ac['gy'] + ac['h'] + gap - 1e-4):
                        collision = True
                        break

                if not collision:
                    accepted.append(it)
                    verified_plan.append(it)
                else:
                    dropped_ids.append(it['id'])

    return verified_plan, dropped_ids


def build_unplaced_buffer(unplaced_parts, containers_data, gap=5.0):
    """Arranges unplaced parts in aligned columns outside containers."""
    if not unplaced_parts:
        return []

    max_x = max(c.get('outer_bbox', [0, 0, 0, 0])[2] if c.get('type') != 'circle'
                else c['circle_center'][0] + c['circle_radius'] for c in containers_data)
    min_y = min(c.get('outer_bbox', [0, 0, 0, 0])[1] if c.get('type') != 'circle'
                else c['circle_center'][1] - c['circle_radius'] for c in containers_data)

    buf_x = max_x + 500.0
    cur_y = min_y
    col_w = 0.0
    plan = []

    for p in unplaced_parts:
        pw, ph = p['w'], p['h']
        plan.append({
            'id': p['id'],
            'gx': buf_x,
            'gy': cur_y,
            'w': pw,
            'h': ph,
            'rot': 0.0,
            'off_x': 0.0,
            'off_y': 0.0,
            'area': pw * ph
        })
        cur_y += ph + gap
        col_w = max(col_w, pw)
        if cur_y - min_y > 3000.0:
            cur_y = min_y
            buf_x += col_w + gap * 2.0
            col_w = 0.0

    return plan


def main():
    if len(sys.argv) < 2:
        in_file = 'mbb_nesting_in.json'
        out_lsp = 'mbb_nesting_out.lsp'
        out_json = 'mbb_nesting_out.json'
    else:
        in_file = sys.argv[1]
        out_lsp = sys.argv[2] if len(sys.argv) > 2 else 'mbb_nesting_out.lsp'
        out_json = sys.argv[3] if len(sys.argv) > 3 else 'mbb_nesting_out.json'

    if not os.path.exists(in_file):
        print(f"[ERROR] Input file not found: {in_file}")
        return

    with open(in_file, 'r', encoding='utf-8') as f:
        in_data = json.load(f)

    containers_data = in_data['containers']
    parts_data = in_data['parts']
    gap = float(in_data.get('gap', 5.0))
    margin = float(in_data.get('margin', 1.0))
    total_parts_count = len(parts_data)

    # Run Competitive Evolutionary Nesting Solver
    champion = solve_competitive_evolution(containers_data, parts_data, gap=gap, margin=margin, pop_size=15, num_generations=25)
    raw_plan = champion['plan']

    # Strict Audit
    verified_plan, dropped_ids = post_verify_and_relax(raw_plan, containers_data, parts_data, gap, margin)

    placed_ids = set(p['id'] for p in verified_plan)
    unplaced_parts = [p for p in parts_data if p['id'] not in placed_ids]
    buffer_plan = build_unplaced_buffer(unplaced_parts, containers_data, gap=gap)

    placed_area = sum(p['area'] for p in verified_plan)
    total_part_area = sum(p.get('area', p['w'] * p['h']) for p in parts_data)
    part_yield = (placed_area / total_part_area * 100.0) if total_part_area > 0 else 0.0

    used_containers = set(p['container_id'] for p in verified_plan)
    used_c_area = sum(float(c.get('area', 1.0)) for c in containers_data if c['id'] in used_containers)
    container_yield = (placed_area / used_c_area * 100.0) if used_c_area > 0 else 0.0

    by_c = defaultdict(list)
    for p in verified_plan:
        by_c[p['container_id']].append(p)

    print("\n" + "=" * 70)
    print(" [ MBBN v4.3 최종 배치 성적표 (Tournament Champion Layout) ]")
    print("=" * 70)
    print(f">> 총 단품: {total_parts_count} 개 | 성공 배치: {len(verified_plan)} 개 | 미배치 외곽: {len(unplaced_parts)} 개")
    print(f">> 단품 수율(Part Yield): {part_yield:.2f}% | 용기 수율(Container Yield): {container_yield:.2f}%")
    print(f">> [물리 검증] 외곽 침범 0개 (Margin {margin:.1f}mm) | 단품 간 충돌 0개 (Gap {gap:.1f}mm 보장)")
    print("-" * 70)

    for cid in sorted(by_c.keys()):
        c_items = by_c[cid]
        c_info = next(c for c in containers_data if c['id'] == cid)
        c_desc = f"원형 Dia={c_info['circle_radius']*2:.1f}" if c_info.get('type') == 'circle' else f"다각형 {c_info['outer_bbox'][2]-c_info['outer_bbox'][0]:.1f}x{c_info['outer_bbox'][3]-c_info['outer_bbox'][1]:.1f}"
        print(f"■ [용기 {cid}] ({c_desc}) -> {len(c_items)}개 단품 최적 안착:")
        for it in c_items:
            print(f"   * 단품 ID {it['id']:2d}: W={it['w']:.1f}, H={it['h']:.1f} -> ({it['gx']:.1f}, {it['gy']:.1f}) 회전={it['rot']:.0f}°")

    if unplaced_parts:
        print(f"\n■ [잔여 미배치 버퍼] -> {len(unplaced_parts)}개 단품 외곽 정렬:")
        for bp in buffer_plan:
            print(f"   * 단품 ID {bp['id']:2d}: W={bp['w']:.1f}, H={bp['h']:.1f} -> 버퍼 ({bp['gx']:.1f}, {bp['gy']:.1f})")

    # Output JSON
    out_dict = {
        "version": "v4.3",
        "status": "SUCCESS",
        "placed_count": len(verified_plan),
        "unplaced_count": len(unplaced_parts),
        "total_count": total_parts_count,
        "part_yield": part_yield,
        "container_yield": container_yield,
        "plan": verified_plan,
        "unplaced_plan": buffer_plan
    }
    with open(out_json, 'w', encoding='utf-8') as f:
        json.dump(out_dict, f, indent=2)

    # Output AutoLISP Plan (ANSI / CP949 compatible)
    with open(out_lsp, 'w', encoding='cp949', errors='replace') as f:
        f.write(";;; MBBN v4.3 Nesting Plan (Generational Tournament Evolutionary Engine)\n")
        f.write(f"(setq MBB_TOTAL_YIELD {container_yield:.2f})\n")
        f.write(f"(setq MBB_PLACED_COUNT {len(verified_plan)})\n")
        f.write(f"(setq MBB_UNPLACED_COUNT {len(unplaced_parts)})\n")
        f.write('(setq MBB_NESTING_STATUS "SUCCESS")\n')
        f.write("(setq MBB_NESTING_PLAN '(\n")
        for item in verified_plan:
            rot_deg = float(item['rot'])
            off_x = float(item.get('off_x', 0.0))
            off_y = float(item.get('off_y', 0.0))
            f.write(f"  ({item['id']} {item['container_id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f} {rot_deg:.2f} {off_x:.4f} {off_y:.4f})\n")
        f.write("))\n")
        f.write("(setq MBB_UNPLACED_PLAN '(\n")
        for item in buffer_plan:
            f.write(f"  ({item['id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f})\n")
        f.write("))\n")
        f.write("(setq *mbb-nesting-result* MBB_NESTING_PLAN)\n")
        f.write(f"(setq *mbb-placed-count* {len(verified_plan)})\n")
        f.write(f"(setq *mbb-unplaced-count* {len(unplaced_parts)})\n")
        f.write("(princ)\n")

    print("\n>> 네스팅 연동 파일 생성 완료: mbb_nesting_out.lsp / mbb_nesting_out.json")


if __name__ == '__main__':
    main()
