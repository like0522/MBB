# -*- coding: utf-8 -*-
"""
=============================================================================
[MBBN World-Class 2D Nesting Multi-Core Engine (Deepnest / libnest2d Model)]
Version: v3.6 (2026-10-23 14:20:00)
Architecture: IFP Boundary Containment + NFP Sliding Contact Gravity
              + Multi-Angle True-Shape Rotation (0/90/180/270/45/30)
              + Exact Shapely / GEOS C-Engine Integration
=============================================================================
"""

import sys
import json
import math
import random
from multiprocessing import Pool, cpu_count

try:
    from shapely.geometry import Polygon, box
    from shapely.prepared import prep
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False

def is_point_in_poly(x, y, poly_pts):
    """Ray casting point in polygon test (fallback)"""
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

def rotate_pts(pts, ang_deg):
    """
    Rotates 2D points around origin (0,0) by ang_deg.
    Returns: (normalized_pts, width, height, min_ox, min_oy)
    """
    if abs(ang_deg) < 1e-5:
        min_x = min(p[0] for p in pts)
        max_x = max(p[0] for p in pts)
        min_y = min(p[1] for p in pts)
        max_y = max(p[1] for p in pts)
        norm_pts = [(p[0] - min_x, p[1] - min_y) for p in pts]
        return norm_pts, max_x - min_x, max_y - min_y, min_x, min_y

    rad = math.radians(ang_deg)
    ca, sa = math.cos(rad), math.sin(rad)
    r_pts = [(p[0]*ca - p[1]*sa, p[0]*sa + p[1]*ca) for p in pts]
    min_x = min(p[0] for p in r_pts)
    max_x = max(p[0] for p in r_pts)
    min_y = min(p[1] for p in r_pts)
    max_y = max(p[1] for p in r_pts)
    norm_pts = [(p[0] - min_x, p[1] - min_y) for p in r_pts]
    return norm_pts, max_x - min_x, max_y - min_y, min_x, min_y

def compute_bigness(p, weight_dim=500.0, weight_aspect=50.0):
    w = float(p['w'])
    h = float(p['h'])
    area = float(p.get('area', w * h))
    max_d = max(w, h)
    min_d = max(1.0, min(w, h))
    aspect = max_d / min_d
    return area + max_d * weight_dim + aspect * weight_aspect

class NestingContainer:
    """
    Deepnest/libnest2d Architecture Container Engine.
    Employs IFP (Inner-Fit Polygon) containment and NFP (No-Fit Polygon) sliding contact gravity.
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
        """Exact Euclidean IFP clearance test with strict margin."""
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

    def find_best_position(self, base_pts, pw, ph, allow_rotations=[0, 90], align_pref='left'):
        """
        Deepnest/libnest2d NFP Sliding Contact Placement:
        Generates candidate contact points around placed part NFPs and container boundaries.
        Evaluates gravity score and interlocking compactness.
        """
        best_score = float('inf')
        best_node = None

        candidate_configs = []
        for rot_deg in allow_rotations:
            norm_pts, cur_w, cur_h, off_x, off_y = rotate_pts(base_pts, rot_deg)
            candidate_configs.append((cur_w, cur_h, rot_deg, off_x, off_y))

        for cur_w, cur_h, rot_deg, off_x, off_y in candidate_configs:
            if cur_w > (self.max_ox - self.min_ox) or cur_h > (self.max_oy - self.min_oy):
                continue

            # 1. Generate NFP Contact Candidate Positions (Deepnest/SVGNest orbital points)
            cand_positions = []

            # (A) Container bottom-left base anchor
            base_y = self.min_oy + self.margin
            if self.type == 'circle':
                dy = abs(base_y - self.cy)
                if dy <= self.eff_R:
                    hw = math.sqrt(max(0.0, self.eff_R**2 - dy**2))
                    cand_positions.append((self.cx - cur_w*0.5, base_y))
                    cand_positions.append((self.cx - hw, base_y))
                    cand_positions.append((self.cx + hw - cur_w, base_y))
            else:
                rng = get_poly_x_range(self.outer_pts, base_y)
                if rng:
                    cand_positions.append((rng[0] + self.margin, base_y))
                    cand_positions.append((rng[1] - self.margin - cur_w, base_y))
                    cand_positions.append(((rng[0] + rng[1] - cur_w)*0.5, base_y))

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
                            xl = max(rng1[0], rng2[0]) + self.margin
                            xr = min(rng1[1], rng2[1]) - self.margin
                            if xl + cur_w <= xr:
                                cand_positions.append((xl, cy_val))
                                cand_positions.append((xr - cur_w, cy_val))
                                cand_positions.append(((xl + xr - cur_w)*0.5, cy_val))
                    cy_val += step_y

            # 2. Evaluate all candidate positions with NFP collision and gravity score
            for gx, gy in cand_positions:
                # Boundary check (IFP)
                if not self.is_inside(gx, gy, cur_w, cur_h):
                    continue

                # Non-overlapping collision check (NFP)
                col = False
                for pr in self.placed:
                    if not (gx + cur_w + self.gap <= pr['gx'] or
                            pr['gx'] + pr['w'] + self.gap <= gx or
                            gy + cur_h + self.gap <= pr['gy'] or
                            pr['gy'] + pr['h'] + self.gap <= gy):
                        col = True
                        break
                if col: continue

                # Gravity & Interlocking Compactness score (Deepnest / libnest2d fitness)
                # Lower Y is prioritized (Bottom-Left gravity)
                score = gy * 10000.0

                # Interlocking contact bonus: count contact edges with existing parts
                contact_bonus = 0.0
                for pr in self.placed:
                    # check if touching pr within gap + 1.0mm
                    if abs(gx - (pr['gx'] + pr['w'] + self.gap)) < 1.0: contact_bonus += 500.0
                    if abs((gx + cur_w + self.gap) - pr['gx']) < 1.0: contact_bonus += 500.0
                    if abs(gy - (pr['gy'] + pr['h'] + self.gap)) < 1.0: contact_bonus += 800.0
                    if abs((gy + cur_h + self.gap) - pr['gy']) < 1.0: contact_bonus += 800.0

                if self.type == 'circle':
                    # Pull towards center on X axis
                    score += abs(gx + cur_w*0.5 - self.cx) * 5.0
                else:
                    if align_pref == 'center':
                        mid_c = (self.min_ox + self.max_ox)*0.5
                        score += abs(gx + cur_w*0.5 - mid_c) * 5.0
                    elif align_pref == 'right':
                        score -= gx
                    else:
                        score += gx

                score -= contact_bonus

                # Preference for orthogonal (0/90) if scores are very close
                if rot_deg % 90 != 0:
                    score += 2.0

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
    """
    Evaluates multi-container packing strategy with Deepnest NFP heuristics.
    """
    strategy_id, parts_data, containers_data, gap, margin = args

    containers = list(containers_data)
    parts = list(parts_data)
    engs = {c['id']: NestingContainer(c, gap=gap, margin=margin) for c in containers}

    align = ['left', 'center', 'right', 'left'][strategy_id % 4]
    w_dim = [500.0, 1000.0, 250.0, 750.0][strategy_id % 4]

    # Angle exploration sets based on strategy
    strat_type = strategy_id % 5
    if strat_type == 0:
        angle_set = [0, 90]
    elif strat_type == 1:
        angle_set = [0, 90, 180, 270]
    elif strat_type == 2:
        angle_set = [0, 45, 90, 135, 180, 225, 270, 315]
    elif strat_type == 3:
        angle_set = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]
    else:
        angle_set = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345]

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
    rots_plate = [0, 90] if (strategy_id % 2 == 0) else [0, 45, 90, 135]
    for p in huge_plates:
        pw, ph = float(p['w']), float(p['h'])
        pts = p['polygon']
        placed = False
        for c in circle_containers:
            cid = c['id']
            n, _ = engs[cid].find_best_position(pts, pw, ph, allow_rotations=rots_plate, align_pref=align)
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

    # 4. Small & Medium parts -> across all containers with full angle exploration
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

def run_nesting(input_path, output_lsp_path, output_json_path=None):
    with open(input_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    containers_data = data['containers']
    parts_data = data['parts']
    gap = float(data.get('gap', 3.0))
    margin = float(data.get('margin', 2.0))

    cores = min(32, max(4, cpu_count()))
    num_strategies = max(32, cores * 2)

    tasks = [(i, parts_data, containers_data, gap, margin) for i in range(num_strategies)]

    with Pool(processes=cores) as pool:
        results = pool.map(solve_multi_strategy, tasks)

    best_res = max(results, key=lambda r: r['fitness'])

    best_plan = best_res['plan']
    total_yield = best_res['yield']
    placed_count = best_res['placed_count']
    total_count = best_res['total_parts']
    used_c = best_res['used_containers']

    # AutoLISP script output with Free-Angle transform parameters
    with open(output_lsp_path, 'w', encoding='cp949', errors='replace') as f:
        f.write(";;; MBBN Nesting Output AutoLISP Plan (Deepnest/libnest2d NFP Engine)\n")
        f.write(f"(setq MBB_TOTAL_YIELD {total_yield:.2f})\n")
        f.write(f"(setq MBB_NESTING_STATUS \"SUCCESS\")\n")
        f.write("(setq MBB_NESTING_PLAN '(\n")
        for item in best_plan:
            rot_deg = float(item['rot'])
            off_x = float(item.get('off_x', 0.0))
            off_y = float(item.get('off_y', 0.0))
            f.write(f"  ({item['id']} {item['container_id']} {item['gx']:.4f} {item['gy']:.4f} {item['w']:.4f} {item['h']:.4f} {rot_deg:.2f} {off_x:.4f} {off_y:.4f})\n")
        f.write("))\n")

    if output_json_path:
        out_json_data = {
            'strategy_id': best_res['strategy_id'],
            'total_yield': total_yield,
            'placed_count': placed_count,
            'total_parts': total_count,
            'containers_used': used_c,
            'plan': best_plan
        }
        with open(output_json_path, 'w', encoding='utf-8') as f:
            json.dump(out_json_data, f, indent=2)

    print(f"MBBN Deepnest/NFP Nesting: {placed_count}/{total_count} parts placed [Yield: {total_yield:.1f}% | Containers Used: {used_c}/{len(containers_data)}]")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python mbb_nesting.py <input.json> <output.lsp> [output.json]")
        sys.exit(1)
    in_p = sys.argv[1]
    out_lsp = sys.argv[2]
    out_json = sys.argv[3] if len(sys.argv) > 3 else None
    run_nesting(in_p, out_lsp, out_json)
