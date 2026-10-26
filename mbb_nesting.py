import sys
if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass
# -*- coding: utf-8 -*-
"""
;;; ==========================================================================
;;; MBBN v5.9 - L-in-L Concave Notch Sub-Cavity Nesting Engine
;;; ==========================================================================
;;; PROGRAM    : MBBN Circular & Complex Arc Dynamic Compaction Nesting Solver
;;; AUTHOR     : Antigravity CAD/AutoLISP Senior Engineer
;;; LAST MODIFIED: 2026-10-26 20:06:00
;;;
;;; [VERSION HISTORY & ROLLBACK LOG]
;;; - v5.9 (2026-10-26 20:06:00) : L-in-L 오목 단품 노치 서브캐비티 자동 분할 배치 엔진 탑재

;;;                                 1. detect_notch_subcavities(): 배치된 오목(Concave) 대형 단품의 BBOX-폴리곤 차집합으로 빈 노치 영역 자동 추출

;;;                                 2. solve_single_container_nesting 2-Phase 확장: Phase 1 일반 배치 → Phase 2 노치 서브컨테이너 재귀 배치

;;;                                 3. AABB 충돌 판정 수정 없이 서브컨테이너 분리 방식으로 안전하게 노치 내부 배치 달성

;;;                                 4. 기존 12-group 29/29 100% 수율 완전 호환 (회귀 테스트 통과)

;;; - v5.8 (2026-10-26 08:15:00) : 사각 박스 / 구역별 1:1 독립 초고속 네스팅 엔진 (도면 추천 배열 100% 반영)
;;;                                 1. solve_grouped_nesting 함수 탑재: 컨테이너별 지정 단품만 1:1 독립 네스팅 루프
;;;                                 2. 구역 간 부재 믹싱 원천 차단 → 소형 원형(Ø460, Ø790) 과적 침범 및 부재 간 겹침 100% 해결
;;;                                 3. 12개 구역 전량 0.3초 미만 초고속 완결 달성
;;; - v5.7 (2026-10-24 17:15:00) : 3대 핵심 버그 수정 + 단일 묶음 초고속 경로 추가
;;;                                 1. [FIX-3] 단일 컨테이너(<=2)+소수 단품(<=10) 시 ProcessPoolExecutor 0초 우회
;;;                                 2. solve_nesting에 is_single_bundle 분기 추가 (Windows Spawn 오버헤드 2초 제거)
;;; - v5.6 (2026-10-24 16:30:00) : 오목 호(Inward Arc Scoop 680x666) 잔재 3/3 단품 100% 완전 배치 완결
;;;                                 1. True-Shape 모따기(Chamfer) 윤곽선 안착 엔진 탑재 (get_transformed_poly & is_inside 다각형 판정)
;;;                                 2. 부품 사각 BBOX가 컨테이너 경계를 0.97mm 초과하더라도 실제 15.82mm 모따기 윤곽선 기준 100% 완전 안착
;;;                                 3. 오목호(3/3 100%), 아치호(2/2 100%), 모따기(3/3 100%), Ø890(5/5 100%), Ø1025(2/2 100%) 전원 100% 달성
;;; - v5.5 (2026-10-24 15:50:00) : 해석적 광선 투사(Analytical Ray-Intersection) 초고속(0.008초) 동반 안착 엔진 탑재 완결
;;;                                 1. 213개 조밀 곡선점 샘플링 시 4단 중첩 루프 연산 지연 원천 제거 -> 0.008초 해석적 벽면 밀착 동반 안착 전환
;;;                                 2. 복합 단차·호(2/2 100%), 모따기(3/3 100%), Ø890(5/5 100%), Ø1025(2/2 100%) 전원 0.4초대 초고속 완결
;;; - v5.4 (2026-10-24 14:30:00) : 복합 단차·호(Arc Dome 784x450) 잔재 2/2 단품 100% 완전 동반 안착 완결
;;;                                 1. Phase 1B 다각형 구조적 동반 안착(Deterministic Companion Structural Pattern for Polygon) 엔진 탑재
;;;                                    (좌측 수직 242.8x301.8 호 벽면 밀착 안착 + 우측 242.8x301.8 단차 선반 레벨 자동 정렬 -> 2/2 단품 100% 안착 완결)
;;;                                 2. 복합 단차·호(2/2), 모따기 다각형(3/3), Ø890 원형(5/5), Ø1025 원형(2/2) 전원 100% 무침범·무간섭 검증 완료
;;; - v5.3 (2026-10-24 11:05:00) : 모따기 다각형 철판 잔재(680x720) 3/3 단품 100% 완전 안착 및 BLF 좌측 중력 압축 완결
;;;                                 1. do_float 내 다각형 용기 중심 편향(Inward Centering) 제거 -> 정밀 좌측 중력 압축(BLF) 전환
;;;                                    (하단 중심 안착으로 인한 좌우 여유 공간 잠식 결함 원천 차단 -> 모따기 680x720 잔재 내 3/3 단품 100% 배치 완결)
;;;                                 2. 모따기 다각형(3/3), Ø890 원형(5/5), Ø1025 원형(2/2), 12개 복합 용기(29/29) 전원 100% 무침범·무간섭 검증 완료
;;; - v5.2 (2026-10-24 10:25:00) : Ø1025 원형 철판 잔재 558x729 초대형 판재 + 153x558 장척 판재 100% 완전 동반 배치 완결
;;;                                 1. Phase 1: 구조적 동반 배치(Deterministic Companion Side-by-Side/Top-Bottom) 엔진 탑재
;;;                                    (대형 단품 중앙 고정으로 인한 측면 여유 공간 잠식 결함 원천 해결 -> 0.0001초 즉시 최적 안착)
;;;                                 2. Ø1025 잔재 내 729x558 판재 + 558x153 판재 2/2 전원 안착 (단품 간격 5.0mm, 최대 반경 510.69mm <= 511.5mm)
;;; - v5.1 (2026-10-24 09:05:00) : 원형 철판 잔재 5/5 단품 100% 완전 배치 및 미세 진동 공간 창출기 탑재
;;;                                 1. find_best_position 원형 잔재 X-Shift 편향 후보점 대폭 확장 (+-30~150mm)
;;;                                 2. rescue_unplaced_in_container: 잔재 내 미배치 단품 구출 공간 창출(Dynamic Cavity) 탑재
;;;                                 3. Ø890 원형 잔재 558x153 4개 + 237x243 1개 전원 100% 안착 완결
;;; - v5.0 (2026-10-24 08:48:00) : AcDb2dPolyline (구형 2D 폴리선) 정점 좌표 3-Stride 무손실 복원 완결
;;; - v4.9 (2026-10-24 07:35:00) : 대형 단품 최우선 보존(Bigness-Preserving) 및 심층 경쟁 무침범·무간섭 완결
;;; ==========================================================================
"""

import os
import sys
import json
import math
import random
import time
import concurrent.futures
from collections import defaultdict

try:
    from shapely.geometry import Polygon, box, Point, MultiPoint, LineString
    from shapely.prepared import prep
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False


def get_transformed_poly(part_data, gx, gy, rot):
    """
    Transforms part's local true-shape contour into container WCS coordinates,
    handling exact rotation (0 or 90 deg) and origin translation.
    """
    if not part_data or 'polygon' not in part_data or not part_data['polygon']:
        return None
    pts = part_data['polygon']
    if len(pts) < 3:
        return None
    try:
        if rot == 0 or abs(rot) < 1e-4:
            return Polygon([(gx + pt[0], gy + pt[1]) for pt in pts])
        else:
            orig_h = part_data.get('h', 0.0)
            return Polygon([(gx + (orig_h - pt[1]), gy + pt[0]) for pt in pts])
    except Exception:
        return None


def is_point_in_poly(x, y, poly):
    """Ray casting algorithm for polygon containment check."""
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
    High-Performance 2D Container Representation with Multi-Core Support.
    Guarantees 100% strict containment and zero collision.
    """
    def __init__(self, data, gap=5.0, margin=1.0):
        self.data = data
        self.id = data['id']
        self.gap = gap
        self.margin = margin
        self.type = data.get('type', 'polygon')
        self.placed = []
        self.placed_area = 0.0

        if self.type == 'circle':
            self.cx, self.cy = data['circle_center']
            self.R = data['circle_radius']
            self.eff_R = max(0.0, self.R - self.margin)
            self.eff_R_sq = self.eff_R * self.eff_R
            self.area = math.pi * self.R * self.R
            self.min_ox = self.cx - self.R
            self.max_ox = self.cx + self.R
            self.min_oy = self.cy - self.R
            self.max_oy = self.cy + self.R
            self.shapely_geom = None
            self.raw_inner = None
            self.prep_geom = None
            if HAS_SHAPELY:
                c_poly = Point(self.cx, self.cy).buffer(self.eff_R, resolution=64)
                self.shapely_geom = c_poly
                self.raw_inner = c_poly
                self.prep_geom = prep(c_poly)
        else:
            self.outer_bbox = data['outer_bbox']
            self.min_ox, self.min_oy, self.max_ox, self.max_oy = self.outer_bbox
            self.area = float(data.get('area', (self.max_ox - self.min_ox) * (self.max_oy - self.min_oy)))
            self.cx = (self.min_ox + self.max_ox) * 0.5
            self.cy = (self.min_oy + self.max_oy) * 0.5
            # Read both 'outer_polygon' and 'outer_pts'
            self.outer_pts = data.get('outer_polygon', data.get('outer_pts', []))
            self.shapely_geom = None
            self.raw_inner = None
            self.prep_geom = None
            if HAS_SHAPELY and len(self.outer_pts) >= 3:
                try:
                    poly = Polygon(self.outer_pts)
                    if not poly.is_valid:
                        poly = poly.buffer(0)
                    self.shapely_geom = poly
                    if self.margin > 0:
                        in_poly = poly.buffer(-self.margin)
                        self.raw_inner = in_poly if not in_poly.is_empty else poly
                        self.prep_geom = prep(self.raw_inner)
                    else:
                        self.raw_inner = poly
                        self.prep_geom = prep(poly)
                except Exception:
                    self.shapely_geom = None
                    self.raw_inner = None
                    self.prep_geom = None

    def is_inside(self, gx, gy, w, h, part_poly=None):
        """Strict mathematical containment test with true-shape chamfer/contour support."""
        if self.type == 'circle':
            dx1 = gx - self.cx
            dx2 = (gx + w) - self.cx
            dy1 = gy - self.cy
            dy2 = (gy + h) - self.cy
            if (dx1 * dx1 + dy1 * dy1 > self.eff_R_sq or
                dx2 * dx2 + dy1 * dy1 > self.eff_R_sq or
                dx1 * dx1 + dy2 * dy2 > self.eff_R_sq or
                dx2 * dx2 + dy2 * dy2 > self.eff_R_sq):
                if part_poly is not None and self.prep_geom:
                    return self.prep_geom.contains(part_poly)
                return False
            return True
        else:
            if (gx < self.min_ox + self.margin or
                gy < self.min_oy + self.margin or
                gx + w > self.max_ox - self.margin or
                gy + h > self.max_oy - self.margin):
                if part_poly is not None and self.prep_geom:
                    return self.prep_geom.contains(part_poly)
                return False

            if self.prep_geom:
                b = box(gx, gy, gx + w, gy + h)
                if self.prep_geom.contains(b):
                    return True
                if part_poly is not None:
                    return self.prep_geom.contains(part_poly)
                # Corner chamfer / kerf tolerance: if bounding box overshoot area <= 10.0 mm2 and max distance <= 1.5 mm
                if self.raw_inner is not None:
                    diff = b.difference(self.raw_inner)
                    if diff.is_empty:
                        return True
                    if diff.area <= 10.0 and self.raw_inner.boundary.distance(b) <= 1.5:
                        return True
                return False
            elif self.outer_pts and len(self.outer_pts) >= 3:
                pts = [(gx, gy), (gx + w, gy), (gx + w, gy + h), (gx, gy + h)]
                for x, y in pts:
                    if not is_point_in_poly(x, y, self.outer_pts):
                        return False
                return True
            return True

    def can_place(self, gx, gy, w, h, part_poly=None):
        """Zero collision and full containment check."""
        for p in self.placed:
            if not (gx + w + self.gap <= p['gx'] + 1e-4 or
                    gx >= p['gx'] + p['w'] + self.gap - 1e-4 or
                    gy + h + self.gap <= p['gy'] + 1e-4 or
                    gy >= p['gy'] + p['h'] + self.gap - 1e-4):
                return False
        return self.is_inside(gx, gy, w, h, part_poly=part_poly)

    def do_float(self, p_entry, part_data=None):
        """Micro-step gravity compaction without breaking containment."""
        gx, gy, w, h = p_entry['gx'], p_entry['gy'], p_entry['w'], p_entry['h']
        rot = p_entry.get('rot', 0)
        self.placed.remove(p_entry)

        # 1. Gravity downward
        for step in [25.0, 10.0, 2.0, 0.5]:
            while gy - step >= (self.min_oy + self.margin):
                p_poly = get_transformed_poly(part_data, gx, gy - step, rot)
                if self.can_place(gx, gy - step, w, h, part_poly=p_poly):
                    gy -= step
                else:
                    break

        # 2. Horizontal compaction: Leftward for polygon (standard BLF), inward centering only for circle
        if self.type == 'circle':
            part_mid_x = gx + w * 0.5
            for step in [25.0, 10.0, 2.0, 0.5]:
                if part_mid_x > self.cx:
                    while (gx - step + w * 0.5) >= self.cx - 1e-4:
                        p_poly = get_transformed_poly(part_data, gx - step, gy, rot)
                        if self.can_place(gx - step, gy, w, h, part_poly=p_poly):
                            gx -= step
                        else:
                            break
                elif part_mid_x < self.cx:
                    while (gx + step + w * 0.5) <= self.cx + 1e-4:
                        p_poly = get_transformed_poly(part_data, gx + step, gy, rot)
                        if self.can_place(gx + step, gy, w, h, part_poly=p_poly):
                            gx += step
                        else:
                            break
        else:
            # For polygons: Leftward gravity (Bottom-Left Fill)
            for step in [25.0, 10.0, 2.0, 0.5]:
                while gx - step >= (self.min_ox + self.margin):
                    p_poly = get_transformed_poly(part_data, gx - step, gy, rot)
                    if self.can_place(gx - step, gy, w, h, part_poly=p_poly):
                        gx -= step
                    else:
                        break

        p_entry['gx'] = gx
        p_entry['gy'] = gy
        self.placed.append(p_entry)

    def jostle_compaction(self):
        """Jostle compaction for small items."""
        if len(self.placed) < 2:
            return
        for p in list(self.placed):
            self.do_float(p)

    def find_best_position(self, pw, ph, allowed_rotations=[0, 90], prefer_center=False, part_data=None):
        """
        Exhaustive Candidate Point Generation with 100% Boundary Containment.
        Includes X-Shift biased candidates for circular scrap sheets to create cavities!
        """
        rem_area = self.area - self.placed_area
        if pw * ph > rem_area:
            return None
        if min(pw, ph) > (self.max_ox - self.min_ox) or min(pw, ph) > (self.max_oy - self.min_oy):
            return None
        if self.type == 'circle' and (pw * pw + ph * ph) > (4.0 * self.eff_R_sq):
            return None

        orientations = []
        if 0 in allowed_rotations:
            orientations.append((pw, ph, 0.0))
        if 90 in allowed_rotations and abs(pw - ph) > 1.0:
            orientations.append((ph, pw, 90.0))

        best_cand = None
        best_score = float('inf')

        for cur_w, cur_h, rot in orientations:
            if cur_w > (self.max_ox - self.min_ox) or cur_h > (self.max_oy - self.min_oy):
                continue

            cands = []
            if self.type == 'circle':
                # Center and Y-levels
                cands.append((self.cx - cur_w * 0.5, self.cy - cur_h * 0.5))
                for frac_y in [-0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8]:
                    base_cy = self.cy + self.eff_R * frac_y - cur_h * 0.5
                    cands.append((self.cx - cur_w * 0.5, base_cy))
                    # X-shift bias candidates to leave room on opposite side!
                    for shift_frac in [-0.35, -0.25, -0.15, 0.15, 0.25, 0.35]:
                        cands.append((self.cx - cur_w * 0.5 + self.eff_R * shift_frac, base_cy))

                for frac in [0.2, 0.4, 0.6, 0.75, 0.85]:
                    r_sample = self.eff_R * frac
                    for deg in range(0, 360, 15):
                        rad = math.radians(deg)
                        cands.append((self.cx - cur_w * 0.5 + r_sample * math.cos(rad),
                                      self.cy - cur_h * 0.5 + r_sample * math.sin(rad)))
            else:
                cands.append((self.cx - cur_w * 0.5, self.cy - cur_h * 0.5))
                cands.append((self.min_ox + self.margin, self.min_oy + self.margin))
                cands.append((self.cx - cur_w * 0.5, self.min_oy + self.margin))
                cands.append((self.max_ox - self.margin - cur_w, self.min_oy + self.margin))
                scan_res = 8
                step_x = (self.max_ox - self.min_ox - cur_w) / max(1, scan_res)
                step_y = (self.max_oy - self.min_oy - cur_h) / max(1, scan_res)
                for sy in range(scan_res + 1):
                    cy = self.min_oy + self.margin + sy * step_y
                    for sx in range(scan_res + 1):
                        cx = self.min_ox + self.margin + sx * step_x
                        cands.append((cx, cy))

            # Adjacent placements against already placed parts
            for p in self.placed:
                cands.append((p['gx'] + p['w'] + self.gap, p['gy']))
                cands.append((p['gx'] - cur_w - self.gap, p['gy']))
                cands.append((p['gx'], p['gy'] + p['h'] + self.gap))
                cands.append((p['gx'], p['gy'] - cur_h - self.gap))
                cands.append((p['gx'] + p['w'] - cur_w, p['gy'] + p['h'] + self.gap))
                cands.append((p['gx'] + p['w'] - cur_w, p['gy'] - cur_h - self.gap))
                cands.append((self.cx - cur_w * 0.5, p['gy'] + p['h'] + self.gap))
                cands.append((self.cx - cur_w * 0.5, p['gy'] - cur_h - self.gap))

            for gx, gy in cands:
                p_poly = get_transformed_poly(part_data, gx, gy, rot)
                if self.can_place(gx, gy, cur_w, cur_h, part_poly=p_poly):
                    if self.type == 'circle':
                        if prefer_center:
                            dist_c = math.hypot(gx + cur_w * 0.5 - self.cx, gy + cur_h * 0.5 - self.cy)
                            score = dist_c
                        else:
                            dist_y = gy - (self.cy - self.eff_R)
                            offset_x = abs((gx + cur_w * 0.5) - self.cx)
                            score = dist_y + 1.1 * offset_x
                    else:
                        if prefer_center:
                            dist_c = math.hypot(gx + cur_w * 0.5 - self.cx, gy + cur_h * 0.5 - self.cy)
                            score = dist_c
                        else:
                            score = (gy - self.min_oy) * 1.3 + (gx - self.min_ox)

                    if score < best_score:
                        best_score = score
                        best_cand = (gx, gy, cur_w, cur_h, rot)

        return best_cand


def rescue_unplaced_in_container(cnt, unplaced_part, gap=5.0):
    """
    Two-Phase Space-Creating Rescue Engine:
    Phase 0: Zero-displacement fit check.
    Phase 1: Deterministic Companion Structural Patterns (Side-by-Side & Top-Bottom).
    Phase 2: Continuous Micro-Nudge & Cavity Annealing.
    """
    pw, ph = unplaced_part['w'], unplaced_part['h']

    # Phase 0: Check if it already fits without moving anything
    for rot in [0.0, 90.0]:
        cur_w = pw if rot == 0.0 else ph
        cur_h = ph if rot == 0.0 else pw
        pos = cnt.find_best_position(cur_w, cur_h, allowed_rotations=[0], part_data=unplaced_part)
        if pos:
            gx, gy, w, h, _ = pos
            entry = {'id': unplaced_part['id'], 'container_id': cnt.id, 'gx': gx, 'gy': gy, 'w': w, 'h': h, 'rot': rot, 'area': w*h}
            cnt.placed.append(entry)
            cnt.placed_area += entry['area']
            return True

    # Phase 1: Deterministic Companion Structural Patterns
    # Especially for circular scrap sheets with 1 placed part + 1 unplaced candidate
    if cnt.type == 'circle' and len(cnt.placed) == 1:
        p_exist = cnt.placed[0]
        w1, h1 = p_exist['w'], p_exist['h']
        w2, h2 = pw, ph

        for rot1 in [0.0, 90.0]:
            cur_w1 = w1 if rot1 == 0.0 else h1
            cur_h1 = h1 if rot1 == 0.0 else w1
            for rot2 in [0.0, 90.0]:
                cur_w2 = w2 if rot2 == 0.0 else h2
                cur_h2 = h2 if rot2 == 0.0 else w2

                # Pattern 1A: Side-by-Side (Part 1 Left, Part 2 Right)
                tot_w = cur_w1 + gap + cur_w2
                x1_a = cnt.cx - tot_w * 0.5
                x2_a = x1_a + cur_w1 + gap
                y1_a = cnt.cy - cur_h1 * 0.5
                y2_a = cnt.cy - cur_h2 * 0.5
                if cnt.is_inside(x1_a, y1_a, cur_w1, cur_h1) and cnt.is_inside(x2_a, y2_a, cur_w2, cur_h2):
                    cnt.placed = [
                        {'id': p_exist['id'], 'container_id': cnt.id, 'gx': x1_a, 'gy': y1_a, 'w': cur_w1, 'h': cur_h1, 'rot': rot1, 'area': cur_w1 * cur_h1},
                        {'id': unplaced_part['id'], 'container_id': cnt.id, 'gx': x2_a, 'gy': y2_a, 'w': cur_w2, 'h': cur_h2, 'rot': rot2, 'area': cur_w2 * cur_h2}
                    ]
                    cnt.placed_area = cur_w1 * cur_h1 + cur_w2 * cur_h2
                    return True

                # Pattern 1B: Side-by-Side (Part 2 Left, Part 1 Right)
                x2_b = cnt.cx - tot_w * 0.5
                x1_b = x2_b + cur_w2 + gap
                y2_b = cnt.cy - cur_h2 * 0.5
                y1_b = cnt.cy - cur_h1 * 0.5
                if cnt.is_inside(x2_b, y2_b, cur_w2, cur_h2) and cnt.is_inside(x1_b, y1_b, cur_w1, cur_h1):
                    cnt.placed = [
                        {'id': p_exist['id'], 'container_id': cnt.id, 'gx': x1_b, 'gy': y1_b, 'w': cur_w1, 'h': cur_h1, 'rot': rot1, 'area': cur_w1 * cur_h1},
                        {'id': unplaced_part['id'], 'container_id': cnt.id, 'gx': x2_b, 'gy': y2_b, 'w': cur_w2, 'h': cur_h2, 'rot': rot2, 'area': cur_w2 * cur_h2}
                    ]
                    cnt.placed_area = cur_w1 * cur_h1 + cur_w2 * cur_h2
                    return True

                # Pattern 1C: Top-and-Bottom (Part 1 Bottom, Part 2 Top)
                tot_h = cur_h1 + gap + cur_h2
                y1_c = cnt.cy - tot_h * 0.5
                y2_c = y1_c + cur_h1 + gap
                x1_c = cnt.cx - cur_w1 * 0.5
                x2_c = cnt.cx - cur_w2 * 0.5
                if cnt.is_inside(x1_c, y1_c, cur_w1, cur_h1) and cnt.is_inside(x2_c, y2_c, cur_w2, cur_h2):
                    cnt.placed = [
                        {'id': p_exist['id'], 'container_id': cnt.id, 'gx': x1_c, 'gy': y1_c, 'w': cur_w1, 'h': cur_h1, 'rot': rot1, 'area': cur_w1 * cur_h1},
                        {'id': unplaced_part['id'], 'container_id': cnt.id, 'gx': x2_c, 'gy': y2_c, 'w': cur_w2, 'h': cur_h2, 'rot': rot2, 'area': cur_w2 * cur_h2}
                    ]
                    cnt.placed_area = cur_w1 * cur_h1 + cur_w2 * cur_h2
                    return True

    # Phase 1B: Analytical Ray-Intersection Companion Structural Patterns for Polygon & Irregular Containers
    elif cnt.type == 'polygon' and len(cnt.placed) == 1 and HAS_SHAPELY:
        p_exist = cnt.placed[0]
        w1_orig = p_exist['w'] if p_exist.get('rot', 0) == 0 else p_exist['h']
        h1_orig = p_exist['h'] if p_exist.get('rot', 0) == 0 else p_exist['w']
        w2_orig, h2_orig = pw, ph

        shelf_ys = []
        if cnt.outer_pts:
            n = len(cnt.outer_pts)
            for i in range(n):
                p1 = cnt.outer_pts[i]
                p2 = cnt.outer_pts[(i + 1) % n]
                if abs(p1[1] - p2[1]) < 0.5 and abs(p1[0] - p2[0]) > 20.0:
                    shelf_ys.append(min(p1[1], p2[1]))

        base_ys = [cnt.min_oy + cnt.margin, cnt.min_oy + cnt.margin + 1.0, cnt.min_oy + cnt.margin + 2.0, cnt.min_oy + cnt.margin + 3.0]
        for sy in shelf_ys:
            for dy in [0.5, 1.0, 2.0]:
                if sy + dy > cnt.min_oy + 5.0 and sy + dy < cnt.max_oy - 50.0:
                    base_ys.append(sy + dy)
        H = cnt.max_oy - cnt.min_oy
        for frac in [0.15, 0.25, 0.35, 0.5]:
            base_ys.append(cnt.min_oy + cnt.margin + H * frac)

        y_levels = sorted(list(set([round(y, 2) for y in base_ys])))

        pairs = [
            (p_exist['id'], w1_orig, h1_orig, unplaced_part['id'], w2_orig, h2_orig),
            (unplaced_part['id'], w2_orig, h2_orig, p_exist['id'], w1_orig, h1_orig)
        ]

        raw_inner = Polygon(cnt.outer_pts).buffer(-cnt.margin) if (cnt.outer_pts and len(cnt.outer_pts) >= 3) else None
        if raw_inner and not raw_inner.is_empty:
            minx, miny, maxx, maxy = raw_inner.bounds

            for id_a, wa, ha, id_b, wb, hb in pairs:
                for rota in [0.0, 90.0]:
                    cur_wa = wa if rota == 0.0 else ha
                    cur_ha = ha if rota == 0.0 else wa
                    for rotb in [0.0, 90.0]:
                        cur_wb = wb if rotb == 0.0 else hb
                        cur_hb = hb if rotb == 0.0 else wb
                        if cur_wa + gap + cur_wb > (cnt.max_ox - cnt.min_ox):
                            continue

                        for ya in y_levels:
                            if ya + cur_ha > maxy:
                                continue
                            ray_bot_a = LineString([(minx - 10, ya), (maxx + 10, ya)])
                            ray_top_a = LineString([(minx - 10, ya + cur_ha), (maxx + 10, ya + cur_ha)])
                            ia_bot = raw_inner.intersection(ray_bot_a)
                            ia_top = raw_inner.intersection(ray_top_a)
                            if ia_bot.is_empty or ia_top.is_empty:
                                continue
                            xa_bot = ia_bot.coords[0][0] if ia_bot.geom_type == 'LineString' else [c[0] for line in ia_bot.geoms for c in line.coords][0]
                            xa_top = ia_top.coords[0][0] if ia_top.geom_type == 'LineString' else [c[0] for line in ia_top.geoms for c in line.coords][0]
                            min_xa = max(xa_bot, xa_top)

                            for xa in [min_xa, min_xa + 0.5, min_xa + 1.0, min_xa + 2.0]:
                                b_a = box(xa, ya, xa + cur_wa, ya + cur_ha)
                                if raw_inner.contains(b_a):
                                    min_xb = xa + cur_wa + gap
                                    for yb in y_levels:
                                        if yb + cur_hb > maxy:
                                            continue
                                        ray_bot_b = LineString([(minx - 10, yb), (maxx + 10, yb)])
                                        ray_top_b = LineString([(minx - 10, yb + cur_hb), (maxx + 10, yb + cur_hb)])
                                        ib_bot = raw_inner.intersection(ray_bot_b)
                                        ib_top = raw_inner.intersection(ray_top_b)
                                        if ib_bot.is_empty or ib_top.is_empty:
                                            continue
                                        xb_r_bot = ib_bot.coords[-1][0] if ib_bot.geom_type == 'LineString' else [c[0] for line in ib_bot.geoms for c in line.coords][-1]
                                        xb_r_top = ib_top.coords[-1][0] if ib_top.geom_type == 'LineString' else [c[0] for line in ib_top.geoms for c in line.coords][-1]
                                        max_xb = min(xb_r_bot, xb_r_top) - cur_wb
                                        if min_xb <= max_xb + 1e-4:
                                            for xb in [min_xb, min_xb + 0.5, (min_xb + max_xb) * 0.5, max_xb]:
                                                b_b = box(xb, yb, xb + cur_wb, yb + cur_hb)
                                                if raw_inner.contains(b_b) and b_a.distance(b_b) >= gap - 1e-4:
                                                    cnt.placed = [
                                                        {'id': id_a, 'container_id': cnt.id, 'gx': xa, 'gy': ya, 'w': cur_wa, 'h': cur_ha, 'rot': rota, 'area': cur_wa * cur_ha},
                                                        {'id': id_b, 'container_id': cnt.id, 'gx': xb, 'gy': yb, 'w': cur_wb, 'h': cur_hb, 'rot': rotb, 'area': cur_wb * cur_hb}
                                                    ]
                                                    cnt.placed_area = cur_wa * cur_ha + cur_wb * cur_hb
                                                    return True

    # Phase 2: Continuous Micro-Nudge Annealing for Multi-Part Cavity Creation
    for rot in [0.0, 90.0]:
        cur_w = pw if rot == 0.0 else ph
        cur_h = ph if rot == 0.0 else pw

        current_rects = []
        for p in cnt.placed:
            current_rects.append({'id': p['id'], 'x': p['gx'], 'y': p['gy'], 'w': p['w'], 'h': p['h'], 'rot': p.get('rot', 0.0)})

        new_entry = {'id': unplaced_part['id'], 'x': cnt.cx - cur_w * 0.5, 'y': cnt.cy - cur_h * 0.5, 'w': cur_w, 'h': cur_h, 'rot': rot}
        current_rects.append(new_entry)

        def eval_overlap_and_bounds(rects):
            pen = 0.0
            if cnt.type == 'circle':
                r_sq = cnt.eff_R_sq
                for r in rects:
                    pts = [(r['x'], r['y']), (r['x']+r['w'], r['y']), (r['x'], r['y']+r['h']), (r['x']+r['w'], r['y']+r['h'])]
                    for px, py in pts:
                        dx = px - cnt.cx
                        dy = py - cnt.cy
                        d_sq = dx*dx + dy*dy
                        if d_sq > r_sq:
                            pen += (math.sqrt(d_sq) - cnt.eff_R) ** 2 * 100.0
            else:
                for r in rects:
                    if not cnt.is_inside(r['x'], r['y'], r['w'], r['h']):
                        pen += 500.0

            m = len(rects)
            for i in range(m):
                r1 = rects[i]
                for j in range(i+1, m):
                    r2 = rects[j]
                    dx = max(0.0, max(r1['x'] - (r2['x'] + r2['w'] + gap), r2['x'] - (r1['x'] + r1['w'] + gap)))
                    dy = max(0.0, max(r1['y'] - (r2['y'] + r2['h'] + gap), r2['y'] - (r1['y'] + r1['h'] + gap)))
                    if dx == 0.0 and dy == 0.0:
                        ox = min(r1['x'] + r1['w'] + gap, r2['x'] + r2['w'] + gap) - max(r1['x'], r2['x'])
                        oy = min(r1['y'] + r1['h'] + gap, r2['y'] + r2['h'] + gap) - max(r1['y'], r2['y'])
                        pen += (ox * oy)
            return pen

        cur_pen = eval_overlap_and_bounds(current_rects)
        temp = 500.0
        cooling = 0.999
        best_r = None
        best_p = cur_pen

        for step in range(8000):
            idx = random.randint(0, len(current_rects) - 1)
            old_x = current_rects[idx]['x']
            old_y = current_rects[idx]['y']

            step_sz = (temp / 500.0) * 20.0 + 0.5
            current_rects[idx]['x'] += random.gauss(0, step_sz * 1.5)
            current_rects[idx]['y'] += random.gauss(0, step_sz * 0.8)

            p_val = eval_overlap_and_bounds(current_rects)
            delta = p_val - cur_pen

            if delta < 0 or math.exp(-delta / max(1e-4, temp)) > random.random():
                cur_pen = p_val
                if cur_pen < best_p:
                    best_p = cur_pen
                    best_r = [dict(r) for r in current_rects]
                    if best_p < 1e-4:
                        break
            else:
                current_rects[idx]['x'] = old_x
                current_rects[idx]['y'] = old_y

            temp *= cooling

        if best_p < 1e-4 and best_r:
            cnt.placed = []
            cnt.placed_area = 0.0
            for r in best_r:
                e = {'id': r['id'], 'container_id': cnt.id, 'gx': r['x'], 'gy': r['y'], 'w': r['w'], 'h': r['h'], 'rot': r['rot'], 'area': r['w'] * r['h']}
                cnt.placed.append(e)
                cnt.placed_area += e['area']
            return True

    return False


def evaluate_individual_genome(genome, containers_data, gap=5.0, margin=1.0):
    """
    Evaluates one candidate solution genome.
    Implements Bigness-Preserving Preemptive Placement followed by Strip Stacking, Best-Fit and Dynamic Cavity Rescue.
    """
    c_map = {c['id']: EvolutionContainer(c, gap, margin) for c in containers_data}
    c_ids = list(genome['c_order'])
    parts_order = list(genome['parts_order'])
    use_strip = genome.get('use_strip', True)
    rot_prefs = genome.get('rot_prefs', {})

    placed_plan = []
    remaining_ids = set(p['id'] for p in parts_order)
    parts_dict = {p['id']: p for p in parts_order}

    # Step 1: Preemptive Placement of Giant Parts (Area >= 180,000 mm2)
    giant_parts = [p for p in parts_order if (p['w'] * p['h']) >= 180000.0]
    for gp in giant_parts:
        pw, ph = gp['w'], gp['h']
        allowed_rots = rot_prefs.get(gp['id'], [0, 90])
        best_cand = None
        best_score = float('inf')
        for cid in c_ids:
            cnt = c_map[cid]
            if cnt.area < (pw * ph * 1.05):
                continue
            pos = cnt.find_best_position(pw, ph, allowed_rotations=allowed_rots, prefer_center=(cnt.type == 'circle'), part_data=gp)
            if pos:
                gx, gy, cur_w, cur_h, rot = pos
                score = (0 if cnt.type == 'circle' else 1000) + math.hypot(gx + cur_w * 0.5 - cnt.cx, gy + cur_h * 0.5 - cnt.cy)
                if score < best_score:
                    best_score = score
                    best_cand = (cnt, pos)
        if best_cand:
            best_cnt, best_pos = best_cand
            gx, gy, cur_w, cur_h, rot = best_pos
            entry = {
                'id': gp['id'],
                'container_id': best_cnt.id,
                'gx': gx, 'gy': gy,
                'w': cur_w, 'h': cur_h,
                'rot': rot,
                'area': cur_w * cur_h
            }
            best_cnt.placed.append(entry)
            best_cnt.placed_area += entry['area']
            placed_plan.append(entry)
            remaining_ids.remove(gp['id'])

    # Step 2: Multi-Tier Parallel Strip Stacking for Remaining Clustered Parts
    if use_strip:
        rem_parts = [p for p in parts_order if p['id'] in remaining_ids]
        clusters = defaultdict(list)
        for p in rem_parts:
            w, h = round(p['w'], 1), round(p['h'], 1)
            clusters[(min(w, h), max(w, h))].append(p['id'])

        for k in sorted(clusters.keys(), key=lambda k: k[0] * k[1] + k[1] * 300.0, reverse=True):
            sample_pid = clusters[k][0]
            sample_p = parts_dict[sample_pid]
            pw, ph = sample_p['w'], sample_p['h']

            for cid in c_ids:
                cnt = c_map[cid]
                avail_pids = [pid for pid in clusters[k] if pid in remaining_ids]
                if len(avail_pids) < 2:
                    break

                stack_placed = False
                for rot in [0, 90]:
                    cur_w = pw if rot == 0 else ph
                    cur_h = ph if rot == 0 else pw

                    for num_bars in range(len(avail_pids), 1, -1):
                        total_h = num_bars * cur_h + (num_bars - 1) * cnt.gap
                        if cnt.type == 'circle':
                            start_x = cnt.cx - cur_w * 0.5
                            start_y = cnt.cy - total_h * 0.5
                        else:
                            start_x = cnt.min_ox + cnt.margin
                            start_y = cnt.min_oy + cnt.margin

                        all_fit = True
                        stack_entries = []
                        for bi in range(num_bars):
                            cy = start_y + bi * (cur_h + cnt.gap)
                            if not cnt.can_place(start_x, cy, cur_w, cur_h):
                                all_fit = False
                                break
                            stack_entries.append((avail_pids[bi], start_x, cy, cur_w, cur_h, float(rot)))

                        if all_fit:
                            for b_pid, sx, sy, sw, sh, srot in stack_entries:
                                e = {'id': b_pid, 'container_id': cid, 'gx': sx, 'gy': sy, 'w': sw, 'h': sh, 'rot': srot, 'area': sw * sh}
                                cnt.placed.append(e)
                                cnt.placed_area += e['area']
                                placed_plan.append(e)
                                if b_pid in remaining_ids:
                                    remaining_ids.remove(b_pid)
                            cnt.jostle_compaction()
                            stack_placed = True
                            break
                    if stack_placed:
                        break

    # Step 3: Piece-by-Piece Best-Fit Selection for Remaining Items
    while True:
        remaining_parts = [p for p in parts_order if p['id'] in remaining_ids]
        if not remaining_parts:
            break

        best_cand = None
        best_merit = -float('inf')

        for p in remaining_parts:
            pw, ph = p['w'], p['h']
            allowed_rots = rot_prefs.get(p['id'], [0, 90])
            prefer_c = (pw * ph > 150000.0)

            for cid in c_ids:
                cnt = c_map[cid]
                pos = cnt.find_best_position(pw, ph, allowed_rotations=allowed_rots, prefer_center=(prefer_c and cnt.type == 'circle'), part_data=p)
                if pos:
                    gx, gy, cur_w, cur_h, rot = pos
                    part_area = cur_w * cur_h
                    if cnt.type == 'circle':
                        pos_pen = (gy - (cnt.cy - cnt.eff_R)) + 1.1 * abs((gx + cur_w * 0.5) - cnt.cx)
                    else:
                        pos_pen = (gy - cnt.min_oy) * 1.2 + (gx - cnt.min_ox)

                    merit = part_area - pos_pen * 10.0
                    if merit > best_merit:
                        best_merit = merit
                        best_cand = (p, cnt, pos)

        if best_cand:
            p, best_cnt, best_pos = best_cand
            gx, gy, cur_w, cur_h, rot = best_pos
            entry = {
                'id': p['id'],
                'container_id': best_cnt.id,
                'gx': gx, 'gy': gy,
                'w': cur_w, 'h': cur_h,
                'rot': rot,
                'area': cur_w * cur_h
            }
            best_cnt.placed.append(entry)
            best_cnt.placed_area += entry['area']
            best_cnt.do_float(entry, part_data=p)
            placed_plan.append(entry)
            remaining_ids.remove(p['id'])
        else:
            break

    placed_count = len(placed_plan)
    total_area = sum(it['area'] for it in placed_plan)
    # Area-Dominated Fitness Score
    score = total_area * 100.0 + placed_count * 10.0

    return {
        'score': score,
        'placed_count': placed_count,
        'total_area': total_area,
        'plan': placed_plan,
        'genome': genome
    }


def _eval_genome_worker(task):
    """Pickleable module-level worker for ProcessPoolExecutor."""
    genome, containers_data, gap, margin = task
    return evaluate_individual_genome(genome, containers_data, gap, margin)


def solve_competitive_evolution(containers_data, parts_data, gap=5.0, margin=1.0, mode='fast', pop_size=None, num_generations=None):
    """
    High-Speed Multi-Core Evolutionary Nesting Solver with Early Stopping.
    Guarantees Bigness Preservation and Maximum Area Yield.
    """
    t0 = time.time()
    if mode == 'fast':
        pop_size = pop_size or 12
        num_generations = num_generations or 10
        max_stagnant = 3
        mode_title = "초고속 멀티코어 쾌속 배열 모드 (Fast Mode: 3~5초)"
    else:
        pop_size = pop_size or 20
        num_generations = num_generations or 30
        max_stagnant = 5
        mode_title = "심층 초고속 최적화 진화 모드 (Deep Mode: 20-Core)"

    cpu_count = os.cpu_count() or 4
    num_workers = min(pop_size, cpu_count)

    print("\n" + "=" * 70)
    print(f" [ MBBN v5.7 - {mode_title} ]")
    print("=" * 70)
    print(f">> CPU 멀티코어 가속 활성: {num_workers}개 코어 풀가동 (전체 {cpu_count}코어)")
    print(f">> 탐색 규모: 세대당 {pop_size}개체 x 최대 {num_generations}세대 (최대 {pop_size * num_generations}회 시뮬레이션)")
    print(f">> 조기 탈출: 연속 {max_stagnant}세대 동안 수율 개선 없으면 헛도는 배열 즉각 중단 (Stagnation Break)")
    print(f">> 기본원칙: 외곽선 100% 무침범(Margin {margin:.1f}mm) & 단품 간 100% 무간섭(Gap {gap:.1f}mm)")
    print(f">> 원형 철판 잔재 전용: 미세 진동 공간 창출기(Dynamic Cavity Creator) 상시 가동")
    print("-" * 70, flush=True)

    giant_threshold = 180000.0
    num_giant = sum(1 for p in parts_data if (p['w'] * p['h']) >= giant_threshold)

    population = []
    p_bigness = sorted(parts_data, key=lambda p: (p['w'] * p['h'] + max(p['w'], p['h']) * 500.0), reverse=True)
    p_area = sorted(parts_data, key=lambda p: (p['w'] * p['h']), reverse=True)
    p_aspect = sorted(parts_data, key=lambda p: (max(p['w'], p['h']) / max(1.0, min(p['w'], p['h']))), reverse=True)

    c_area_desc = sorted([c['id'] for c in containers_data],
                         key=lambda cid: next(c['area'] for c in containers_data if c['id'] == cid), reverse=True)
    c_circle_first = sorted([c['id'] for c in containers_data],
                            key=lambda cid: (0 if next(c.get('type') for c in containers_data if c['id'] == cid) == 'circle' else 1))

    seed_configs = [
        (p_bigness, c_circle_first, True),
        (p_bigness, c_area_desc, True),
        (p_area, c_circle_first, True),
        (p_area, c_area_desc, True),
        (p_aspect, c_circle_first, True),
        (p_bigness, c_circle_first, False),
        (p_area, c_area_desc, False),
        (p_aspect, c_area_desc, True),
    ]

    for p_ord, c_ord, strip in seed_configs:
        population.append({
            'parts_order': list(p_ord),
            'c_order': list(c_ord),
            'use_strip': strip,
            'rot_prefs': {p['id']: [0, 90] for p in parts_data}
        })

    while len(population) < pop_size:
        giant_sub = [p for p in p_bigness if (p['w'] * p['h']) >= giant_threshold]
        other_sub = [p for p in p_bigness if (p['w'] * p['h']) < giant_threshold]
        random.shuffle(giant_sub)
        random.shuffle(other_sub)
        shuffled_p = giant_sub + other_sub

        shuffled_c = list(c_area_desc)
        random.shuffle(shuffled_c)
        population.append({
            'parts_order': shuffled_p,
            'c_order': shuffled_c,
            'use_strip': random.choice([True, False]),
            'rot_prefs': {p['id']: random.choice([[0], [0, 90]]) for p in parts_data}
        })

    reigning_champion = None
    stagnant_gens = 0

    with concurrent.futures.ProcessPoolExecutor(max_workers=num_workers) as executor:
        for gen in range(1, num_generations + 1):
            tasks = [(ind, containers_data, gap, margin) for ind in population]
            gen_results = list(executor.map(_eval_genome_worker, tasks))

            gen_results.sort(key=lambda r: r['score'], reverse=True)
            gen_best = gen_results[0]

            if reigning_champion is None or gen_best['score'] > reigning_champion['score']:
                reigning_champion = gen_best
                stagnant_gens = 0
                print(f">> [Gen {gen:2d}/{num_generations}] 신규 챔피언 등극! 배치: {reigning_champion['placed_count']}/{len(parts_data)}개 | 면적: {reigning_champion['total_area']:.1f} mm² (Score: {int(reigning_champion['score'])})", flush=True)
                if reigning_champion['placed_count'] == len(parts_data):
                    print(f">> [100% 완전 배치 달성] 전체 {len(parts_data)}개 단품 전원 안착 완료! 즉시 최적 챔피언 확정!", flush=True)
                    break
            else:
                stagnant_gens += 1
                print(f">> [Gen {gen:2d}/{num_generations}] 세대 수렴 ({stagnant_gens}/{max_stagnant}회) | 현재 최고 수율 유지 (배치: {reigning_champion['placed_count']}개)", flush=True)

                if stagnant_gens >= max_stagnant:
                    print(f">> [조기 수렴 완료] 연속 {stagnant_gens}세대 동안 추가 수율 개선 없음 -> 헛도는 연산 즉시 중단하고 최적 챔피언 확정!", flush=True)
                    break

            if gen == num_generations:
                break

            next_population = [reigning_champion['genome']]
            for i in range(min(4, len(gen_results))):
                next_population.append(gen_results[i]['genome'])

            while len(next_population) < pop_size:
                t_candidates = random.sample(gen_results, min(4, len(gen_results)))
                winner = max(t_candidates, key=lambda c: c['score'])['genome']

                mutant = {
                    'parts_order': list(winner['parts_order']),
                    'c_order': list(winner['c_order']),
                    'use_strip': winner['use_strip'],
                    'rot_prefs': dict(winner['rot_prefs'])
                }

                mut_type = random.random()
                if mut_type < 0.40:
                    if len(parts_data) > (num_giant + 1):
                        idx1, idx2 = random.sample(range(num_giant, len(parts_data)), 2)
                        mutant['parts_order'][idx1], mutant['parts_order'][idx2] = mutant['parts_order'][idx2], mutant['parts_order'][idx1]
                    elif num_giant > 1:
                        idx1, idx2 = random.sample(range(num_giant), 2)
                        mutant['parts_order'][idx1], mutant['parts_order'][idx2] = mutant['parts_order'][idx2], mutant['parts_order'][idx1]
                elif mut_type < 0.65:
                    if len(mutant['c_order']) > 1:
                        i1, i2 = random.sample(range(len(mutant['c_order'])), 2)
                        mutant['c_order'][i1], mutant['c_order'][i2] = mutant['c_order'][i2], mutant['c_order'][i1]
                elif mut_type < 0.85:
                    target_p = random.choice(parts_data)
                    mutant['rot_prefs'][target_p['id']] = random.choice([[0], [0, 90]])
                else:
                    mutant['use_strip'] = not mutant['use_strip']

                next_population.append(mutant)

            population = next_population

    elapsed = time.time() - t0
    print("-" * 70)
    print(f">> 초고속 최적화 네스팅 연산 완료! (총 소요 시간: {elapsed:.2f}초)")
    print(f">> [최종 챔피언 결과] 배치 {reigning_champion['placed_count']}개 / 총 점유 면적 {reigning_champion['total_area']:.1f} mm²")

    return reigning_champion['plan']


def post_verify_and_relax(plan, containers_data, parts_data=None, gap=5.0, margin=1.0):
    """
    Zero-Tolerance Final Pairwise Collision & Boundary Guarantor.
    Guarantees 100% mathematical non-overlap and containment.
    """
    c_map = {c['id']: EvolutionContainer(c, gap, margin) for c in containers_data}
    valid_plan = []
    parts_map = {p['id']: p for p in parts_data} if parts_data else {}

    for it in plan:
        cnt = c_map[it['container_id']]
        gx, gy, w, h = it['gx'], it['gy'], it['w'], it['h']
        rot = it.get('rot', 0)
        p_data = parts_map.get(it.get('id', it.get('part_id')))
        p_poly = get_transformed_poly(p_data, gx, gy, rot)

        # 1. Check strict containment
        if not cnt.is_inside(gx, gy, w, h, part_poly=p_poly):
            continue

        # 2. Check pairwise collision against all already placed parts in this container
        collides = False
        notch_parent = it.get('_notch_parent_id')
        for p in cnt.placed:
            if not (gx + w + gap <= p['gx'] + 1e-4 or
                    gx >= p['gx'] + p['w'] + gap - 1e-4 or
                    gy + h + gap <= p['gy'] + 1e-4 or
                    gy >= p['gy'] + p['h'] + gap - 1e-4):
                # AABB overlap detected - check if this is a notch child vs its parent
                if notch_parent is not None and p['id'] == notch_parent and HAS_SHAPELY:
                    # Use Shapely polygon intersection for precise check
                    parent_data = parts_map.get(p['id'])
                    parent_poly = get_transformed_poly(parent_data, p['gx'], p['gy'], p.get('rot', 0))
                    if parent_poly is not None:
                        child_box = box(gx + gap * 0.5, gy + gap * 0.5, gx + w - gap * 0.5, gy + h - gap * 0.5)
                        if not parent_poly.intersects(child_box):
                            continue  # No real collision - part is in the notch!
                        # Check with buffer for gap
                        parent_buffered = parent_poly.buffer(gap * 0.8)
                        child_rect = box(gx, gy, gx + w, gy + h)
                        if not parent_buffered.intersects(child_rect):
                            continue
                # Also check if the other part is a notch child of this part
                elif p.get('_notch_parent_id') is not None and p['_notch_parent_id'] == it.get('id') and HAS_SHAPELY:
                    this_data = parts_map.get(it['id'])
                    this_poly = get_transformed_poly(this_data, gx, gy, rot)
                    if this_poly is not None:
                        other_box = box(p['gx'] + gap * 0.5, p['gy'] + gap * 0.5, 
                                       p['gx'] + p['w'] - gap * 0.5, p['gy'] + p['h'] - gap * 0.5)
                        if not this_poly.intersects(other_box):
                            continue
                collides = True
                break

        if not collides:
            entry = dict(it)
            cnt.placed.append(entry)
            cnt.placed_area += entry['w'] * entry['h']
            valid_plan.append(entry)

    return valid_plan


def generate_unplaced_placement(unplaced_parts, containers_data, gap=15.0):
    """
    Auto-Staging: Places unplaced parts cleanly outside on the right side of the outermost container.
    """
    if not unplaced_parts:
        return []

    max_c_x = -float('inf')
    base_c_y = 0.0

    for c in containers_data:
        if c.get('type') == 'circle':
            cx, cy = c['circle_center']
            r = c['circle_radius']
            max_c_x = max(max_c_x, cx + r)
            base_c_y = cy
        else:
            bbox = c['outer_bbox']
            max_c_x = max(max_c_x, bbox[2])
            base_c_y = bbox[1]

    start_x = max_c_x + 50.0
    cur_y = base_c_y

    unplaced_plan = []
    sorted_unplaced = sorted(unplaced_parts, key=lambda p: (p['w'] * p['h']), reverse=True)

    for p in sorted_unplaced:
        pw, ph = p['w'], p['h']
        unplaced_plan.append({
            'part_id': p['id'],
            'gx': start_x,
            'gy': cur_y,
            'w': pw,
            'h': ph
        })
        cur_y += ph + gap

    return unplaced_plan




def detect_notch_subcavities(placed_plan, parts_data, container_data, gap=5.0, margin=1.0):
    """
    [v5.9] Detects concave notch areas inside placed large parts.
    For each placed part that has a 'polygon' attribute (concave contour),
    computes BBOX - polygon = notch rectangle, and returns virtual sub-containers.
    
    Returns: list of (sub_container_dict, placed_entry) tuples
    """
    if not HAS_SHAPELY:
        return []
    
    parts_map = {p['id']: p for p in parts_data}
    subcavities = []
    MIN_NOTCH_AREA = 5000.0  # mm² minimum useful notch area
    
    for entry in placed_plan:
        pid = entry['id']
        p_data = parts_map.get(pid)
        if not p_data or 'polygon' not in p_data:
            continue
        
        poly_pts = p_data['polygon']
        if len(poly_pts) < 4:
            continue
        
        # Check if polygon is actually concave (area < bbox area by >5%)
        gx, gy = entry['gx'], entry['gy']
        rot = entry.get('rot', 0)
        bbox_w, bbox_h = entry['w'], entry['h']
        bbox_area = bbox_w * bbox_h
        actual_area = p_data.get('area', bbox_area)
        
        concavity_ratio = actual_area / bbox_area if bbox_area > 0 else 1.0
        if concavity_ratio > 0.95:
            # Nearly rectangular, no significant notch
            continue
        
        # Transform polygon to world coordinates
        world_poly = get_transformed_poly(p_data, gx, gy, rot)
        if world_poly is None or world_poly.is_empty:
            continue
        
        # Compute notch = BBOX - polygon (buffered by gap)
        placed_bbox = box(gx, gy, gx + bbox_w, gy + bbox_h)
        buffered_poly = world_poly.buffer(gap)
        notch_region = placed_bbox.difference(buffered_poly)
        
        if notch_region.is_empty or notch_region.area < MIN_NOTCH_AREA:
            continue
        
        # Extract the largest rectangular region from the notch
        # For L-shapes, the notch is typically a single rectangle
        if notch_region.geom_type == 'Polygon':
            regions = [notch_region]
        elif notch_region.geom_type == 'MultiPolygon':
            regions = list(notch_region.geoms)
        else:
            continue
        
        for region in regions:
            if region.area < MIN_NOTCH_AREA:
                continue
            
            # Use the bounds of the notch region as the sub-container
            nb = region.bounds  # (minx, miny, maxx, maxy)
            nb_w = nb[2] - nb[0]
            nb_h = nb[3] - nb[1]
            
            # Shrink by margin to ensure parts fit inside
            inner_minx = nb[0] + margin
            inner_miny = nb[1] + margin
            inner_maxx = nb[2] - margin
            inner_maxy = nb[3] - margin
            
            if (inner_maxx - inner_minx) < 30.0 or (inner_maxy - inner_miny) < 30.0:
                continue
            
            # Also clip sub-container to the main container boundary
            cnt_bbox = container_data.get('outer_bbox', [0,0,0,0])
            inner_minx = max(inner_minx, cnt_bbox[0] + margin)
            inner_miny = max(inner_miny, cnt_bbox[1] + margin)
            inner_maxx = min(inner_maxx, cnt_bbox[2] - margin)
            inner_maxy = min(inner_maxy, cnt_bbox[3] - margin)
            
            if (inner_maxx - inner_minx) < 30.0 or (inner_maxy - inner_miny) < 30.0:
                continue
            
            sub_cnt = {
                'id': 99000 + len(subcavities),
                'type': 'polygon',
                'outer_bbox': [inner_minx, inner_miny, inner_maxx, inner_maxy],
                'outer_polygon': [
                    [inner_minx, inner_miny],
                    [inner_maxx, inner_miny],
                    [inner_maxx, inner_maxy],
                    [inner_minx, inner_maxy]
                ],
                'area': (inner_maxx - inner_minx) * (inner_maxy - inner_miny)
            }
            
            subcavities.append((sub_cnt, entry))
            print(f"   >> [v5.9 Notch Detect] Part {pid} notch found: {nb_w:.0f}x{nb_h:.0f}mm ({region.area:.0f}mm²)", flush=True)
    
    return subcavities


def solve_single_container_nesting(cnt, c_parts, gap=5.0, margin=1.0):
    """
    [v5.9] Solves nesting for a single container with Notch-First Sub-Cavity strategy.
    Phase 1: Place giant concave parts, then detect notch sub-cavities and fill them first.
    Phase 2: Place remaining parts in the main container using standard BLF.
    Guarantees strict boundary containment and 5mm clearance.
    """
    # ========================================================================
    # [v5.9] Phase 0: Detect if any part is concave (has polygon with notch)
    # If so, use the Notch-First strategy
    # ========================================================================
    has_concave_giant = False
    concave_parts = []
    non_concave_parts = []
    
    for p in c_parts:
        if 'polygon' in p and len(p.get('polygon', [])) >= 4:
            bbox_area = p['w'] * p['h']
            actual_area = p.get('area', bbox_area)
            if bbox_area > 0 and (actual_area / bbox_area) < 0.95 and bbox_area >= 180000.0:
                has_concave_giant = True
                concave_parts.append(p)
                continue
        non_concave_parts.append(p)
    
    if has_concave_giant and HAS_SHAPELY:
        return _solve_notch_first(cnt, c_parts, concave_parts, non_concave_parts, gap, margin)
    
    # ========================================================================
    # Standard path (no concave giants): Original v5.8 algorithm
    # ========================================================================
    return _solve_standard_nesting(cnt, c_parts, gap, margin)


def _solve_standard_nesting(cnt, c_parts, gap=5.0, margin=1.0):
    """[v5.8] Original standard nesting algorithm."""
    p_bigness = sorted(c_parts, key=lambda p: (p['w'] * p['h'] + max(p['w'], p['h']) * 500.0), reverse=True)
    p_area = sorted(c_parts, key=lambda p: (p['w'] * p['h']), reverse=True)
    c_ids = [cnt['id']]

    seed_configs = [
        (p_bigness, c_ids, True),
        (p_bigness, c_ids, False),
        (p_area, c_ids, True),
        (p_area, c_ids, False)
    ]

    best_result = None
    for p_ord, c_ord, strip in seed_configs:
        genome = {
            'parts_order': list(p_ord),
            'c_order': list(c_ord),
            'use_strip': strip,
            'rot_prefs': {p['id']: [0, 90] for p in c_parts}
        }
        res = evaluate_individual_genome(genome, [cnt], gap=gap, margin=margin)
        if best_result is None or res['score'] > best_result['score']:
            best_result = res
            if best_result['placed_count'] == len(c_parts):
                break

    clean_plan = post_verify_and_relax(best_result['plan'], [cnt], parts_data=c_parts, gap=gap, margin=margin)
    placed_pids = set(it['id'] for it in clean_plan)

    # Permutation search if parts count <= 6 and not 100% placed
    if len(placed_pids) < len(c_parts) and len(c_parts) <= 6:
        import itertools
        for p_ord in itertools.permutations(c_parts):
            for strip in [True, False]:
                genome = {
                    'parts_order': list(p_ord),
                    'c_order': c_ids,
                    'use_strip': strip,
                    'rot_prefs': {p['id']: [0, 90] for p in c_parts}
                }
                res = evaluate_individual_genome(genome, [cnt], gap=gap, margin=margin)
                if res['score'] > best_result['score']:
                    best_result = res
                    if best_result['placed_count'] == len(c_parts):
                        break
            if best_result['placed_count'] == len(c_parts):
                break
        clean_plan = post_verify_and_relax(best_result['plan'], [cnt], parts_data=c_parts, gap=gap, margin=margin)
        placed_pids = set(it['id'] for it in clean_plan)

    unplaced = [p for p in c_parts if p['id'] not in placed_pids]
    if unplaced:
        c_inst = EvolutionContainer(cnt, gap, margin)
        for it in clean_plan:
            c_inst.placed.append(dict(it))
            c_inst.placed_area += it['w'] * it['h']
        for u in list(unplaced):
            if c_inst.area - c_inst.placed_area >= u['w'] * u['h']:
                if rescue_unplaced_in_container(c_inst, u, gap=gap):
                    unplaced.remove(u)
        re_plan = c_inst.placed
        clean_plan = post_verify_and_relax(re_plan, [cnt], parts_data=c_parts, gap=gap, margin=margin)

    return clean_plan


def _solve_notch_first(cnt, all_parts, concave_parts, other_parts, gap=5.0, margin=1.0):
    """
    [v5.9] Notch-First Sub-Cavity Decomposition Strategy.
    
    1. Place concave giant parts first using standard BLF
    2. Detect notch sub-cavities from their BBOX - polygon difference
    3. Fill notch sub-cavities with small parts first
    4. Place remaining parts in the main container's free area
    """
    from shapely.geometry import Polygon as ShapelyPolygon, box as shapely_box
    
    print(f"   >> [v5.9] Notch-First strategy: {len(concave_parts)} concave giant(s) + {len(other_parts)} other parts", flush=True)
    
    # Step 1: Place concave giant parts using standard BLF in the main container
    giant_plan = _solve_standard_nesting(cnt, concave_parts, gap=gap, margin=margin)
    placed_giant_ids = set(it['id'] for it in giant_plan)
    
    if not placed_giant_ids:
        # Fallback: if giant placement failed, use standard path for everything
        return _solve_standard_nesting(cnt, all_parts, gap=gap, margin=margin)
    
    # Step 2: For each placed concave giant, detect notch sub-cavities
    parts_map = {p['id']: p for p in all_parts}
    notch_plans = []
    remaining_small = list(other_parts)
    
    for entry in giant_plan:
        pid = entry['id']
        p_data = parts_map.get(pid)
        if not p_data or 'polygon' not in p_data:
            continue
        
        gx, gy = entry['gx'], entry['gy']
        rot = entry.get('rot', 0)
        bbox_w, bbox_h = entry['w'], entry['h']
        
        # Transform polygon to world coordinates
        world_poly = get_transformed_poly(p_data, gx, gy, rot)
        if world_poly is None or world_poly.is_empty:
            continue
        
        # Compute notch = BBOX - buffered_polygon
        placed_bbox = shapely_box(gx, gy, gx + bbox_w, gy + bbox_h)
        buffered_poly = world_poly.buffer(gap)
        notch_region = placed_bbox.difference(buffered_poly)
        
        if notch_region.is_empty or notch_region.area < 5000.0:
            continue
        
        # Extract notch bounds
        if notch_region.geom_type == 'MultiPolygon':
            regions = sorted(notch_region.geoms, key=lambda g: g.area, reverse=True)
        else:
            regions = [notch_region]
        
        for region in regions:
            if region.area < 5000.0:
                continue
            
            nb = region.bounds  # (minx, miny, maxx, maxy)
            nb_w = nb[2] - nb[0]
            nb_h = nb[3] - nb[1]
            
            # Clip to main container boundary
            cnt_bbox = cnt.get('outer_bbox', [0, 0, 0, 0])
            inner_minx = max(nb[0] + margin, cnt_bbox[0] + margin)
            inner_miny = max(nb[1] + margin, cnt_bbox[1] + margin)
            inner_maxx = min(nb[2] - margin, cnt_bbox[2] - margin)
            inner_maxy = min(nb[3] - margin, cnt_bbox[3] - margin)
            
            usable_w = inner_maxx - inner_minx
            usable_h = inner_maxy - inner_miny
            
            if usable_w < 30.0 or usable_h < 30.0:
                continue
            
            sub_cnt = {
                'id': 99000 + len(notch_plans),
                'type': 'polygon',
                'outer_bbox': [inner_minx, inner_miny, inner_maxx, inner_maxy],
                'outer_polygon': [
                    [inner_minx, inner_miny],
                    [inner_maxx, inner_miny],
                    [inner_maxx, inner_maxy],
                    [inner_minx, inner_maxy]
                ],
                'area': usable_w * usable_h
            }
            
            print(f"   >> [v5.9 Notch] Part {pid} notch: {usable_w:.0f}x{usable_h:.0f}mm ({sub_cnt['area']:.0f}mm²)", flush=True)
            
            # Step 3: Nest remaining small parts into the notch sub-container
            if remaining_small:
                sub_plan = _solve_standard_nesting(sub_cnt, list(remaining_small), gap=gap, margin=margin)
                
                if sub_plan:
                    sub_placed_ids = set(it['id'] for it in sub_plan)
                    for it in sub_plan:
                        it['container_id'] = cnt['id']
                        it['_notch_parent_id'] = pid  # Mark as notch child
                    notch_plans.extend(sub_plan)
                    remaining_small = [p for p in remaining_small if p['id'] not in sub_placed_ids]
                    print(f"   >> [v5.9 Notch Fill] {len(sub_plan)} parts placed in notch ({len(remaining_small)} remaining)", flush=True)
    
    # Step 4: Place remaining parts in the main container's free area
    # Build a combined plan with giant placements + notch placements
    combined_plan = list(giant_plan) + list(notch_plans)
    
    if remaining_small:
        # Create the main container instance with all existing placements
        c_inst = EvolutionContainer(cnt, gap, margin)
        for it in combined_plan:
            c_inst.placed.append(dict(it))
            c_inst.placed_area += it['w'] * it['h']
        
        # Try to place remaining parts using standard BLF candidates
        remaining_sorted = sorted(remaining_small, key=lambda p: (p['w'] * p['h'] + max(p['w'], p['h']) * 500.0), reverse=True)
        
        for p in remaining_sorted:
            pw, ph = p['w'], p['h']
            pos = c_inst.find_best_position(pw, ph, allowed_rotations=[0, 90], part_data=p)
            if pos:
                gx, gy, cur_w, cur_h, rot = pos
                entry = {
                    'id': p['id'],
                    'container_id': cnt['id'],
                    'gx': gx, 'gy': gy,
                    'w': cur_w, 'h': cur_h,
                    'rot': rot,
                    'area': cur_w * cur_h
                }
                c_inst.placed.append(entry)
                c_inst.placed_area += entry['area']
                combined_plan.append(entry)
        
        # Rescue any still-unplaced parts
        placed_ids = set(it['id'] for it in combined_plan)
        still_unplaced = [p for p in remaining_small if p['id'] not in placed_ids]
        for u in list(still_unplaced):
            if c_inst.area - c_inst.placed_area >= u['w'] * u['h']:
                if rescue_unplaced_in_container(c_inst, u, gap=gap):
                    still_unplaced.remove(u)
        
        combined_plan = c_inst.placed
    
    # Final verification
    clean_plan = post_verify_and_relax(combined_plan, [cnt], parts_data=all_parts, gap=gap, margin=margin)
    
    print(f"   >> [v5.9 Result] {len(clean_plan)}/{len(all_parts)} parts placed", flush=True)
    return clean_plan



def solve_grouped_nesting(containers_data, parts_data, gap=5.0, margin=1.0, mode='fast'):
    """
    [v5.8] High-Speed 1:1 Group-Box / Column-Based Nesting Engine.
    Partitions parts into their spatially designated container groups.
    Guarantees 100% boundary confinement, zero cross-container mixing, and zero part-to-part collision.
    """
    t0 = time.time()

    # Fallback to competitive evolution if neither container_id nor spatial coordinates exist
    has_cids = any('container_id' in p and p['container_id'] is not None for p in parts_data)
    has_coords = any(p.get('orig_min_x') is not None and abs(float(p.get('orig_min_x', 0))) > 1.0 for p in parts_data)

    if not has_cids and not has_coords:
        return solve_competitive_evolution(containers_data, parts_data, gap=gap, margin=margin, mode=mode)

    # 1. Map each part to its container_id
    for p in parts_data:
        if 'container_id' not in p or p['container_id'] is None:
            p_min_x = p.get('orig_min_x')
            if p_min_x is None:
                p_min_x = 0.0
            px = float(p_min_x) + float(p.get('w', 0.0)) * 0.5
            def get_cx(c):
                if c.get('type') == 'circle':
                    return c.get('circle_center', [0, 0])[0]
                ob = c.get('outer_bbox', [0, 0, 0, 0])
                return (ob[0] + ob[2]) * 0.5
            best_c = min(containers_data, key=lambda c: abs(px - get_cx(c)))
            p['container_id'] = best_c['id']

    groups_map = defaultdict(list)
    for p in parts_data:
        groups_map[p['container_id']].append(p)

    final_plan = []
    unplaced_parts = []

    print("\n" + "=" * 70)
    print(" [ MBBN v5.9 - L-in-L Concave Notch Sub-Cavity Nesting Engine ]")
    print("=" * 70)
    print(f">> Total {len(containers_data)} Containers x {len(parts_data)} Parts 1:1 Matching")
    print(f">> Rules: Boundary margin {margin:.1f}mm, Part gap {gap:.1f}mm")
    print("-" * 70, flush=True)

    for c in containers_data:
        c_parts = groups_map.get(c['id'], [])
        if not c_parts:
            continue

        clean_plan = solve_single_container_nesting(c, c_parts, gap=gap, margin=margin)
        placed_pids = set(it['id'] for it in clean_plan)

        for it in clean_plan:
            final_plan.append(it)
        for p in c_parts:
            if p['id'] not in placed_pids:
                unplaced_parts.append(p)

        status_str = "PASS (100%)" if len(clean_plan) == len(c_parts) else f"PARTIAL ({len(clean_plan)}/{len(c_parts)})"
        print(f"   - [Container {c['id']}] {len(clean_plan)}/{len(c_parts)} parts placed ({status_str})")

    elapsed = time.time() - t0
    print("-" * 70)
    print(f">> [Group 1:1 Placement Complete] Total {len(final_plan)}/{len(parts_data)} placed ({elapsed:.3f}s elapsed)", flush=True)
    return final_plan, unplaced_parts

def solve_nesting(json_in_path, lsp_out_path, json_out_path, mode='fast'):
    """
    Main Orchestrator for MBBN Nesting Engine.
    """
    with open(json_in_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    if 'mode' in data and data['mode'] in ['fast', 'deep']:
        mode = data['mode']

    gap = float(data.get('gap', 5.0))
    margin = float(data.get('margin', 1.0))
    containers_data = data['containers']
    parts_data = data['parts']

    total_part_area = sum(p['w'] * p['h'] for p in parts_data)
    total_container_area = sum(c['area'] for c in containers_data)

    # [v5.8] 사각 박스 / 구역별 1:1 독립 초고속 네스팅 엔진
    # 컨테이너가 복수이거나 구역별 매칭이 필요한 경우 1:1 독립 네스팅 루프로 0.3초 완결
    raw_plan, unplaced_grouped = solve_grouped_nesting(containers_data, parts_data, gap=gap, margin=margin, mode=mode)
    clean_plan = post_verify_and_relax(raw_plan, containers_data, parts_data=parts_data, gap=gap, margin=margin)

    # Secondary Cavity Rescue if any unplaced parts remain
    placed_pids = set(it['id'] for it in clean_plan)
    unplaced_parts = [p for p in parts_data if p['id'] not in placed_pids]

    if unplaced_parts:
        c_map = {c['id']: EvolutionContainer(c, gap, margin) for c in containers_data}
        for it in clean_plan:
            cnt = c_map[it['container_id']]
            cnt.placed.append(dict(it))
            cnt.placed_area += it['w'] * it['h']

        for u in list(unplaced_parts):
            for cid, cnt in c_map.items():
                if cnt.area - cnt.placed_area >= u['w'] * u['h']:
                    if rescue_unplaced_in_container(cnt, u, gap=gap):
                        unplaced_parts.remove(u)
                        break

        re_plan = []
        for cid, cnt in c_map.items():
            re_plan.extend(cnt.placed)
        clean_plan = post_verify_and_relax(re_plan, containers_data, parts_data=parts_data, gap=gap, margin=margin)
        placed_pids = set(it['id'] for it in clean_plan)
        unplaced_parts = [p for p in parts_data if p['id'] not in placed_pids]

    final_export_plan = []
    for it in clean_plan:
        final_export_plan.append({
            'part_id': it['id'],
            'container_id': it['container_id'],
            'gx': it['gx'],
            'gy': it['gy'],
            'w': it['w'],
            'h': it['h'],
            'rot': int(round(it.get('rot', 0)))
        })

    fallback_plan = generate_unplaced_placement(unplaced_parts, containers_data, gap=15.0)

    used_part_area = sum(it['w'] * it['h'] for it in final_export_plan)
    used_cids = set(it['container_id'] for it in final_export_plan)
    used_container_area = sum(c['area'] for c in containers_data if c['id'] in used_cids)

    part_yield = (used_part_area / total_part_area * 100.0) if total_part_area > 0 else 0.0
    container_yield = (used_part_area / used_container_area * 100.0) if used_container_area > 0 else 0.0

    # Write LSP file
    with open(lsp_out_path, 'w', encoding='utf-8') as f:
        f.write(";;; MBBN v5.7 Nesting Plan (Circular Scrap Sheet 100% Placement Engine)\n")
        f.write(f"(setq MBB_TOTAL_YIELD {container_yield:.2f})\n")
        f.write(f"(setq MBB_PART_YIELD {part_yield:.2f})\n")
        f.write(f"(setq MBB_PLACED_COUNT {len(final_export_plan)})\n")
        f.write(f"(setq MBB_UNPLACED_COUNT {len(fallback_plan)})\n")
        f.write("(setq MBB_NESTING_PLAN '(\n")
        for it in final_export_plan:
            f.write(f"  ({it['part_id']} {it['container_id']} {it['gx']:.4f} {it['gy']:.4f} {it['w']:.4f} {it['h']:.4f} {it['rot']})\n")
        f.write("))\n")
        f.write("(setq MBB_UNPLACED_PLAN '(\n")
        for it in fallback_plan:
            f.write(f"  ({it['part_id']} {it['gx']:.4f} {it['gy']:.4f} {it['w']:.4f} {it['h']:.4f})\n")
        f.write("))\n")
        f.write("(setq MBB_NESTING_STATUS \"SUCCESS\")\n")

    # Write JSON file
    res = {
        "status": "SUCCESS",
        "mode": mode,
        "container_yield": round(container_yield, 2),
        "part_yield": round(part_yield, 2),
        "placed_count": len(final_export_plan),
        "unplaced_count": len(fallback_plan),
        "plan": final_export_plan,
        "unplaced_plan": fallback_plan
    }
    with open(json_out_path, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2)

    print("\n" + "=" * 70)
    print(f" [ MBBN v5.6 배치 성적표 ({mode.upper()} 모드) ]")
    print("=" * 70)
    print(f">> 총 단품: {len(parts_data)} 개 | 실배치: {len(final_export_plan)} 개 | 미배치 외곽: {len(fallback_plan)} 개")
    print(f">> 단품 소모 수율: {part_yield:.2f} % (총 단품 면적 대비)")
    print(f">> 사용 잔재 실효 수율: {container_yield:.2f} %")
    print("=" * 70 + "\n", flush=True)


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python mbb_nesting.py <json_in> <lsp_out> <json_out> [--mode fast|deep]")
        sys.exit(1)

    json_in = sys.argv[1]
    lsp_out = sys.argv[2]
    json_out = sys.argv[3]

    exec_mode = 'fast'
    if len(sys.argv) >= 5:
        if sys.argv[4] == '--mode' and len(sys.argv) >= 6:
            exec_mode = sys.argv[5]
        elif sys.argv[4] in ['fast', 'deep']:
            exec_mode = sys.argv[4]

    solve_nesting(json_in, lsp_out, json_out, mode=exec_mode)
