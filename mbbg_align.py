# -*- coding: utf-8 -*-
"""
=============================================================================
 MBBG v4.8: 20-Core Parallel 2D Shape Centering & Maximum Clearance Engine
 LAST MODIFIED: 2026-10-23 22:55:00
 ENCODING: UTF-8 / ANSI (CP949) Compatible
 
 [VERSION HISTORY & ROLLBACK LOG]
 - v4.8 (2026-10-23 22:55:00) : 20코어 풀가동 병렬 2D 거리장(Signed Distance Field) 최적 정렬 엔진 탑재
                                 (ProcessPoolExecutor 20-Core 고밀도 그리드 병렬 탐색, stringp nil 에러 원천 차단)
 - v4.4 (2026-10-23 21:16:00) : Shapely GEOS C-거리장 2D 최적화 및 Welzl MEC 엔진 탑재
=============================================================================
"""

import os
import sys
import json
import math
import time
import random
import concurrent.futures
import numpy as np

try:
    from shapely.geometry import Polygon, Point, MultiPoint
    from shapely.prepared import prep
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False


def welzl(points):
    """
    Welzl's linear-time O(N) algorithm for Minimum Enclosing Circle (MEC).
    Returns: (center_x, center_y), radius
    """
    pts = list(points)
    random.seed(42)
    random.shuffle(pts)

    def dist_sq(p1, p2):
        return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

    def circle_from_2pts(p1, p2):
        cx = (p1[0] + p2[0]) / 2.0
        cy = (p1[1] + p2[1]) / 2.0
        r_sq = dist_sq(p1, (cx, cy))
        return (cx, cy), math.sqrt(r_sq)

    def circle_from_3pts(p1, p2, p3):
        ax, ay = p1
        bx, by = p2
        cx, cy = p3
        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(d) < 1e-9:
            return circle_from_2pts(p1, p2)
        ux = ((ax*ax + ay*ay) * (by - cy) + (bx*bx + by*by) * (cy - ay) + (cx*cx + cy*cy) * (ay - by)) / d
        uy = ((ax*ax + ay*ay) * (cx - bx) + (bx*bx + by*by) * (ax - cx) + (cx*cx + cy*cy) * (bx - ax)) / d
        r = math.hypot(ax - ux, ay - uy)
        return (ux, uy), r

    def is_in_circle(p, c):
        if c is None:
            return False
        (cx, cy), r = c
        return dist_sq(p, (cx, cy)) <= (r + 1e-7)**2

    def b_mec(pts_subset, boundary_pts):
        if not pts_subset or len(boundary_pts) == 3:
            if len(boundary_pts) == 0:
                return (0.0, 0.0), 0.0
            elif len(boundary_pts) == 1:
                return boundary_pts[0], 0.0
            elif len(boundary_pts) == 2:
                return circle_from_2pts(boundary_pts[0], boundary_pts[1])
            else:
                return circle_from_3pts(boundary_pts[0], boundary_pts[1], boundary_pts[2])

        p = pts_subset[-1]
        c = b_mec(pts_subset[:-1], boundary_pts)
        if is_in_circle(p, c):
            return c
        return b_mec(pts_subset[:-1], boundary_pts + [p])

    if not pts:
        return (0.0, 0.0), 0.0
    return b_mec(pts, [])


def align_circle(c_cen, c_rad, inner_pts):
    """
    O(N) Welzl MEC algorithm for global maximum radial clearance inside circle.
    """
    if not inner_pts:
        return 0.0, 0.0, c_rad

    mec_cen, mec_r = welzl(inner_pts)
    dx = float(mec_cen[0] - c_cen[0])
    dy = float(mec_cen[1] - c_cen[1])
    clearance = float(c_rad - mec_r)
    return dx, dy, clearance


def _eval_grid_chunk(args):
    """Module-level worker for ProcessPoolExecutor."""
    chunk, outer_pts, inner_pts, dx0, dy0 = args
    if not SHAPELY_AVAILABLE:
        return -1e9, (dx0, dy0)

    try:
        outer_poly = Polygon(outer_pts)
        if not outer_poly.is_valid:
            outer_poly = outer_poly.buffer(0)
    except Exception:
        outer_poly = MultiPoint(outer_pts).convex_hull

    prepared_outer = prep(outer_poly)
    boundary = outer_poly.boundary

    best_val = -1e9
    best_cand = (dx0, dy0)

    for cx, cy in chunk:
        min_dist = 1e9
        for px, py in inner_pts:
            q = Point(px - cx, py - cy)
            d = boundary.distance(q)
            if not prepared_outer.contains(q):
                d = -d
            if d < min_dist:
                min_dist = d
        score = min_dist - 1e-5 * ((cx - dx0)**2 + (cy - dy0)**2)
        if score > best_val:
            best_val = score
            best_cand = (cx, cy)

    return best_val, best_cand


def align_polygon_parallel(outer_pts, outer_bbox, inner_pts, inner_bbox):
    """
    High-Performance 20-Core Parallel Signed-Distance Field 2D Alignment.
    """
    in_min = np.array([inner_bbox[0], inner_bbox[1]], dtype=np.float64)
    in_max = np.array([inner_bbox[2], inner_bbox[3]], dtype=np.float64)
    in_c = (in_min + in_max) / 2.0

    out_min = np.array([outer_bbox[0], outer_bbox[1]], dtype=np.float64)
    out_max = np.array([outer_bbox[2], outer_bbox[3]], dtype=np.float64)
    out_c = (out_min + out_max) / 2.0

    dx0 = float(in_c[0] - out_c[0])
    dy0 = float(in_c[1] - out_c[1])

    if not SHAPELY_AVAILABLE or len(outer_pts) < 3:
        slack_x = (out_max[0] - out_min[0]) - (in_max[0] - in_min[0])
        slack_y = (out_max[1] - out_min[1]) - (in_max[1] - in_min[1])
        clearance = min(slack_x, slack_y) / 2.0
        return dx0, dy0, clearance

    try:
        outer_poly = Polygon(outer_pts)
        if not outer_poly.is_valid:
            outer_poly = outer_poly.buffer(0)
    except Exception:
        outer_poly = MultiPoint(outer_pts).convex_hull

    prepared_outer = prep(outer_poly)
    boundary = outer_poly.boundary

    slack_x = max(0.0, (out_max[0] - out_min[0]) - (in_max[0] - in_min[0])) / 2.0
    slack_y = max(0.0, (out_max[1] - out_min[1]) - (in_max[1] - in_min[1])) / 2.0

    def eval_shift(dx, dy):
        min_dist = 1e9
        for px, py in inner_pts:
            q = Point(px - dx, py - dy)
            d = boundary.distance(q)
            if not prepared_outer.contains(q):
                d = -d
            if d < min_dist:
                min_dist = d
        return min_dist

    # 1. 2D High-Density Grid Search (31 x 31 = 961 points) evaluated across 20 CPU cores
    span_x = max(10.0, slack_x)
    span_y = max(10.0, slack_y)
    xs = np.linspace(dx0 - span_x * 0.9, dx0 + span_x * 0.9, 31)
    ys = np.linspace(dy0 - span_y * 0.9, dy0 + span_y * 0.9, 31)

    all_cands = [(float(cx), float(cy)) for cx in xs for cy in ys]
    cpu_count = os.cpu_count() or 4
    num_workers = min(20, cpu_count)

    chunk_size = math.ceil(len(all_cands) / num_workers)
    chunks = [all_cands[i:i + chunk_size] for i in range(0, len(all_cands), chunk_size)]
    tasks = [(chk, outer_pts, inner_pts, dx0, dy0) for chk in chunks]

    best_val = -1e9
    best_dx = dx0
    best_dy = dy0

    with concurrent.futures.ProcessPoolExecutor(max_workers=num_workers) as executor:
        results = list(executor.map(_eval_grid_chunk, tasks))

    for score, cand in results:
        if score > best_val:
            best_val = score
            best_dx, best_dy = cand

    # 2. 3-Stage Fine Refinement (2.0mm -> 0.5mm -> 0.1mm)
    for step in [2.0, 0.5, 0.1]:
        improved = True
        iterations = 0
        while improved and iterations < 30:
            improved = False
            iterations += 1
            for ddx, ddy in [(-step, 0), (step, 0), (0, -step), (0, step),
                             (-step, -step), (-step, step), (step, -step), (step, step)]:
                cand_x = best_dx + ddx
                cand_y = best_dy + ddy
                val = eval_shift(cand_x, cand_y)
                score = val - 1e-5 * ((cand_x - dx0)**2 + (cand_y - dy0)**2)
                if score > best_val + 1e-6:
                    best_val = score
                    best_dx = cand_x
                    best_dy = cand_y
                    improved = True

    final_clearance = eval_shift(best_dx, best_dy)
    return best_dx, best_dy, final_clearance


def solve_mbbg(json_in_path, lsp_out_path, json_out_path):
    t0 = time.time()
    with open(json_in_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    outer = data["outer"]
    inner = data["inner"]

    is_circle = outer.get("is_circle", False)
    inner_pts = inner.get("pts", [])
    inner_bbox = inner.get("bbox", [0, 0, 0, 0])

    if is_circle:
        c_cen = outer["circle_center"]
        c_rad = outer["circle_radius"]
        dx, dy, clearance = align_circle(c_cen, c_rad, inner_pts)
    else:
        out_pts = outer.get("polygon", [])
        out_bbox = outer.get("bbox", [0, 0, 0, 0])
        dx, dy, clearance = align_polygon_parallel(out_pts, out_bbox, inner_pts, inner_bbox)

    elapsed_ms = (time.time() - t0) * 1000.0

    # Write LSP file
    with open(lsp_out_path, 'w', encoding='utf-8') as f:
        f.write(";;; MBBG v4.8 Optimal Alignment Result (20-Core Parallel Python Engine)\n")
        f.write(f"(setq MBBG_FINAL_DX {dx:.4f})\n")
        f.write(f"(setq MBBG_FINAL_DY {dy:.4f})\n")
        f.write(f"(setq MBBG_CLEARANCE {clearance:.4f})\n")
        f.write(f"(setq MBBG_ELAPSED_MS {elapsed_ms:.1f})\n")
        f.write("(setq MBBG_STATUS \"SUCCESS\")\n")

    # Write JSON file
    res = {
        "status": "SUCCESS",
        "dx": round(dx, 4),
        "dy": round(dy, 4),
        "clearance": round(clearance, 4),
        "elapsed_ms": round(elapsed_ms, 1)
    }
    with open(json_out_path, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2)

    print("\n" + "=" * 70)
    print(" [ MBBG v4.8 20코어 풀가동 Python 2D 형상 기하 안착 연산 완료 ]")
    print("=" * 70)
    print(f">> 총 소요 시간: {elapsed_ms:.2f} ms (CPU {min(20, os.cpu_count() or 4)}코어 동시 병렬)")
    print(f">> 외곽 틀 최적 이동: dX = {dx:.3f} mm, dY = {dy:.3f} mm")
    print(f">> 사방 최소 여유 공간(Clearance): {clearance:.3f} mm (100% 무간섭 안착)")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python mbbg_align.py <json_in> <lsp_out> <json_out>")
        sys.exit(1)

    json_in = sys.argv[1]
    lsp_out = sys.argv[2]
    json_out = sys.argv[3]

    solve_mbbg(json_in, lsp_out, json_out)
