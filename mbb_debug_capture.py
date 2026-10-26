# -*- coding: utf-8 -*-
"""
mbb_nesting.py 디버그 래퍼: JSON 입력 캡처 + 상세 로그
TEMPPREFIX 디렉토리의 mbb_nesting_in.json을 MBB 디렉토리에 복사 보존
"""
import sys, os, json, shutil

# mbb_nesting.py를 직접 임포트하여 원래 solve 함수 호출 전 JSON 캡처
def main():
    if len(sys.argv) < 4:
        print("Usage: python mbb_debug_capture.py <json_in> <lsp_out> <json_out>")
        sys.exit(1)
    
    json_in = sys.argv[1]
    lsp_out = sys.argv[2]
    json_out = sys.argv[3]
    
    # JSON 입력 파일을 MBB 디렉토리에 복사 보존
    backup_dir = r"c:\Users\like\OneDrive\문서\MBB"
    backup_path = os.path.join(backup_dir, "DEBUG_nesting_input.json")
    try:
        shutil.copy2(json_in, backup_path)
        print(f"[DEBUG] JSON input saved to: {backup_path}", flush=True)
    except Exception as e:
        print(f"[DEBUG] Failed to save JSON: {e}", flush=True)
    
    # JSON 내용 분석
    try:
        with open(json_in, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        containers = data.get('containers', [])
        parts = data.get('parts', [])
        
        print(f"[DEBUG] Containers: {len(containers)}", flush=True)
        for c in containers:
            ctype = c.get('type', '?')
            outer_pts = c.get('outer_polygon', [])
            print(f"  Container {c.get('id')}: type={ctype}, outer_polygon={len(outer_pts)} pts, area={c.get('area')}", flush=True)
            if outer_pts:
                xs = [p[0] for p in outer_pts]
                ys = [p[1] for p in outer_pts]
                print(f"    Polygon BBOX: X=[{min(xs):.2f}, {max(xs):.2f}], Y=[{min(ys):.2f}, {max(ys):.2f}]", flush=True)
                print(f"    Polygon width={max(xs)-min(xs):.1f}, height={max(ys)-min(ys):.1f}", flush=True)
        
        print(f"[DEBUG] Parts: {len(parts)}", flush=True)
        for p in parts:
            poly = p.get('polygon', [])
            print(f"  Part {p.get('id')}: w={p.get('w')}, h={p.get('h')}, area={p.get('area')}, polygon={len(poly)} pts, container_id={p.get('container_id')}", flush=True)
            if poly:
                xs = [pt[0] for pt in poly]
                ys = [pt[1] for pt in poly]
                print(f"    Polygon local BBOX: X=[{min(xs):.2f}, {max(xs):.2f}], Y=[{min(ys):.2f}, {max(ys):.2f}]", flush=True)
                # Shoelace area
                n = len(poly)
                sa = 0.0
                for i in range(n):
                    x1, y1 = poly[i]
                    x2, y2 = poly[(i+1) % n]
                    sa += x1*y2 - x2*y1
                actual_area = abs(sa) * 0.5
                print(f"    Polygon Shoelace area: {actual_area:.0f} mm² (BBOX area: {p.get('w',0)*p.get('h',0):.0f})", flush=True)
    except Exception as e:
        print(f"[DEBUG] JSON analysis error: {e}", flush=True)
    
    # 원래 mbb_nesting.py 호출
    print("[DEBUG] Calling mbb_nesting.py...", flush=True)
    sys.argv = ['mbb_nesting.py', json_in, lsp_out, json_out]
    
    # Import and run
    script_dir = os.path.dirname(os.path.abspath(__file__))
    nesting_path = os.path.join(script_dir, 'mbb_nesting.py')
    
    import importlib.util
    spec = importlib.util.spec_from_file_location("mbb_nesting", nesting_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

if __name__ == '__main__':
    main()
