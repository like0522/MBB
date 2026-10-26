# -*- coding: utf-8 -*-
"""
mbb_nesting.py v5.11b: JSON 입력 백업 + BBOX 면적 필터 전면 수정
"""
import datetime

now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

with open('mbb_nesting.py', 'r', encoding='utf-8') as f:
    content = f.read()

# ============================================================
# PATCH A: solve_nesting 시작 시 JSON 백업 캡처
# ============================================================
old_solve = """    with open(json_in_path, 'r', encoding='utf-8') as f:
        data = json.load(f)"""

new_solve = """    with open(json_in_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    # [v5.11 DEBUG] Save JSON input copy for diagnosis
    try:
        import shutil
        _dbg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'DEBUG_nesting_input.json')
        shutil.copy2(json_in_path, _dbg_path)
    except Exception:
        pass"""

if old_solve in content:
    content = content.replace(old_solve, new_solve, 1)
    print('  [OK] PATCH A: JSON backup added')
else:
    print('  [FAIL] PATCH A')

# ============================================================
# PATCH B: rescue_unplaced_in_container BBOX 면적 필터 수정 (line 1604)
# ============================================================
old_rescue = "if cnt.area - cnt.placed_area >= u['w'] * u['h']:"
new_rescue = """# [v5.11 FIX] Use actual polygon area for rescue check
                    _u_area = u['w'] * u['h']
                    if 'polygon' in u and len(u.get('polygon',[])) >= 3:
                        _up = u['polygon']
                        _usa = sum(_up[_i][0]*_up[(_i+1)%len(_up)][1] - _up[(_i+1)%len(_up)][0]*_up[_i][1] for _i in range(len(_up)))
                        _u_area = max(abs(_usa)*0.5, 1.0)
                    if cnt.area - cnt.placed_area >= _u_area:"""

if old_rescue in content:
    content = content.replace(old_rescue, new_rescue, 1)
    print('  [OK] PATCH B: rescue area filter fixed')
else:
    print('  [FAIL] PATCH B')

# ============================================================
# PATCH C: solve_nesting 시작 시 디버그 출력
# ============================================================
old_total = """    total_part_area = sum(p['w'] * p['h'] for p in parts_data)
    total_container_area = sum(c['area'] for c in containers_data)"""

new_total = """    total_part_area = sum(p['w'] * p['h'] for p in parts_data)
    total_container_area = sum(c['area'] for c in containers_data)

    # [v5.11 DEBUG] Print container and part details
    for _c in containers_data:
        _op = _c.get('outer_polygon', [])
        print(f"   >> [DEBUG] Container {_c.get('id')}: type={_c.get('type')}, outer_polygon={len(_op)} pts, area={_c.get('area')}", flush=True)
    for _p in parts_data:
        _pp = _p.get('polygon', [])
        _pa = _p['w'] * _p['h']
        if _pp and len(_pp) >= 3:
            _n = len(_pp)
            _s = sum(_pp[_i][0]*_pp[(_i+1)%_n][1] - _pp[(_i+1)%_n][0]*_pp[_i][1] for _i in range(_n))
            _pa = abs(_s)*0.5
        print(f"   >> [DEBUG] Part {_p.get('id')}: w={_p.get('w')}, h={_p.get('h')}, polygon={len(_pp)} pts, actual_area={_pa:.0f}, cid={_p.get('container_id')}", flush=True)"""

if old_total in content:
    content = content.replace(old_total, new_total, 1)
    print('  [OK] PATCH C: debug output added')
else:
    print('  [FAIL] PATCH C')

# ============================================================
# Verify and save
# ============================================================
all_ok = all([
    'DEBUG_nesting_input.json' in content,
    '_u_area = u[' in content,
    '[DEBUG] Container' in content,
])
if all_ok:
    with open('mbb_nesting.py', 'w', encoding='utf-8') as f:
        f.write(content)
    import os
    print(f'\n[SUCCESS] mbb_nesting.py v5.11b 패치 완료 ({now_str})')
    print(f'  파일 크기: {os.path.getsize("mbb_nesting.py")} bytes')
else:
    print('\n[ABORT] 패치 실패')
