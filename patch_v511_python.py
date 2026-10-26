# -*- coding: utf-8 -*-
"""
mbb_nesting.py v5.11 패치: 호(Arc/Bulge) 곡선 단품 BBOX면적 필터 수정
- find_best_position: pw*ph > rem_area 조기 탈락에 polygon 실면적 사용
- Strip Stacking: part_poly 전달 누락 수정
"""
import datetime

now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

with open('mbb_nesting.py', 'r', encoding='utf-8') as f:
    content = f.read()

original = content

# ============================================================
# PATCH 1: find_best_position - BBOX면적 대신 polygon 실면적 사용
# Line 298-300: 
#   rem_area = self.area - self.placed_area
#   if pw * ph > rem_area:
#       return None
# ============================================================
old_area_check = """        rem_area = self.area - self.placed_area
        if pw * ph > rem_area:
            return None"""

new_area_check = """        rem_area = self.area - self.placed_area
        # [v5.11 FIX] Use actual polygon area if available, not BBOX area
        # D-shape/crescent parts have BBOX area >> actual polygon area
        part_actual_area = pw * ph
        if part_data and 'polygon' in part_data and len(part_data.get('polygon', [])) >= 3:
            _pp = part_data['polygon']
            _sa = 0.0
            for _pi in range(len(_pp)):
                _x1, _y1 = _pp[_pi]
                _x2, _y2 = _pp[(_pi + 1) % len(_pp)]
                _sa += _x1 * _y2 - _x2 * _y1
            part_actual_area = abs(_sa) * 0.5
            if part_actual_area < 1.0:
                part_actual_area = pw * ph  # fallback
        if part_actual_area > rem_area:
            return None"""

if old_area_check in content:
    content = content.replace(old_area_check, new_area_check)
    print('  [OK] PATCH 1: find_best_position area check fixed')
else:
    print('  [FAIL] PATCH 1: cannot find target')

# ============================================================
# PATCH 2: Version header update
# ============================================================
old_header = ";;;                                 5. [v5.9d FIX]"
new_header = f";;;                                 6. [v5.11 FIX] find_best_position BBOX면적→polygon 실면적 필터 수정\n;;;                                 5. [v5.9d FIX]"
if old_header in content:
    content = content.replace(old_header, new_header, 1)
    print('  [OK] PATCH 2: version header updated')

# ============================================================
# PATCH 3: Strip stacking - pass part_data for polygon check
# Line 719: if not cnt.can_place(start_x, cy, cur_w, cur_h):
# Need to add part_poly for polygon containers
# ============================================================
old_strip = """                        for bi in range(num_bars):
                            cy = start_y + bi * (cur_h + cnt.gap)
                            if not cnt.can_place(start_x, cy, cur_w, cur_h):
                                all_fit = False
                                break
                            stack_entries.append((avail_pids[bi], start_x, cy, cur_w, cur_h, float(rot)))"""

new_strip = """                        for bi in range(num_bars):
                            cy = start_y + bi * (cur_h + cnt.gap)
                            # [v5.11 FIX] Pass part polygon for non-rectangular containers
                            _sp = parts_dict.get(avail_pids[bi])
                            _sp_poly = get_transformed_poly(_sp, start_x, cy, rot) if _sp else None
                            if not cnt.can_place(start_x, cy, cur_w, cur_h, part_poly=_sp_poly):
                                all_fit = False
                                break
                            stack_entries.append((avail_pids[bi], start_x, cy, cur_w, cur_h, float(rot)))"""

if old_strip in content:
    content = content.replace(old_strip, new_strip)
    print('  [OK] PATCH 3: strip stacking part_poly fix')
else:
    print('  [FAIL] PATCH 3: cannot find strip target')

# ============================================================
# PATCH 4: Giant part preemptive placement threshold
# Line 649: giant_parts = [p for p in parts_order if (p['w'] * p['h']) >= 180000.0]
# This uses BBOX area. For arc parts, actual area could be much less.
# Not critical for this bug, but let's note it.
# ============================================================

# ============================================================  
# Final verification
# ============================================================
checks = [
    ('area check fix', 'part_actual_area > rem_area' in content),
    ('Shoelace in find_best_position', '_sa += _x1 * _y2 - _x2 * _y1' in content),
    ('strip part_poly', '_sp_poly = get_transformed_poly(_sp, start_x, cy, rot)' in content),
    ('original functions preserved', 'def find_best_position' in content),
    ('evaluate preserved', 'def evaluate_individual_genome' in content),
]

all_ok = True
for name, ok in checks:
    status = 'OK' if ok else 'FAIL'
    print(f'  [{status}] {name}')
    if not ok:
        all_ok = False

if all_ok:
    with open('mbb_nesting.py', 'w', encoding='utf-8') as f:
        f.write(content)
    import os
    fsize = os.path.getsize('mbb_nesting.py')
    print(f'\n[SUCCESS] mbb_nesting.py v5.11 패치 완료 ({now_str})')
    print(f'  파일 크기: {fsize} bytes')
else:
    print('\n[ABORT] 패치 검증 실패. 파일 변경 없음.')
