# -*- coding: utf-8 -*-
"""
analyze_l_shape_reports.py - Analyze the two GEOMINFO reports for L-shaped nesting
"""

# ============================================================
# Report 1: 원본 외관 (Original Layout) - 25 items
# ============================================================
report1 = {
    'items': [
        # [1] L-shaped container (or big piece)
        {'id': 1, 'type': 'polyline', 'color': 256, 'closed': True, 'area': 1818350, 'length': 5700,
         'vertices': [(478739.7788,-131866.5355), (478739.7788,-132076.5355), (479044.7788,-132076.5355),
                      (479044.7788,-133676.5355), (478004.7788,-133676.5355), (478004.7788,-131866.5355)],
         'bbox': (478004.7788, -133676.5355, 479044.7788, -131866.5355),
         'label': 'L-shape 1040x1810'},
        # [2] Circle (bolt hole)
        {'id': 2, 'type': 'circle', 'color': 256, 'center': (478086.4241,-130723.0156), 'radius': 54.5},
        # [3] Circle (bolt hole)
        {'id': 3, 'type': 'circle', 'color': 256, 'center': (478636.4241,-130723.0156), 'radius': 54.5},
        # [4] L-shaped part (700-22)
        {'id': 4, 'type': 'polyline', 'color': 256, 'closed': True, 'area': 896244, 'length': 4452,
         'vertices': [(477857.4241,-131077.0156), (477857.4241,-129859.0156), (478215.4241,-129859.0156),
                      (478215.4241,-130369.0156), (478865.4241,-130369.0156), (478865.4241,-131077.0156)],
         'bbox': (477857.4241, -131077.0156, 478865.4241, -129859.0156),
         'label': '700-22 L-shape 1008x1218'},
        # [5] Text for 700-22
        {'id': 5, 'type': 'text', 'content': '700-22_16tx1000x1210- 2 EA'},
        # Parts (paired with text):
        # [6-7] 700-1 (122x112)
        {'id': 6, 'type': 'polyline', 'color': 256, 'area': 13664, 'w': 122, 'h': 112, 'label': '700-1'},
        # [8-9] 700-1 (122x112)
        {'id': 8, 'type': 'polyline', 'color': 256, 'area': 13664, 'w': 122, 'h': 112, 'label': '700-1'},
        # [10-11] 700-1 (122x112)
        {'id': 10, 'type': 'polyline', 'color': 256, 'area': 13664, 'w': 122, 'h': 112, 'label': '700-1'},
        # [12-13] 700-7 (52x452)
        {'id': 12, 'type': 'polyline', 'color': 256, 'area': 23504, 'w': 52, 'h': 452, 'label': '700-7'},
        # [14-15] 700-8 (67x67)
        {'id': 14, 'type': 'polyline', 'color': 256, 'area': 4489, 'w': 67, 'h': 67, 'label': '700-8'},
        # [16-17] 700-8 (67x67)
        {'id': 16, 'type': 'polyline', 'color': 256, 'area': 4489, 'w': 67, 'h': 67, 'label': '700-8'},
        # [18-19] 700-7 (52x452)
        {'id': 18, 'type': 'polyline', 'color': 256, 'area': 23504, 'w': 52, 'h': 452, 'label': '700-7'},
        # [20-21] 700-1 (122x112)
        {'id': 20, 'type': 'polyline', 'color': 256, 'area': 13664, 'w': 122, 'h': 112, 'label': '700-1'},
        # [22-23] 102-10 (arc shape, ~437x270)
        {'id': 22, 'type': 'polyline', 'color': 256, 'area': 51854, 'label': '102-10 arc shape',
         'bbox_w': 437, 'bbox_h': 270, 'has_bulge': True},
        # [24-25] 604-39 (chamfered polygon, ~249x144)
        {'id': 24, 'type': 'polyline', 'color': 256, 'area': 32688, 'label': '604-39 chamfered',
         'bbox_w': 249, 'bbox_h': 144, 'vertex_count': 8},
    ]
}

# ============================================================
# Report 2: 사용자 수동 배치 결과 (Desired Arrangement) - 25 items
# ============================================================

# Container (Color=6)
container = {
    'type': 'L-shape', 'color': 6,
    'vertices': [(475505.0853,-132773.7104), (475505.0853,-132983.7104), (475810.0853,-132983.7104),
                 (475810.0853,-134583.7104), (474770.0853,-134583.7104), (474770.0853,-132773.7104)],
    'bbox': (474770.0853, -134583.7104, 475810.0853, -132773.7104),
    'bbox_w': 1040, 'bbox_h': 1810,
    'area': 1818350,
    'notch': {'x_min': 475505.0853, 'y_min': -132983.7104, 'x_max': 475810.0853, 'y_max': -132773.7104,
              'w': 305, 'h': 210}
}

# 700-22 placed inside container
part_700_22_placed = {
    'label': '700-22',
    'vertices': [(474780.0853,-134573.7104), (474780.0853,-133355.7104), (475138.0853,-133355.7104),
                 (475138.0853,-133865.7104), (475788.0853,-133865.7104), (475788.0853,-134573.7104)],
    'bbox': (474780.0853, -134573.7104, 475788.0853, -133355.7104),
    'bbox_w': 1008, 'bbox_h': 1218,
    'margin_from_container': {
        'left': 474780.0853 - 474770.0853,   # = 10mm
        'bottom': -134573.7104 - (-134583.7104),  # = 10mm
        'right': 475810.0853 - 475788.0853,   # = 22mm
    },
    'notch': {
        'x_min': 475138.0853, 'y_min': -133865.7104,
        'x_max': 475788.0853, 'y_max': -133355.7104,
        'w': 650, 'h': 510,
        'area': 650 * 510  # = 331,500
    }
}

# Small parts placed in the 700-22's notch area
small_parts_placed = [
    {'label': '102-10', 'bbox': (475143.0853,-133860.7104, 475412.5853,-133423.7104), 'w': 270, 'h': 437, 'area': 51854},
    {'label': '604-39', 'bbox': (475542.0854,-133860.7104, 475686.0853,-133611.7105), 'w': 144, 'h': 249, 'area': 32688},
    {'label': '700-1 #1', 'bbox': (475304.2962,-133642.1296, 475416.2962,-133520.1296), 'w': 112, 'h': 122, 'area': 13664},
    {'label': '700-1 #2', 'bbox': (475421.2962,-133642.1296, 475533.2962,-133520.1296), 'w': 112, 'h': 122, 'area': 13664},
    {'label': '700-1 #3', 'bbox': (475308.0854,-133769.1296, 475420.0854,-133647.1296), 'w': 112, 'h': 122, 'area': 13664},
    {'label': '700-1 #4', 'bbox': (475425.0854,-133860.7104, 475537.0854,-133738.7104), 'w': 112, 'h': 122, 'area': 13664},
    {'label': '700-7 #1', 'bbox': (475691.0853,-133860.7104, 475743.0853,-133408.7104), 'w': 52, 'h': 452, 'area': 23504},
    {'label': '700-7 #2', 'bbox': (475748.0853,-133860.7104, 475800.0853,-133408.7104), 'w': 52, 'h': 452, 'area': 23504},
    {'label': '700-8 #1', 'bbox': (475425.0863,-133733.7104, 475492.0863,-133666.7104), 'w': 67, 'h': 67, 'area': 4489},
    {'label': '700-8 #2', 'bbox': (475538.2962,-133606.7105, 475605.2962,-133539.7105), 'w': 67, 'h': 67, 'area': 4489},
]

print("=" * 70)
print(" L형 잔재 네스팅 분석 보고서")
print("=" * 70)

print(f"\n[컨테이너] L형 잔재판")
print(f"  전체 BBOX: {container['bbox_w']}mm x {container['bbox_h']}mm")
print(f"  면적: {container['area']:,.0f} mm²")
print(f"  노치(컨테이너): {container['notch']['w']}mm x {container['notch']['h']}mm (상단 우측)")

print(f"\n[대형 단품] 700-22 (L형)")
print(f"  BBOX: {part_700_22_placed['bbox_w']}mm x {part_700_22_placed['bbox_h']}mm")
print(f"  배치 위치: 좌하단 (마진 L={part_700_22_placed['margin_from_container']['left']:.0f}mm, B={part_700_22_placed['margin_from_container']['bottom']:.0f}mm)")
print(f"  노치(단품): {part_700_22_placed['notch']['w']}mm x {part_700_22_placed['notch']['h']}mm")
print(f"  노치 면적: {part_700_22_placed['notch']['area']:,.0f} mm²")

total_small_area = sum(p['area'] for p in small_parts_placed)
print(f"\n[소형 단품] 총 {len(small_parts_placed)}개")
for p in small_parts_placed:
    print(f"  - {p['label']:12s}: {p['w']:4d} x {p['h']:4d} = {p['area']:6,d} mm²")
print(f"  소형 단품 총 면적: {total_small_area:,.0f} mm²")
print(f"  노치 면적 대비 충전율: {total_small_area/part_700_22_placed['notch']['area']*100:.1f}%")

# Check if all small parts are within the notch area
notch = part_700_22_placed['notch']
all_in_notch = True
for p in small_parts_placed:
    bx0, by0, bx1, by1 = p['bbox']
    in_x = bx0 >= notch['x_min'] - 5 and bx1 <= notch['x_max'] + 25  # 22mm right margin
    in_y = by0 >= notch['y_min'] - 5 and by1 <= notch['y_max'] + 5
    if not (in_x and in_y):
        all_in_notch = False
        print(f"  [!] {p['label']} is outside notch area")

print(f"\n[핵심 발견]")
print(f"  1. 대형 L형 단품(700-22)을 잔재 하단에 먼저 배치")
print(f"  2. 소형 단품 {len(small_parts_placed)}개 전원이 700-22의 노치(오목부) 내부에 배치됨")
print(f"  3. 전체 배치 패턴: 'L-in-L 노치 채우기' 방식")
print(f"  4. 잔재 상단 영역(~735x582mm)은 사용하지 않음")
print(f"\n[현재 엔진 한계]")
print(f"  - 단품을 직사각 바운딩박스(w x h)로 취급")
print(f"  - 700-22를 1008x1218 직사각으로 간주하면 노치 영역에 다른 단품 배치 불가")
print(f"  - 해결 방안: L형 단품 배치 후 노치를 가상 서브컨테이너로 분할하여 소형 단품 배치")
print("=" * 70)
