# -*- coding: utf-8 -*-
"""Test with ACTUAL captured JSON from AutoCAD"""
import json, sys, os

path = r'C:\Users\like\AppData\Local\Temp\mbb_nesting_in.json'
with open(path, 'r', encoding='utf-8') as f:
    raw = f.read()

# Apply the same fix as mbb_nesting.py
raw = raw.replace(', .', ', 0.').replace('[.', '[0.').replace('-.', '-0.')

data = json.loads(raw)
print("JSON parsed OK!")

c = data['containers'][0]
pts = c['outer_polygon']
print(f"Container: {len(pts)} pts, area={c['area']}")
bbox = c['outer_bbox']
print(f"  BBOX: W={bbox[2]-bbox[0]:.1f}, H={bbox[3]-bbox[1]:.1f}")

for p in data['parts']:
    poly = p.get('polygon', [])
    n = len(poly)
    if n >= 3:
        sa = sum(poly[i][0]*poly[(i+1)%n][1] - poly[(i+1)%n][0]*poly[i][1] for i in range(n))
        pa = abs(sa)*0.5
    else:
        pa = p['w']*p['h']
    print(f"Part {p['id']}: w={p['w']}, h={p['h']}, BBOX_area={p['w']*p['h']:.0f}, poly_area={pa:.0f}, poly_pts={n}")

# Run the actual nesting engine
print("\n=== Running actual nesting ===")
sys.path.insert(0, r"c:\Users\like\OneDrive\문서\MBB")

lsp_out = os.path.join(os.path.dirname(path), "test_out.lsp")
json_out = os.path.join(os.path.dirname(path), "test_out.json")

# Write fixed JSON
fixed_path = os.path.join(os.path.dirname(path), "mbb_nesting_in_fixed.json")
with open(fixed_path, 'w') as f:
    f.write(raw)

import mbb_nesting
mbb_nesting.solve_nesting(fixed_path, lsp_out, json_out, mode='fast')
