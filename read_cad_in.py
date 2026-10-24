import json

with open('mbb_nesting_in.json') as f:
    d_in = json.load(f)

print('=== INPUT JSON ===')
print('Mode:', d_in.get('mode'))
print('Gap:', d_in.get('gap'), 'Margin:', d_in.get('margin'))
print('Containers count:', len(d_in['containers']))
for i, c in enumerate(d_in['containers']):
    print(f'Container {i}: type={c.get("type")}, area={c.get("area")}, bbox={c.get("outer_bbox")}')
    if 'outer_polygon' in c:
        print(f'  outer_polygon points count: {len(c["outer_polygon"])}')

print('Parts count:', len(d_in['parts']))
for i, p in enumerate(d_in['parts']):
    print(f'Part {i}: id={p.get("id")}, orig=({p.get("orig_min_x")}, {p.get("orig_min_y")}), size=({p.get("w")}x{p.get("h")}), has_poly={"polygon" in p}')
    if 'polygon' in p:
        print(f'  polygon points count: {len(p["polygon"])}, pts: {p["polygon"][:3]}...')

print('\n=== OUTPUT JSON ===')
with open('mbb_nesting_out.json') as f:
    d_out = json.load(f)

print('Status:', d_out.get('status'))
print('Mode:', d_out.get('mode'))
print('Placed count:', d_out.get('placed_count'))
print('Unplaced count:', d_out.get('unplaced_count'))
print('Plan:')
for p in d_out.get('plan', []):
    print(' ', p)
print('Unplaced plan:')
for u in d_out.get('unplaced_plan', []):
    print(' ', u)
