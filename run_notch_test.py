# -*- coding: utf-8 -*-
import test_l_shape_nesting as tln
import mbb_nesting as mn

plan = mn.solve_single_container_nesting(tln.container, tln.parts, gap=5.0, margin=1.0)
print('Placed count:', len(plan), '/', len(tln.parts))

# 700-22 placed at bottom-left with margin ~1mm
# Notch area of 700-22: x=[orig+358, orig+1008], y=[orig+708, orig+1218]
# Where orig is approximately the placement position
big = next((p for p in plan if p['id'] == 4), None)
if big:
    notch_min_x = big['gx'] + 358.0
    notch_max_x = big['gx'] + big['w']
    notch_min_y = big['gy'] + 708.0
    notch_max_y = big['gy'] + big['h']
    print()
    print('700-22 placed at (%.0f, %.0f) w=%.0f h=%.0f' % (big['gx'], big['gy'], big['w'], big['h']))
    print('Notch area: X=[%.0f, %.0f], Y=[%.0f, %.0f]' % (notch_min_x, notch_max_x, notch_min_y, notch_max_y))
    print()

in_notch = 0
in_upper = 0
for p in plan:
    label = next((t.get('label', str(t['id'])) for t in tln.parts if t['id'] == p['id']), str(p['id']))
    if p['id'] == 4:
        loc = 'BASE'
    elif big and p['gx'] >= notch_min_x - 10 and p['gy'] >= notch_min_y - 10 and p['gy'] + p['h'] <= notch_max_y + 10:
        loc = 'NOTCH'
        in_notch += 1
    else:
        loc = 'UPPER'
        in_upper += 1
    print('  %-12s id=%3d at (%.0f, %.0f) w=%.0f h=%.0f rot=%.0f -> %s' % (label, p['id'], p['gx'], p['gy'], p['w'], p['h'], p['rot'], loc))

print()
print('In notch: %d, In upper: %d' % (in_notch, in_upper))
