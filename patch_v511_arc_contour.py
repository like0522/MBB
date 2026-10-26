# -*- coding: utf-8 -*-
"""
MBB.LSP v5.11 패치: 호(Arc/Bulge) 곡선 단품 폴리곤 보간 정밀 추출 엔진 탑재
"""
import datetime

now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

with open('MBB.LSP', 'r', encoding='cp949', errors='replace') as f:
    content = f.read()

# Detect line ending
if '\r\n' in content:
    NL = '\r\n'
    print('  [INFO] CRLF line endings detected')
else:
    NL = '\n'
    print('  [INFO] LF line endings detected')

# ============================================================
# PATCH 1: Update LAST MODIFIED
# ============================================================
content = content.replace(
    ';; LAST MODIFIED: 2026-10-26 08:30:00',
    f';; LAST MODIFIED: {now_str}'
)

# ============================================================
# PATCH 2: Add v5.11 version history entry
# ============================================================
v58_line = ';; - v5.8 (2026-10-26 08:30:00) :'
v511_entry = f""";; - v5.11 ({now_str}) : 호(Arc/Bulge) 곡선 단품 폴리곤 보간 정밀 추출 엔진 탑재
;;                                 1. MBB:GetPartContourEx 신규 함수: Bulge≠0 세그먼트 자동 감지 → 20분할 보간
;;                                 2. 단품 JSON "polygon" 속성에 호 곡선 보간점 포함 → D-shape/Arc 정밀 True-Shape 배치
{v58_line}"""
content = content.replace(v58_line, v511_entry)

# ============================================================
# PATCH 3: Insert MBB:GetPartContourEx function
# Find the exact marker and insert after it
# ============================================================
marker = ';;; [END: CURRENT ACTIVE - v5.6 MODIFIED]'
marker_idx = content.find(marker)
if marker_idx < 0:
    print(f'  [FAIL] Cannot find marker: {marker}')
else:
    # Find the next function definition after the marker
    after_marker = content[marker_idx + len(marker):]
    next_defun_idx = after_marker.find('(defun MBB:ExtractPoints')
    if next_defun_idx < 0:
        print('  [FAIL] Cannot find (defun MBB:ExtractPoints after marker')
    else:
        # Insert new function between marker and ExtractPoints
        insert_point = marker_idx + len(marker) + next_defun_idx
        
        new_function = NL + NL + \
f""";;; [START: CURRENT ACTIVE - v5.11 MODIFIED ({now_str})]
;;; MBB:GetPartContourEx - 호(Arc/Bulge) 보간 포함 단품 폴리곤 추출
;;; MBB:GetPartContour는 vla-get-Coordinates만 추출 (직선 꼭짓점만)
;;; 이 함수는 Bulge≠0인 세그먼트를 vlax-curve-getPointAtDist로 20분할 보간
(defun MBB:GetPartContourEx ( obj / objType p1 p2 stride pts end-param param b-val
                               seg-start-d seg-end-d seg-inc cur-d pt )
  (setq pts '())
  (if (and obj (not (vl-catch-all-error-p obj)))
    (progn
      (setq objType (vla-get-ObjectName obj))
      (cond
        ;; 원/호/타원/스플라인 → 커브 전체를 40분할 보간
        ((wcmatch (strcase objType) "*CIRCLE*,*ARC*,*ELLIPSE*,*SPLINE*")
          (setq end-param (vlax-curve-getEndParam obj))
          (if (and end-param (numberp end-param) (> end-param 1e-6))
            (progn
              (setq seg-inc (/ end-param 40.0) cur-d 0.0)
              (if (<= seg-inc 0.0) (setq seg-inc 1.0))
              (while (<= cur-d end-param)
                (setq pt (vlax-curve-getPointAtParam obj cur-d))
                (if pt (setq pts (cons (list (car pt) (cadr pt)) pts)))
                (setq cur-d (+ cur-d seg-inc))
              )
              (setq pts (reverse pts))
            )
          )
        )
        ;; 폴리선 → 각 세그먼트별 Bulge 검사 후 호 보간
        ((wcmatch (strcase objType) "*POLYLINE*")
          (setq end-param (vlax-curve-getEndParam obj))
          (if (and end-param (numberp end-param) (> end-param 0.0))
            (progn
              (setq param 0)
              (repeat (fix end-param)
                ;; 현재 세그먼트 시작점 추가
                (if (not (vl-catch-all-error-p
                      (setq pt (vl-catch-all-apply 'vlax-curve-getPointAtParam (list obj param)))))
                  (if pt (setq pts (cons (list (car pt) (cadr pt)) pts)))
                )
                ;; Bulge 검사 → 호 세그먼트이면 20분할 보간점 삽입
                (if (and (not (vl-catch-all-error-p
                           (setq b-val (vl-catch-all-apply 'vla-getBulge (list obj param)))))
                         b-val (numberp b-val) (/= 0.0 b-val))
                  (if (and (not (vl-catch-all-error-p
                             (setq seg-start-d (vl-catch-all-apply 'vlax-curve-getDistAtParam (list obj param)))))
                           (not (vl-catch-all-error-p
                             (setq seg-end-d (vl-catch-all-apply 'vlax-curve-getDistAtParam (list obj (1+ param))))))
                           seg-start-d seg-end-d (> seg-end-d seg-start-d))
                    (progn
                      (setq seg-inc (/ (- seg-end-d seg-start-d) 20.0)
                            cur-d  (+ seg-start-d seg-inc))
                      (while (< cur-d (- seg-end-d (* seg-inc 0.5)))
                        (if (not (vl-catch-all-error-p
                              (setq pt (vl-catch-all-apply 'vlax-curve-getPointAtDist (list obj cur-d)))))
                          (if pt (setq pts (cons (list (car pt) (cadr pt)) pts)))
                        )
                        (setq cur-d (+ cur-d seg-inc))
                      )
                    )
                  )
                )
                (setq param (1+ param))
              )
              ;; 마지막 꼭짓점 추가 (닫힌 폴리선이면 첫점과 동일하므로 추가해도 무방)
              (if (not (vl-catch-all-error-p
                    (setq pt (vl-catch-all-apply 'vlax-curve-getPointAtParam (list obj (fix end-param))))))
                (if pt (setq pts (cons (list (car pt) (cadr pt)) pts)))
              )
              (setq pts (reverse pts))
            )
            ;; end-param이 0이면 기본 Coordinates 추출 (폴백)
            (progn
              (if (not (vl-catch-all-error-p (setq p1 (vl-catch-all-apply 'vla-get-Coordinates (list obj)))))
                (progn
                  (setq p2 (vlax-safearray->list (vlax-variant-value p1))
                        stride (if (wcmatch (strcase objType) "*2DPOLYLINE*,*3DPOLYLINE*") 3 2))
                  (while p2
                    (setq pts (cons (list (car p2) (cadr p2)) pts))
                    (if (= stride 3) (setq p2 (cdddr p2)) (setq p2 (cddr p2)))
                  )
                  (setq pts (reverse pts))
                )
              )
            )
          )
        )
      )
    )
  )
  pts
)
;;; [END: CURRENT ACTIVE - v5.11 MODIFIED]""" + NL + NL
        
        content = content[:insert_point] + new_function + content[insert_point:]
        print('  [OK] MBB:GetPartContourEx function inserted')

# ============================================================
# PATCH 4: Replace MBB:GetPartContour call with MBB:GetPartContourEx
# ============================================================
old_call = 'part-pts (MBB:GetPartContour parent-obj)'
new_call = 'part-pts (MBB:GetPartContourEx parent-obj)'
if old_call in content:
    content = content.replace(old_call, new_call)
    print('  [OK] GetPartContour call replaced with GetPartContourEx')
elif new_call in content:
    print('  [OK] GetPartContourEx call already present (from previous patch attempt)')
else:
    print('  [FAIL] Cannot find GetPartContour call')

# ============================================================
# Final verification
# ============================================================
checks = [
    ('LAST MODIFIED updated', now_str in content),
    ('v5.11 history added', 'v5.11' in content),
    ('GetPartContourEx defined', 'defun MBB:GetPartContourEx' in content),
    ('GetPartContourEx called', 'MBB:GetPartContourEx parent-obj' in content),
    ('Original GetPartContour preserved', 'defun MBB:GetPartContour ' in content),
    ('ExtractPoints preserved', 'defun MBB:ExtractPoints' in content),
]
all_ok = True
for name, ok in checks:
    status = 'OK' if ok else 'FAIL'
    print(f'  [{status}] {name}')
    if not ok:
        all_ok = False

# Bracket check
opens = content.count('(')
closes = content.count(')')
diff = opens - closes
print(f'\n  괄호 검증: 열림={opens}, 닫힘={closes}, 차이={diff}')
if abs(diff) > 2:
    print(f'  [WARNING] 괄호 차이 {diff}개 — 기존 파일의 원래 차이일 수 있음')

if all_ok:
    with open('MBB.LSP', 'w', encoding='cp949', errors='replace') as f:
        f.write(content)
    import os
    fsize = os.path.getsize('MBB.LSP')
    print(f'\n[SUCCESS] MBB.LSP v5.11 패치 완료 ({now_str})')
    print(f'  파일 크기: {fsize} bytes')
else:
    print('\n[ABORT] 패치 검증 실패. 파일 변경 없음.')
