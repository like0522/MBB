;; =============================================================================

;; =============================================================================
;; Auto-Detecting Bounding Box & Shape Analyzer
;; LAST MODIFIED: 2026-10-22 18:54:30
;; ENCODING: ANSI (CP949)
;;
;; [VERSION HISTORY & ROLLBACK LOG]
;; - v1.0 (2026-04-06 00:00:00) : 최초 MBB 바운딩 박스 & 형상 분석기 안정화
;; - v1.1 (2026-09-12 13:48:40) : ExtractPoints 무손실 추출 및 직교 스냅 필터 적용
;; - v1.2 (2026-10-22 18:41:30) : MBBG 신규 추가 (문자 제외 순수 기하 기준 외곽 균등 여백 자동 배치)
;; - v1.3 (2026-10-22 18:54:30) : MBBG 외곽 모형 역이동 및 다각형 간섭 방지 최적화 (침범 0mm 보정)
;; =============================================================================
;;
;; [명령어 구성]
;; 1. MBB   : 전체 형상 단일 최소사각박스 산출 및 중량/수율
;; 2. MBB1  : 개별 부품별 바운딩 박스
;; 3. MBB2  : WCS + 5mm 여유 바운딩 박스
;; 4. MBBC  : 최소 외접원 (Min Bounding Circle)
;; 5. MBBCI : 최대 내접원 (Max Inscribed Circle)
;; 6. MBBW  : 도심(Centroid) 및 순중량 산출
;; 7. PCLOSE: 폴리선 자동 닫기
;; 8. MBBG  : 외곽 틀 내부 균등 여백 자동 배치 (문자 제외 순수 기하 기준)
;; 9. MBBUI : DCL 통합 인터페이스 창 실행
;; =============================================================================

(defun c:MBB ( / sel i obj ename all-objs

                 g-minpt g-maxpt wcs-w wcs-h wcs-area wcs-cen

                 all-hull-pts global-hull best-ang

                 rot-pts rot-min-x rot-max-x rot-min-y rot-max-y 

                 rot-w rot-h rot-area rot-cen

                 final-w final-h final-area final-ang final-cen

                 box-pts thk box-weight net-weight thk-str yield

                 tmp-min tmp-max pts acDoc mspace is-closed test-regions )

    (vl-load-com)

    (setq acDoc (vla-get-ActiveDocument (vlax-get-acad-object)))

    (setq mspace (vla-get-ModelSpace acDoc))

    

    (princ "\n================================================================")

    (princ "\n[MBB] 전체 통합 모드 - 포함 관계 분석 (자동 구멍 인식)")

    (princ "\n================================================================")

    (princ "\n>> 객체 선택 (글자는 자동 제외됨): ")

    

    (if (setq sel (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION"))))

        (progn

            (if (not *MBB_THK*) (setq *MBB_THK* 4.5)) 

            (setq thk (if *MBB_DCL_THK* *MBB_DCL_THK* (getreal (strcat "\n>> 소재 두께(T) 입력 <" (rtos *MBB_THK* 2 1) ">: "))))

            (if thk (setq *MBB_THK* thk) (setq thk *MBB_THK*))

            (setq thk-str (rtos thk 2 1))



            (princ "\n>> 데이터 수집 및 스마트 영역 분석 중...")



            (setq all-objs '() i 0)

            (repeat (sslength sel)

                (setq ename (ssname sel i))

                (setq obj (vlax-ename->vla-object ename))

                (setq all-objs (cons obj all-objs) i (1+ i))

            )

            

            (setq all-hull-pts '() g-minpt nil g-maxpt nil)

            (foreach obj all-objs

                ;; [안정성 보완] BoundingBox 추출 시 발생할 수 있는 에러 제어

                (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))

                    (progn

                        (setq tmp-min (vlax-safearray->list tmp-min) tmp-max (vlax-safearray->list tmp-max))

                        (if (and tmp-min tmp-max)

                            (if (null g-minpt)

                                (setq g-minpt tmp-min g-maxpt tmp-max)

                                (setq g-minpt (mapcar 'min g-minpt tmp-min) g-maxpt (mapcar 'max g-maxpt tmp-max))

                            )

                        )

                    )

                )

                (setq pts (MBB:ExtractPoints obj 200))

                (if pts (setq all-hull-pts (append all-hull-pts (MBB:ConvexHull pts))))

            )



            ;; [안정성 보완] 경계 영역을 성공적으로 가져온 경우에만 후속 연산 진행

            (if (and g-minpt g-maxpt)

                (progn

                    (setq net-weight (MBB:CalcSmartNetWeight all-objs thk mspace))

                    (setq wcs-w (- (car g-maxpt) (car g-minpt)) wcs-h (- (cadr g-maxpt) (cadr g-minpt)) wcs-area (* wcs-w wcs-h))

                    (setq wcs-cen (list (/ (+ (car g-minpt) (car g-maxpt)) 2.0) (/ (+ (cadr g-minpt) (cadr g-maxpt)) 2.0)))



                    (setq global-hull (MBB:ConvexHull all-hull-pts))

                    (setq best-ang (MBB:FindBestAngle global-hull))

                    (setq rot-pts (MBB:RotatePoints global-hull (- best-ang)))

                    

                    (if (and rot-pts (> (length rot-pts) 0))

                        (progn

                            (setq rot-min-x (car (nth 0 rot-pts)) rot-max-x rot-min-x rot-min-y (cadr (nth 0 rot-pts)) rot-max-y rot-min-y)

                            (foreach p rot-pts

                                (setq rot-min-x (min rot-min-x (car p)) rot-max-x (max rot-max-x (car p))

                                      rot-min-y (min rot-min-y (cadr p)) rot-max-y (max rot-max-y (cadr p)))

                            )

                            (setq rot-w (- rot-max-x rot-min-x) rot-h (- rot-max-y rot-min-y) rot-area (* rot-w rot-h))

                            (setq rot-cen (list (/ (+ rot-min-x rot-max-x) 2.0) (/ (+ rot-min-y rot-max-y) 2.0)))

                            (setq rot-cen (MBB:RotatePoint rot-cen '(0 0 0) best-ang))

                        )

                        (setq rot-w wcs-w rot-h wcs-h rot-area wcs-area rot-cen wcs-cen)

                    )



                    (setq final-w wcs-w final-h wcs-h final-area wcs-area final-ang 0.0 final-cen wcs-cen)

                    (if (and (< rot-area (* wcs-area 0.985)) (/= final-ang 0.0))

                        (setq final-w rot-w final-h rot-h final-area rot-area final-ang best-ang final-cen rot-cen)

                    )



                    (setq box-weight (/ (* final-area thk 7.85) 1000000.0))

                    (if (> box-weight 0.0) (setq yield (* (/ net-weight box-weight) 100.0)) (setq yield 0.0))



                    (setq box-pts (MBB:GetBoxPoints final-cen final-w final-h final-ang))

                    

                    ;; 선택한 객체들이 닫힌 영역(Region)을 형성할 수 있는지 확인

                    (setq is-closed nil)

                    (setq test-regions (MBB:ListToRegionList all-objs mspace))

                    (if test-regions

                        (progn

                            (setq is-closed T)

                            (foreach r test-regions (vla-delete r)) 

                        )

                    )



                    ;; 개폐 상태에 따른 조건별 출력 제어

                    (if is-closed

                        (MBB:DrawResult box-pts final-w final-h final-ang box-weight net-weight yield thk-str final-cen)

                        (entmake (append '( (000 . "LWPOLYLINE") (100 . "AcDbEntity") (100 . "AcDbPolyline") (090 . 4) (070 . 1) (062 . 1) ) (mapcar '(lambda (p) (cons 10 p)) box-pts)))

                    )

                    (princ "\n>> MBB (전체) 완료.")

                )

                (princ "\n>> 오류: 선택한 객체들의 경계를 정의할 수 없습니다.")

            )

        )

        (princ "\n>> 선택된 객체가 없습니다. (글자, 치수 등은 제외됨)")

    )

    (princ)

)



;; =============================================================================

;; 2. [MBB1] 개별 경계 사각 (부품별 / 자동 구멍)

;; =============================================================================

(defun c:MBB1 ( / sel i obj ename all-objs clusters parent children

                 minpt maxpt wcs-w wcs-h wcs-area wcs-cen pts hull-pts best-ang 

                 rot-w rot-h rot-area rot-cen final-w final-h final-area final-ang final-cen

                 box-pts thk box-weight net-weight thk-str rot-data yield acDoc mspace )

    (vl-load-com)

    (setq acDoc (vla-get-ActiveDocument (vlax-get-acad-object)))

    (setq mspace (vla-get-ModelSpace acDoc))

    (princ "\n================================================================")

    (princ "\n[MBB1] 개별 부품 모드 - 포함 관계 분석 (자동 구멍 인식)")

    (princ "\n================================================================")

    (princ "\n>> 객체 선택 (글자는 자동 제외됨): ")

    

    (if (setq sel (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION"))))

        (progn

            (if (not *MBB_THK*) (setq *MBB_THK* 4.5)) 

            (setq thk (if *MBB_DCL_THK* *MBB_DCL_THK* (getreal (strcat "\n>> 소재 두께(T) 입력 <" (rtos *MBB_THK* 2 1) ">: "))))

            (if thk (setq *MBB_THK* thk) (setq thk *MBB_THK*))

            (setq thk-str (rtos thk 2 1))



            (setq all-objs '() i 0)

            (repeat (sslength sel)

                (setq ename (ssname sel i) obj (vlax-ename->vla-object ename))

                (setq all-objs (cons obj all-objs) i (1+ i))

            )



            (setq clusters (MBB:ClusterObjects all-objs))

            (princ (strcat "\n>> 총 " (itoa (length clusters)) "개의 부품 세트 처리 중..."))



            (foreach group clusters

                (setq parent (car group) children (cadr group))   

                (setq net-weight (MBB:CalcSmartNetWeight (cons parent children) thk mspace))

                

                ;; [안정성 보완] 부품별 경계 추출 시 예외 처리 적용

                (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list parent 'minpt 'maxpt))))

                    (progn

                        (setq minpt (vlax-safearray->list minpt) maxpt (vlax-safearray->list maxpt))

                        (setq wcs-w (- (car maxpt) (car minpt)) wcs-h (- (cadr maxpt) (cadr minpt)) wcs-area (* wcs-w wcs-h))

                        (setq wcs-cen (list (/ (+ (car minpt) (car maxpt)) 2.0) (/ (+ (cadr minpt) (cadr maxpt)) 2.0)))



                        (setq best-ang 0.0 rot-area 1e308)

                        (if (not (eq (vla-get-objectname parent) "AcDbCircle"))

                            (progn

                                (setq pts (MBB:ExtractPoints parent 200)) 

                                (if (and pts (> (length pts) 2))

                                    (progn

                                        (setq hull-pts (MBB:ConvexHull pts))

                                        (setq best-ang (MBB:FindBestAngle hull-pts))

                                    )

                                )

                            )

                        )



                        (if (not (equal best-ang 0.0 1e-6))

                            (progn

                                (setq rot-data (MBB:GetPreciseRotatedDim parent best-ang))

                                (setq rot-w (nth 0 rot-data) rot-h (nth 1 rot-data) rot-area (nth 2 rot-data) rot-cen (nth 3 rot-data))

                            )

                            (setq rot-area wcs-area) 

                        )



                        (setq final-w wcs-w final-h wcs-h final-area wcs-area final-ang 0.0 final-cen wcs-cen)

                        (if (and (< rot-area (* wcs-area 0.985)) (/= final-ang 0.0)) (setq final-w rot-w final-h rot-h final-area rot-area final-ang best-ang final-cen rot-cen))



                        (setq box-weight (/ (* final-area thk 7.85) 1000000.0))

                        (if (> box-weight 0.0) (setq yield (* (/ net-weight box-weight) 100.0)) (setq yield 0.0))



                        (setq box-pts (MBB:GetBoxPoints final-cen final-w final-h final-ang))

                        (MBB:DrawResult box-pts final-w final-h final-ang box-weight net-weight yield thk-str final-cen)

                    )

                )

            )

            (princ "\n>> MBB1 (개별) 완료.")

        )

        (princ "\n>> 선택된 객체가 없습니다. (글자, 치수 등은 제외됨)")

    )

    (princ)

)



;; =============================================================================

;; 3. [MBB2] 통합 경계 사각 (5mm 여유 + 자동 구멍)

;; =============================================================================

(defun c:MBB2 ( / sel i obj ename all-objs thk thk-str g-minpt g-maxpt tmp-min tmp-max 

                 box-w box-h box-area box-weight net-weight yield p1 p2 p3 p4 box-pts wcs-cen acDoc mspace )

    (vl-load-com)

    (setq acDoc (vla-get-ActiveDocument (vlax-get-acad-object)) mspace (vla-get-ModelSpace acDoc))

    (princ "\n================================================================")

    (princ "\n[MBB2] WCS 기준 5mm 여유폭 모드 - 스마트 구멍 인식")

    (princ "\n================================================================")

    (princ "\n>> 객체 선택 (글자는 자동 제외됨): ")

    

    (if (setq sel (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION"))))

        (progn

            (if (not *MBB_THK*) (setq *MBB_THK* 22.0)) 

            (setq thk (if *MBB_DCL_THK* *MBB_DCL_THK* (getreal (strcat "\n>> 소재 두께(T) 입력 <" (rtos *MBB_THK* 2 1) ">: "))))

            (if thk (setq *MBB_THK* thk) (setq thk *MBB_THK*))

            (setq thk-str (rtos thk 2 1))



            (setq all-objs '() g-minpt nil g-maxpt nil i 0)

            (repeat (sslength sel)

                (setq ename (ssname sel i) obj (vlax-ename->vla-object ename))

                (setq all-objs (cons obj all-objs))

                ;; [안정성 보완] 불량 객체 경계 무시

                (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))

                    (progn

                        (setq tmp-min (vlax-safearray->list tmp-min) tmp-max (vlax-safearray->list tmp-max))

                        (if (null g-minpt)

                            (setq g-minpt tmp-min g-maxpt tmp-max)

                            (setq g-minpt (mapcar 'min g-minpt tmp-min) g-maxpt (mapcar 'max g-maxpt tmp-max))

                        )

                    )

                )

                (setq i (1+ i))

            )



            (if (and g-minpt g-maxpt)

                (progn

                    (princ "\n>> 스마트 영역 및 중량 연산 중...")

                    (setq net-weight (MBB:CalcSmartNetWeight all-objs thk mspace))



                    (setq p1 (list (- (car g-minpt) 5.0) (- (cadr g-minpt) 5.0))  

                          p3 (list (+ (car g-maxpt) 5.0) (+ (cadr g-maxpt) 5.0))  

                          p2 (list (car p3) (cadr p1))                             

                          p4 (list (car p1) (cadr p3))                             

                    )

                    (setq box-w (- (car p3) (car p1)) box-h (- (cadr p3) (cadr p1)) box-area (* box-w box-h))

                    (setq box-weight (/ (* box-area thk 7.85) 1000000.0))

                    (if (> box-weight 0.0) (setq yield (* (/ net-weight box-weight) 100.0)) (setq yield 0.0))



                    (setq box-pts (list p1 p2 p3 p4))

                    (setq wcs-cen (list (/ (+ (car p1) (car p3)) 2.0) (/ (+ (cadr p1) (cadr p3)) 2.0) 0.0))



                    (MBB:DrawResult box-pts box-w box-h 0.0 box-weight net-weight yield thk-str wcs-cen)

                    (princ (strcat "\n>> 통합 MBB2 완료. (수율: " (rtos yield 2 1) "%)"))

                )

                (princ "\n>> 오류: 선택한 객체들의 경계를 정의할 수 없습니다.")

            )

        )

        (princ "\n>> 선택된 객체가 없습니다. (글자, 치수 등은 제외됨)")

    )

    (princ)

)



;; =============================================================================

;; 4. [MBBC] Minimum Bounding Circle (최소 외접원 / 자동 닫기 유지)

;; =============================================================================

(defun c:MBBC ( / sel lst cen rad final-circle i obj closed-cnt )

    (vl-load-com)

    (princ "\n================================================================")

    (princ "\n[MBBC] 최소 외접원 - 자동 보정(닫기) 후 최소 원을 그립니다.")

    (princ "\n================================================================")

    (princ "\n>> 객체 선택 (글자는 자동 제외됨): ")

    

    (if (setq sel (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION"))))

        (progn

            (setq i 0 closed-cnt 0)

            (repeat (sslength sel)

                (setq obj (vlax-ename->vla-object (ssname sel i)))

                (if (MBB:ForceClosePolyline obj) (setq closed-cnt (1+ closed-cnt)))

                (setq i (1+ i))

            )

            (if (> closed-cnt 0) (princ (strcat "\n>> 알림: " (itoa closed-cnt) "개의 열린 객체를 닫았습니다.")))



            (setq lst (LM:ExtractPointsHighPrecision sel))

            

            (if (< (length lst) 2)

                (princ "\n>> 계산할 점이 부족합니다.")

                (progn

                    (princ (strcat "\n>> 추출된 " (itoa (length lst)) "개의 정밀 좌표로 최적 원을 계산합니다..."))

                    

                    (setq final-circle (LM:Welzl lst))

                    

                    (if (and final-circle (listp final-circle) (car final-circle) (cadr final-circle))

                        (progn

                            (setq cen (car final-circle) rad (cadr final-circle))

                            (entmake

                                (list '(0 . "CIRCLE") '(100 . "AcDbEntity") '(100 . "AcDbCircle") (cons 10 cen) (cons 40 rad) '(62 . 1))

                            )

                            (princ (strcat "\n>> 완료: 반지름 " (rtos rad 2 2) "의 최소 외접원 생성."))

                        )

                        (princ "\n>> 원을 계산하는데 실패했습니다. 형태가 유효하지 않습니다.")

                    )

                )

            )

        )

        (princ "\n>> 선택된 객체가 없습니다. (글자, 치수 등은 제외됨)")

    )

    (princ)

)



;; =============================================================================

;; [핵심 엔진] Lee Mac의 완벽 검증된 순수 Welzl 알고리즘 (안정성 보완)

;; =============================================================================

(defun LM:ExtractPointsHighPrecision ( sel / i obj name pts all-pts param end-param bul p1 p2 dist1 dist2 inc d ll ur )

    (setq all-pts '() i 0)

    (repeat (sslength sel)

        (setq obj (vlax-ename->vla-object (ssname sel i)) name (vla-get-objectname obj) i (1+ i))

        (cond

            ((wcmatch name "AcDbPolyline,AcDb2dPolyline")

                ;; [안정성 보완]

                (if (not (vl-catch-all-error-p (setq end-param (vl-catch-all-apply 'vlax-curve-getEndParam (list obj)))))

                    (progn

                        (setq end-param (fix end-param) param 0)

                        (repeat end-param

                            (setq p1 (vlax-curve-getPointAtParam obj param) all-pts (cons (list (car p1) (cadr p1) 0.0) all-pts))

                            (if (not (vl-catch-all-error-p (setq bul (vl-catch-all-apply 'vla-getBulge (list obj param)))))

                                (if (/= bul 0.0)

                                    (progn

                                        (setq dist1 (vlax-curve-getDistAtParam obj param) dist2 (vlax-curve-getDistAtParam obj (1+ param)))

                                        (if (and dist1 dist2 (> dist2 dist1))

                                            (progn

                                                (setq inc (/ (- dist2 dist1) 20.0) d (+ dist1 inc))

                                                (while (< d dist2) (setq p2 (vlax-curve-getPointAtDist obj d) all-pts (cons (list (car p2) (cadr p2) 0.0) all-pts) d (+ d inc)))

                                            )

                                        )

                                    )

                                )

                            )

                            (setq param (1+ param))

                        )

                        (setq p1 (vlax-curve-getEndPoint obj) all-pts (cons (list (car p1) (cadr p1) 0.0) all-pts))

                    )

                )

            )

            ((wcmatch name "AcDbCircle,AcDbArc,AcDbSpline")

                (if (not (vl-catch-all-error-p (setq end-param (vl-catch-all-apply 'vlax-curve-getEndParam (list obj)))))

                    (progn

                        (setq dist1 0.0 dist2 (vlax-curve-getDistAtParam obj end-param))

                        (if (and dist2 (> dist2 0.0))

                            (progn

                                (setq inc (/ dist2 100.0) d 0.0)

                                (repeat 100 (setq p2 (vlax-curve-getPointAtDist obj d) all-pts (cons (list (car p2) (cadr p2) 0.0) all-pts) d (+ d inc)))

                            )

                        )

                    )

                )

            )

            ((wcmatch name "AcDbLine")

                (setq all-pts (cons (vlax-curve-getStartPoint obj) all-pts) all-pts (cons (vlax-curve-getEndPoint obj) all-pts))

            )

            (T 

                (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'll 'ur))))

                    (progn

                        (setq p1 (vlax-safearray->list ll) p2 (vlax-safearray->list ur))

                        (setq all-pts (cons p1 all-pts) all-pts (cons p2 all-pts) all-pts (cons (list (car p1) (cadr p2) 0.0) all-pts) all-pts (cons (list (car p2) (cadr p1) 0.0) all-pts))

                    )

                )

            )

        )

    )

    (LM:UniqueFuzz (mapcar '(lambda (p) (list (car p) (cadr p) 0.0)) all-pts) 0.0001)

)



(defun LM:UniqueFuzz ( lst fuzz / x r )

    (while lst (setq x (car lst) lst (vl-remove-if '(lambda (y) (equal x y fuzz)) (cdr lst)) r (cons x r)))

    (reverse r)

)



(defun LM:Welzl ( p ) (LM:Welzl-0 (LM:Shuffle p) nil))



(defun LM:Welzl-0 ( p b / c )

    (cond

        (   (null p)

            (cond

                (   (= 0 (length b)) '(nil 0.0))

                (   (= 1 (length b))  (list (car b) 0.0))

                (   (= 2 (length b))  (LM:Circle-2P (car b) (cadr b)))

                (   (LM:Circle-3P (car b) (cadr b) (caddr b))   )

            )

        )

        (   (<= (cadr (setq c (LM:Welzl-0 (cdr p) b))) 0.0)

            (LM:Welzl-0 (cdr p) (cons (car p) b))

        )

        (   (LM:InCircle-p c (car p)) c)

        (   (LM:Welzl-0 (cdr p) (cons (car p) b)))

    )

)



(defun LM:InCircle-p ( c p ) (<= (distance (car c) p) (+ (cadr c) 1e-8)))

(defun LM:Circle-2P ( p1 p2 ) (list (mapcar '(lambda ( a b ) (/ (+ a b) 2.0)) p1 p2) (/ (distance p1 p2) 2.0)))

(defun LM:Circle-3P ( p1 p2 p3 / c )

    (if (setq c (LM:CircleCenter p1 p2 p3)) (list c (distance c p1)) (LM:Circle-2P-Max p1 p2 p3))

)

(defun LM:Circle-2P-Max ( p1 p2 p3 / d1 d2 d3 )

    (setq d1 (distance p1 p2) d2 (distance p2 p3) d3 (distance p3 p1))

    (cond ((and (<= d2 d1) (<= d3 d1)) (LM:Circle-2P p1 p2)) ((and (<= d1 d2) (<= d3 d2)) (LM:Circle-2P p2 p3)) ( (LM:Circle-2P p3 p1) ))

)

(defun LM:CircleCenter ( p1 p2 p3 / m1 m2 v1 v2 )

    (setq m1 (mapcar '(lambda ( a b ) (/ (+ a b) 2.0)) p1 p2) m2 (mapcar '(lambda ( a b ) (/ (+ a b) 2.0)) p2 p3)

          v1 (list (- (cadr p1) (cadr p2)) (- (car p2) (car p1)) 0.0) v2 (list (- (cadr p2) (cadr p3)) (- (car p3) (car p2)) 0.0))

    (inters m1 (mapcar '+ m1 v1) m2 (mapcar '+ m2 v2) nil)

)

(defun LM:Shuffle ( l / i j tmp )

    (setq i (length l))

    (while (> i 1)

        (setq j (fix (* i (/ (rem (getvar "cputicks") 1e5) 1e5))) i (1- i) tmp (nth i l))

        (setq l (LM:ReplaceNth i (nth j l) l) l (LM:ReplaceNth j tmp l))

    )

    l

)

(defun LM:ReplaceNth ( n x l / i ) (setq i -1) (mapcar '(lambda ( y ) (if (= n (setq i (1+ i))) x y)) l))



;; =============================================================================

;; 5. [MBBCI] Maximum Inscribed Circle (최대 내접원 / 자동 닫기 유지)

;; =============================================================================

(defun c:MBBCI ( / sel obj minpt maxpt seg-pts result cen rad )

    (vl-load-com)

    (princ "\n================================================================")

    (princ "\n[MBBCI] 최대 내접원 - 자동 보정(닫기) 후 최대 원을 찾습니다.")

    (princ "\n================================================================")

    (princ "\n>> 객체 선택 (글자는 자동 제외됨): ")

    

    (if (setq sel (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION"))))

        (progn

            (setq obj (vlax-ename->vla-object (ssname sel 0)))

            (if (MBB:ForceClosePolyline obj) (princ "\n>> 알림: 선택된 열린 객체를 자동으로 닫았습니다."))



            (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'minpt 'maxpt))))

                (progn

                    (setq minpt (vlax-safearray->list minpt) maxpt (vlax-safearray->list maxpt) minpt (list (car minpt) (cadr minpt) 0.0) maxpt (list (car maxpt) (cadr maxpt) 0.0))

                    

                    (princ "\n>> 형상 정밀 분석 중...")

                    (setq seg-pts (MBBCI:GetSegmentedPoints obj 200))

                    

                    (if seg-pts

                        (progn

                            (setq result (MBBCI:FindMaxIncircle obj seg-pts minpt maxpt))

                            (if result

                                (progn

                                    (setq cen (car result) rad (cadr result))

                                    (entmake (list '(0 . "CIRCLE") '(100 . "AcDbEntity") '(100 . "AcDbCircle") (cons 10 cen) (cons 40 rad) '(62 . 3)))

                                    (princ (strcat "\n>> 완료: 반지름 " (rtos rad 2 2) "의 최적 내접원 생성."))

                                )

                                (princ "\n>> 내부 공간을 특정할 수 없습니다.")

                            )

                        )

                        (princ "\n>> 객체 분석 실패.")

                    )

                )

                (princ "\n>> 오류: 선택한 객체의 경계를 파악할 수 없습니다.")

            )

        )

        (princ "\n>> 선택된 객체가 없습니다. (글자, 치수 등은 제외됨)")

    )

    (princ)

)



;; =============================================================================

;; 6. [MBBW] 도형의 무게 중심(Centroid) 표기 (자동 닫기 유지)

;; =============================================================================

(defun c:MBBW ( / sel i obj acDoc mspace regions reg cen pt minpt maxpt diag rad ptObj circObj closed-cnt )

    (vl-load-com)

    (princ "\n================================================================")

    (princ "\n[MBBW] 무게 중심 - 자동 보정(닫기) 후 무게중심을 표시합니다.")

    (princ "\n================================================================")

    (princ "\n>> 객체 선택 (글자는 자동 제외됨): ")

    

    (if (setq sel (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION"))))

        (progn

            (setq acDoc (vla-get-ActiveDocument (vlax-get-acad-object)) mspace (vla-get-ModelSpace acDoc) i 0 closed-cnt 0)

            (repeat (sslength sel)

                (setq obj (vlax-ename->vla-object (ssname sel i)))

                (if (MBB:ForceClosePolyline obj) (setq closed-cnt (1+ closed-cnt)))



                (setq regions (MBBW:CreateRegion obj mspace))

                (if regions

                    (progn

                        (setq reg (car regions))

                        (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-get-Centroid (list reg))))

                            (progn

                                (setq cen (vlax-safearray->list (vlax-variant-value (vla-get-Centroid reg))) pt (list (car cen) (cadr cen) 0.0))

                                (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'minpt 'maxpt))))

                                    (progn

                                        (setq minpt (vlax-safearray->list minpt) maxpt (vlax-safearray->list maxpt) diag (distance minpt maxpt) rad (* diag 0.02)) 

                                        (if (< rad 0.001) (setq rad 1.0))

                                        (setq ptObj (vla-AddPoint mspace (vlax-3d-point pt))) (vla-put-Color ptObj 1)

                                        (setq circObj (vla-AddCircle mspace (vlax-3d-point pt) rad)) (vla-put-Color circObj 1)

                                    )

                                )

                            )

                        )

                        (vla-Delete reg)

                    )

                )

                (setq i (1+ i))

            )

            (if (> closed-cnt 0) (princ (strcat "\n>> 알림: " (itoa closed-cnt) "개의 열린 객체를 닫았습니다.")))

            (princ "\n>> 무게 중심 표기 완료.")

        )

        (princ "\n>> 선택된 객체가 없습니다. (글자, 치수 등은 제외됨)")

    )

    (princ)

)



;; =============================================================================

;; 7.[PCLOSE] 독립적인 폴리선 닫기 명령어

;; =============================================================================

(defun c:PCLOSE ( / sel i obj closed-cnt )

    (vl-load-com)

    (princ "\n>> PCLOSE - 열린 폴리선 닫기")

    (princ "\n>> 대상 객체 선택: ")

    (if (setq sel (ssget "_:L" '((0 . "LWPOLYLINE,POLYLINE,SPLINE"))))

        (progn

            (setq closed-cnt 0 i 0)

            (repeat (sslength sel)

                (setq obj (vlax-ename->vla-object (ssname sel i)))

                (if (MBB:ForceClosePolyline obj) (setq closed-cnt (1+ closed-cnt)))

                (setq i (1+ i))

            )

            (if (> closed-cnt 0) (princ (strcat "\n>> 완료: 총 " (itoa closed-cnt) "개의 객체를 닫았습니다.")) (princ "\n>> 완료: 선택된 객체들은 이미 모두 닫혀있습니다."))

        )

        (princ "\n>> 선택된 폴리선/스플라인이 없습니다.")

    )

    (princ)

)



;; =========================================================================

;;[자동 닫기 유틸]

;; =========================================================================

(defun MBB:ForceClosePolyline ( obj / name flag )

    (setq name (vla-get-objectname obj) flag nil)

    (cond

        ((wcmatch name "AcDbPolyline,AcDb2dPolyline")

            (if (= (vla-get-Closed obj) :vlax-false) (progn (vla-put-Closed obj :vlax-true) (setq flag T)))

        )

        ((wcmatch name "AcDbSpline")

            (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-get-Closed (list obj))))

                (if (= (vla-get-Closed obj) :vlax-false)

                    (progn (vl-catch-all-apply 'vla-put-Closed (list obj :vlax-true)) (setq flag T))

                )

            )

        )

    )

    flag

)



;; =========================================================================

;; [스마트 중량 계산 & 클러스터 (안정성 보완)]

;; =========================================================================

(defun MBB:CalcSmartNetWeight ( objList thk mspace / regions sorted masterReg tmpReg area0 area1 netArea )

    (setq netArea 0.0 regions (MBB:ListToRegionList objList mspace))

    (if regions

        (progn

            (setq sorted (vl-sort regions 

                '(lambda (a b / areaA areaB)

                    (setq areaA (vl-catch-all-apply 'vla-get-Area (list a)))

                    (setq areaB (vl-catch-all-apply 'vla-get-Area (list b)))

                    (if (vl-catch-all-error-p areaA) (setq areaA 0.0))

                    (if (vl-catch-all-error-p areaB) (setq areaB 0.0))

                    (and (numberp areaA) (numberp areaB) (> areaA areaB))

                ))

                masterReg (car sorted)

            )

            (foreach r (cdr sorted)

                (setq tmpReg (vla-copy masterReg) area0 (vla-get-Area tmpReg))

                (vla-Boolean tmpReg acUnion (vla-copy r))

                (setq area1 (vla-get-Area tmpReg))

                (vla-delete tmpReg)

                (if (equal area0 area1 1e-4) (vla-Boolean masterReg acSubtraction r) (vla-Boolean masterReg acUnion r))

            )

            (setq netArea (vla-get-Area masterReg))

            (vla-delete masterReg)

        )

    )

    (/ (* netArea thk 7.85) 1000000.0)

)

(defun MBB:ClusterObjects ( objList / sorted clusters placed i parent group minP maxP minC maxC tmpArea )

    (setq sorted '() clusters '())

    ;; [안정성 보완] 불량 객체 무시 및 글로벌 변수 누수 차단

    (foreach obj objList

        (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'minP 'maxP))))

            (progn

                (setq tmpArea (vl-catch-all-apply 'vla-get-Area (list obj)))

                (if (vl-catch-all-error-p tmpArea) (setq tmpArea 0.0))

                (setq sorted (cons (list obj tmpArea (vlax-safearray->list minP) (vlax-safearray->list maxP)) sorted))

            )

        )

    )

    (setq sorted (vl-sort sorted '(lambda (a b) (> (cadr a) (cadr b)))))

    (foreach item sorted

        (setq obj (car item) minC (nth 2 item) maxC (nth 3 item) placed nil i 0)

        (while (and (not placed) (< i (length clusters)))

            (setq group (nth i clusters) parent (car group) minP (nth 2 parent) maxP (nth 3 parent))

            (if (and (>= (car minC) (car minP)) (>= (cadr minC) (cadr minP)) (<= (car maxC) (car maxP)) (<= (cadr maxC) (cadr maxP)))

                (progn (setq group (list (car group) (cons obj (cadr group))) clusters (MBB:ReplaceNth i group clusters) placed T))

            )

            (setq i (1+ i))

        )

        (if (not placed) (setq clusters (cons (list item (list)) clusters)))

    )

    (mapcar '(lambda (x) (list (caar x) (cadr x))) (reverse clusters))

)

(defun MBB:ReplaceNth ( n item lst / i )

    (setq i -1)

    (mapcar '(lambda (x) (if (= n (setq i (1+ i))) item x)) lst)

)

(defun MBB:ListToRegionList ( objList mspace / curves regionArr i obj tmp tempObjs res )

    (if (and objList (> (length objList) 0))

        (progn

            (setq curves (vlax-make-safearray vlax-vbObject (cons 0 (1- (length objList)))) tempObjs '() i 0)

            (foreach obj objList (setq tmp (vla-copy obj) tempObjs (cons tmp tempObjs)) (vlax-safearray-put-element curves i tmp) (setq i (1+ i)))

            (setq res '())

            (if (not (vl-catch-all-error-p (setq regionArr (vl-catch-all-apply 'vla-AddRegion (list mspace curves))))) (setq res (vlax-safearray->list (vlax-variant-value regionArr))))

            (foreach obj tempObjs (vla-delete obj))

            res

        )

        nil

    )

)



;; =============================================================================

;;[통합 텍스트 및 사각형 출력 함수]

;; =============================================================================

(defun MBB:DrawResult ( box-pts w h ang box-wt net-wt yield thk-str cen / str txt-h ang-rem L_val W_val )

    (entmake (append '( (000 . "LWPOLYLINE") (100 . "AcDbEntity") (100 . "AcDbPolyline") (090 . 4) (070 . 1) (062 . 1) ) (mapcar '(lambda (p) (cons 10 p)) box-pts)))

    (setq ang-rem (rem ang pi))

    (if (and (> ang-rem (* 0.25 pi)) (< ang-rem (* 0.75 pi))) (setq L_val h W_val w) (setq L_val w W_val h))

    

    (setq str (strcat thk-str "tx" (itoa (fix (+ L_val 0.5))) "x" (itoa (fix (+ W_val 0.5))) "-1EA (N:" (rtos net-wt 2 1) "kg / B:" (rtos box-wt 2 1) "kg / YIELD: " (rtos yield 2 1) "%)"))

    

    (setq txt-h (* (max w h) 0.025))

    (if (< txt-h 0.5) (setq txt-h 0.5))

    (setq p4 (nth 3 box-pts))

    (setq txt-pt (list (car p4) (+ (cadr p4) (* txt-h 0.5)) 0.0))

    (entmake (list '(0 . "MTEXT") '(100 . "AcDbEntity") '(100 . "AcDbMText") (cons 10 txt-pt) (cons 40 txt-h) '(71 . 7) (cons 1 str) '(62 . 4)))

)



;; =============================================================================

;; [안정성 대폭 보완된 포인트 추출 함수]

;; =============================================================================

(defun MBB:ExtractPoints ( obj resolution / objType pts param end-param inc i p has-bulge b-val ll ur p1 p2 p3 p4 ) 

    (setq objType (vla-get-objectname obj) pts '())

    (cond

        ((wcmatch objType "AcDbCircle,AcDbArc,AcDbEllipse,AcDbSpline")

            (setq end-param (vlax-curve-getEndParam obj))

            (if (and end-param (numberp end-param) (> end-param 1e-6))

                (progn

                    (setq inc (/ end-param (float resolution)) i 0.0)

                    (if (<= inc 0.0) (setq inc 1.0))

                    (while (<= i end-param) 

                        (setq p (vlax-curve-getPointAtParam obj i)) 

                        (if p (setq pts (cons (list (car p) (cadr p)) pts)))

                        (setq i (+ i inc))

                    )

                )

            )

        )

        ((wcmatch objType "AcDbPolyline,AcDb2dPolyline")
            ;; 1. 모든 정점 좌표를 vla-get-Coordinates로 100% 무손실 직접 추출
            (if (not (vl-catch-all-error-p (setq p1 (vl-catch-all-apply 'vla-get-Coordinates (list obj)))))
                (progn
                    (setq p2 (vlax-safearray->list (vlax-variant-value p1)))
                    (while p2
                        (setq pts (cons (list (car p2) (cadr p2)) pts))
                        (setq p2 (cddr p2))
                    )
                )
            )
            ;; 2. 곡률(Bulge) 구간 안전 보간
            (setq end-param (vlax-curve-getEndParam obj))
            (if (and end-param (numberp end-param) (> end-param 0.0))
                (progn
                    (setq param 0)
                    (repeat (fix end-param)
                        (if (and (not (vl-catch-all-error-p (setq b-val (vl-catch-all-apply 'vla-getBulge (list obj param)))))
                                 b-val (/= 0.0 b-val))
                            (if (and (not (vl-catch-all-error-p (setq ll (vl-catch-all-apply 'vlax-curve-getDistAtParam (list obj param)))))
                                     (not (vl-catch-all-error-p (setq ur (vl-catch-all-apply 'vlax-curve-getDistAtParam (list obj (1+ param))))))
                                     ll ur (> ur ll))
                                (progn
                                    (setq inc (/ (- ur ll) 20.0))
                                    (setq i (+ ll inc))
                                    (while (< i ur)
                                        (if (not (vl-catch-all-error-p (setq p2 (vl-catch-all-apply 'vlax-curve-getPointAtDist (list obj i)))))
                                            (if p2 (setq pts (cons (list (car p2) (cadr p2)) pts)))
                                        )
                                        (setq i (+ i inc))
                                    )
                                )
                            )
                        )
                        (setq param (1+ param))
                    )
                )
            )
            ;; 3. BoundingBox 4개 극점을 안전 가드로 추가하여 박스가 모형을 파고들지 않도록 방어
            (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'll 'ur))))
                (progn
                    (setq p1 (vlax-safearray->list ll) p3 (vlax-safearray->list ur))
                    (setq pts (cons (list (car p1) (cadr p1))
                              (cons (list (car p3) (cadr p1))
                              (cons (list (car p3) (cadr p3))
                              (cons (list (car p1) (cadr p3)) pts)))))
                )
            )
        )
        ((eq objType "AcDbLine")

             (setq pts (mapcar '(lambda (p) (list (car p) (cadr p))) (list (vlax-curve-getStartPoint obj) (vlax-curve-getEndPoint obj))))

        )

        (t

            (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'll 'ur))))

                (progn

                    (setq p1 (vlax-safearray->list ll) p3 (vlax-safearray->list ur) p2 (list (car p3) (cadr p1)) p4 (list (car p1) (cadr p3)))

                    (setq pts (list (list (car p1) (cadr p1)) (list (car p2) (cadr p2)) (list (car p3) (cadr p3)) (list (car p4) (cadr p4))))

                )

            )

        )

    )

    (reverse pts)

)



(defun MBB:ConvexHull ( pts / sorted lower upper )

    (setq sorted (vl-sort pts '(lambda (a b) (if (equal (car a) (car b) 1e-8) (< (cadr a) (cadr b)) (< (car a) (car b))))))

    (setq lower '())

    (foreach p sorted (while (and (>= (length lower) 2) (<= (MBB:CrossProduct (cadr lower) (car lower) p) 0)) (setq lower (cdr lower))) (setq lower (cons p lower)))

    (setq upper '())

    (foreach p (reverse sorted) (while (and (>= (length upper) 2) (<= (MBB:CrossProduct (cadr upper) (car upper) p) 0)) (setq upper (cdr upper))) (setq upper (cons p upper)))

    (append (reverse (cdr lower)) (reverse (cdr upper)))

)

(defun MBB:CrossProduct ( o a b ) (- (* (- (car a) (car o)) (- (cadr b) (cadr o))) (* (- (cadr a) (cadr o)) (- (car b) (car o)))))

(defun MBB:FindBestAngle ( pts / i p1 p2 ang pts-rot min-x max-x min-y max-y w h area min-area best-ang ang-deg ang-mod )
    (setq min-area 1e308 best-ang 0.0 i 0)
    (repeat (length pts)
        (setq p1 (nth i pts) p2 (nth (rem (1+ i) (length pts)) pts) ang (angle p1 p2))
        ;; 직교 축(0, 90, 180, 270도)과 3도(약 0.052 rad) 이내의 미세한 각도는 0도로 스냅
        (setq ang-deg (* (/ ang pi) 180.0))
        (setq ang-mod (rem ang-deg 90.0))
        (if (or (< ang-mod 3.0) (> ang-mod 87.0))
            (setq ang 0.0)
        )
        (setq pts-rot (MBB:RotatePoints pts (- ang)))
        (setq min-x (car (nth 0 pts-rot)) max-x min-x min-y (cadr (nth 0 pts-rot)) max-y min-y)
        (foreach p pts-rot (setq min-x (min min-x (car p)) max-x (max max-x (car p)) min-y (min min-y (cadr p)) max-y (max max-y (cadr p))))
        (setq w (- max-x min-x) h (- max-y min-y) area (* w h))
        (if (< area min-area) (setq min-area area best-ang ang))
        (setq i (1+ i))
    )
    ;; 최종 best-ang도 직교 스냅 재확인
    (setq ang-deg (* (/ best-ang pi) 180.0))
    (setq ang-mod (rem ang-deg 90.0))
    (if (or (< ang-mod 3.0) (> ang-mod 87.0))
        (setq best-ang 0.0)
    )
    best-ang
)

(defun MBB:RotatePoints ( pts ang / cos-a sin-a x y )

    (setq cos-a (cos ang) sin-a (sin ang))

    (mapcar '(lambda (p) (list (- (* (car p) cos-a) (* (cadr p) sin-a)) (+ (* (car p) sin-a) (* (cadr p) cos-a)))) pts)

)

(defun MBB:RotatePoint ( pt base ang / x y bx by cos-a sin-a nx ny )

    (setq bx (car base) by (cadr base) cos-a (cos ang) sin-a (sin ang))

    (setq nx (+ bx (- (* (- (car pt) bx) cos-a) (* (- (cadr pt) by) sin-a))) ny (+ by (+ (* (- (car pt) bx) sin-a) (* (- (cadr pt) by) cos-a))))

    (list nx ny 0.0)

)

(defun MBB:GetBoxPoints ( cen w h ang / hw hh p1 p2 p3 p4 )

    (setq hw (/ w 2.0) hh (/ h 2.0) p1 (list (- hw) (- hh)) p2 (list hw (- hh)) p3 (list hw hh) p4 (list (- hw) hh))

    (mapcar '(lambda (p) (MBB:RotatePoint (list (+ (car cen) (car p)) (+ (cadr cen) (cadr p))) cen ang)) (list p1 p2 p3 p4))

)

(defun MBB:GetPreciseRotatedDim ( obj ang / copyObj minpt maxpt w h cen temp-cen final-cen )

    (setq copyObj (vla-copy obj))

    (vla-getboundingbox obj 'minpt 'maxpt)

    (setq cen (list (/ (+ (vlax-safearray-get-element minpt 0) (vlax-safearray-get-element maxpt 0)) 2.0) (/ (+ (vlax-safearray-get-element minpt 1) (vlax-safearray-get-element maxpt 1)) 2.0) 0.0))

    (vla-rotate copyObj (vlax-3d-point cen) (- ang))

    (vla-getboundingbox copyObj 'minpt 'maxpt)

    (setq minpt (vlax-safearray->list minpt) maxpt (vlax-safearray->list maxpt) w (- (car maxpt) (car minpt)) h (- (cadr maxpt) (cadr minpt)))

    (setq temp-cen (list (/ (+ (car minpt) (car maxpt)) 2.0) (/ (+ (cadr minpt) (cadr maxpt)) 2.0) 0.0) final-cen (MBB:RotatePoint temp-cen cen ang))

    (vla-delete copyObj)

    (list w h (* w h) final-cen)

)



;; =========================================================================

;; [MBBCI 보조 함수] (안정성 보완)

;; =========================================================================

(defun MBBCI:FindMaxIncircle ( obj seg-pts p1 p2 / box-w box-h dx dy x y pt dist max-dist best-pt resolution )

    (setq box-w (- (car p2) (car p1)) box-h (- (cadr p2) (cadr p1)))

    ;; [안정성 보완] Division by zero 예방 (폭이나 높이가 0에 가까우면 중단)

    (if (or (<= box-w 1e-6) (<= box-h 1e-6))

        nil

        (progn

            (setq max-dist 0.0 best-pt nil resolution 30.0 dx (/ box-w resolution) dy (/ box-h resolution) x (car p1))

            (repeat (fix (+ resolution 1))

                (setq y (cadr p1))

                (repeat (fix (+ resolution 1))

                    (setq pt (list x y 0.0))

                    (if (MBBCI:IsPointInside pt seg-pts)

                        (progn

                            (setq dist (distance pt (vlax-curve-getClosestPointTo obj pt)))

                            (if (> dist max-dist) (setq max-dist dist best-pt pt))

                        )

                    )

                    (setq y (+ y dy))

                )

                (setq x (+ x dx))

            )

            (if best-pt (MBBCI:HillClimb obj seg-pts best-pt max-dist) nil)

        )

    )

)

(defun MBBCI:HillClimb ( obj seg-pts startPt startDist / bestPt maxDist step minStep dirs checkPt checkDist improved )

    (setq bestPt startPt maxDist startDist step (/ maxDist 4.0) minStep 0.0001 dirs '((1 0) (-1 0) (0 1) (0 -1) (0.7 0.7) (-0.7 0.7) (-0.7 -0.7) (0.7 -0.7)))

    (while (> step minStep)

        (setq improved nil)

        (foreach d dirs

            (setq checkPt (list (+ (car bestPt) (* (car d) step)) (+ (cadr bestPt) (* (cadr d) step)) 0.0))

            (if (MBBCI:IsPointInside checkPt seg-pts)

                (progn

                    (setq checkDist (distance checkPt (vlax-curve-getClosestPointTo obj checkPt)))

                    (if (> checkDist maxDist) (setq maxDist checkDist bestPt checkPt improved T))

                )

            )

        )

        (if (not improved) (setq step (* step 0.5)))

    )

    (list bestPt maxDist)

)

(defun MBBCI:GetSegmentedPoints ( obj n / len inc i pt pts )

    (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vlax-curve-getEndParam (list obj))))

        (progn

            (setq len (vlax-curve-getDistAtParam obj (vlax-curve-getEndParam obj)))

            (setq inc (/ len n) i 0 pts '())

            (repeat n (setq pt (vlax-curve-getPointAtDist obj (* i inc)) pts (cons (list (car pt) (cadr pt)) pts) i (1+ i)))

            (reverse pts)

        )

        nil

    )

)

(defun MBBCI:IsPointInside ( pt pts / i j p1 p2 c x y )

    (setq x (car pt) y (cadr pt) c nil i 0 j (1- (length pts)))

    (repeat (length pts)

        (setq p1 (nth i pts) p2 (nth j pts))

        (if (and (or (and (> (cadr p1) y) (not (> (cadr p2) y))) (and (not (> (cadr p1) y)) (> (cadr p2) y)))

                 (< x (+ (car p1) (* (/ (- (car p2) (car p1)) (- (cadr p2) (cadr p1))) (- y (cadr p1))))))

            (setq c (not c))

        )

        (setq j i i (1+ i))

    )

    c

)



;; =========================================================================

;; [MBBW 보조 함수]

;; =========================================================================

(defun MBBW:CreateRegion ( obj mspace / curves regionArr )

    (setq curves (vlax-make-safearray vlax-vbObject '(0 . 0)))

    (vlax-safearray-put-element curves 0 obj)

    (if (not (vl-catch-all-error-p (setq regionArr (vl-catch-all-apply 'vla-AddRegion (list mspace curves)))))

        (vlax-safearray->list (vlax-variant-value regionArr))

        nil

    )

)



;; =============================================================================

;; [명령어 및 사용법 안내]

;; =============================================================================

(princ "\n================================================================================")

(princ "\n               [ 오토캐드 유틸리티 통합 리습 로드 완료 ]")

(princ "\n================================================================================")

(princ "\n  1. MBB    : 전체 객체 통합 최적 재단 규격 및 중량/수율 산출 (자동 구멍)")

(princ "\n              * 열린 도형은 사각 박스만 제도되고 텍스트 표기는 자동 생략됩니다.")

(princ "\n  2. MBB1   : 개별 부품별 최적 재단 규격 및 중량/수율 산출 (자동 구멍)")

(princ "\n  3. MBB2   : WCS 여유폭 5mm 모드 (스마트 구멍 인식)")

(princ "\n  4. MBBC   : 최소 외접원 (Minimum Bounding Circle) 그리기")

(princ "\n  5. MBBCI  : 최대 내접원 (Maximum Inscribed Circle) 그리기")

(princ "\n  6. MBBW   : 도형의 무게 중심 (Centroid) 표시")

(princ "\n  7. PCLOSE : 폴리선 닫기
  8. MBBG   : 외곽 틀 내부 균등 여백 자동 배치 (문자 제외 순수 기하 기준)
  9. MBBUI  : 
  8. MBBUI  : DCL 통합 관리 창 실행선택한 폴리선/스플라인 강제 닫기 (Close)")

(princ "\n--------------------------------------------------------------------------------")

(princ "\n  [특징] 객체 선택 시 텍스트(글자), 치수, 해치 등은 자동으로 무시됩니다.")

(princ "\n  [Auto-Fix] MBBC, MBBCI, MBBW 실행 시 열린 객체를 자동으로 닫은 후 작업을 수행합니다.")

(princ "\n================================================================================")

(princ)



;; =============================================================================
;; [MBBUI] 단일 파일 내장 인라인 DCL 대화상자 인터페이스
;; =============================================================================

;;; =============================================================================
;;; [START: CURRENT ACTIVE - v1.3 MODIFIED (2026-10-22 18:54:30)]
;;; [MBBG] 외곽 모형 역이동 및 다각형 간섭 방지 최적 배치 (Interference-Free Outer Shifting)
;;; - 내부 부품(부품+문자)은 원래 자리에 완전 고정
;;; - 외곽 모형(틀)이 자동으로 이동하여 내부 부품군을 100% 감싸도록 역이동
;;; - 다각형 경사면 침범(돌출) 자동 검출 및 안전 여백 최적화 (방안 B 완결)
;;; - 문자는 내부 배치 여백(BBOX) 계산에서 완전 배제
;;; =============================================================================

;; 임의 Y 높이에서 다각형의 좌우 X 경계 산출 헬퍼 함수
(defun MBBG:GetXRangeAtY ( poly y / xs i n p1 p2 y1 y2 x1 x2 t-val )
  (setq xs '() n (length poly) i 0)
  (repeat n
    (setq p1 (nth i poly)
          p2 (nth (rem (1+ i) n) poly)
          y1 (cadr p1)
          y2 (cadr p2)
          x1 (car p1)
          x2 (car p2)
          i  (1+ i))
    (if (and (<= (min y1 y2) y) (<= y (max y1 y2)))
      (if (equal y1 y2 1e-6)
        (setq xs (cons x1 (cons x2 xs)))
        (progn
          (setq t-val (/ (- y y1) (- y2 y1)))
          (setq xs (cons (+ x1 (* t-val (- x2 x1))) xs))
        )
      )
    )
  )
  (if (>= (length xs) 2)
    (list (apply 'min xs) (apply 'max xs))
    nil
  )
)

(defun c:MBBG ( / *error* ss-out ss-in acDoc mspace i ename obj objName
                  out-min out-max in-min in-max tmp-min tmp-max
                  out-cx out-cy in-cx in-cy dx dy
                  out-pts in-pts move-out-objs p1 p2
                  temp-poly max-over-r max-over-l min-clear-r min-clear-l
                  x-rng xl xr cl cr corr-x final-dx final-dy
                  all-inside final-min-clear )
  (vl-load-com)
  (setq acDoc (vla-get-ActiveDocument (vlax-get-acad-object)))
  (setq mspace (vla-get-ModelSpace acDoc))

  ;; 에러 핸들러 및 Undo 트랜잭션 방어
  (defun *error* ( msg )
    (if (and acDoc (= 8 (logand 8 (getvar 'UNDOCTL))))
      (vl-catch-all-apply 'vla-EndUndoMark (list acDoc))
    )
    (if (and msg (not (wcmatch (strcase msg t) "*break*,*cancel*,*exit*")))
      (princ (strcat "
>> [MBBG 오류]: " msg))
    )
    (setvar 'CMDECHO 1)
    (princ)
  )

  (setvar 'CMDECHO 0)
  (vla-StartUndoMark acDoc)

  (princ "
================================================================")
  (princ "
[MBBG] 외곽 모형 역이동 및 간섭 방지 최적 배치 (문자 제외 순수 기하)")
  (princ "
================================================================")

  ;; 1단계: 외곽 기준 모형 선택
  (princ "
>> [1단계] 이동시킬 외곽 모형(컨테이너 틀) 선택: ")
  (if (setq ss-out (ssget ":S:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION"))))
    (progn
      (setq out-min nil out-max nil out-pts '() move-out-objs '() i 0)
      (repeat (sslength ss-out)
        (setq ename (ssname ss-out i)
              obj   (vlax-ename->vla-object ename)
              move-out-objs (cons obj move-out-objs)
              i     (1+ i))
        (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))
          (progn
            (setq tmp-min (vlax-safearray->list tmp-min)
                  tmp-max (vlax-safearray->list tmp-max))
            (setq out-min (if (null out-min) tmp-min (mapcar 'min out-min tmp-min))
                  out-max (if (null out-max) tmp-max (mapcar 'max out-max tmp-max)))
          )
        )
        ;; 외곽 폴리선 정점 추출
        (if (not (vl-catch-all-error-p (setq p1 (vl-catch-all-apply 'vla-get-Coordinates (list obj)))))
          (progn
            (setq p2 (vlax-safearray->list (vlax-variant-value p1)))
            (while p2
              (setq out-pts (cons (list (car p2) (cadr p2)) out-pts))
              (setq p2 (cddr p2))
            )
          )
        )
      )
      (setq out-pts (reverse out-pts))

      (if (and out-min out-max)
        (progn
          (setq out-cx (/ (+ (car out-min) (car out-max)) 2.0)
                out-cy (/ (+ (cadr out-min) (cadr out-max)) 2.0))

          ;; 2단계: 내부에 수용될 부품 및 문자 객체들 선택 (고정 대상)
          (princ "
>> [2단계] 고정시킬 내부 부품 및 문자 객체들 모두 선택: ")
          (if (setq ss-in (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION,TEXT,MTEXT,INSERT"))))
            (progn
              (setq in-min nil in-max nil in-pts '() i 0)
              (repeat (sslength ss-in)
                (setq ename (ssname ss-in i)
                      obj   (vlax-ename->vla-object ename)
                      objName (vla-get-objectname obj)
                      i     (1+ i))
                ;; [규칙]: 문자는 BBOX 여백 계산에서 배제하고 순수 기하 객체만 측정
                (if (not (wcmatch (strcase objName) "*TEXT*"))
                  (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))
                    (progn
                      (setq tmp-min (vlax-safearray->list tmp-min)
                            tmp-max (vlax-safearray->list tmp-max))
                      (setq in-min (if (null in-min) tmp-min (mapcar 'min in-min tmp-min))
                            in-max (if (null in-max) tmp-max (mapcar 'max in-max tmp-max)))
                      ;; 부품의 4개 꼭짓점 코너를 검사점에 추가
                      (setq in-pts (cons (list (car tmp-min) (cadr tmp-min))
                                   (cons (list (car tmp-max) (cadr tmp-min))
                                   (cons (list (car tmp-max) (cadr tmp-max))
                                   (cons (list (car tmp-min) (cadr tmp-max)) in-pts)))))
                    )
                  )
                )
              )

              (if (and in-min in-max out-min out-max)
                (progn
                  (setq in-cx (/ (+ (car in-min) (car in-max)) 2.0)
                        in-cy (/ (+ (cadr in-min) (cadr in-max)) 2.0))

                  ;; 1차 이동 벡터: 내부 부품 중심에 외곽 모형 중심 맞춤
                  (setq dx (- in-cx out-cx)
                        dy (- in-cy out-cy))

                  ;; 2차 다각형 간섭 검출 및 오프셋 보정 (방안 B)
                  (if (and out-pts (>= (length out-pts) 3))
                    (progn
                      (setq temp-poly (mapcar '(lambda (p) (list (+ (car p) dx) (+ (cadr p) dy))) out-pts))
                      (setq max-over-r 0.0 max-over-l 0.0 min-clear-r 1e9 min-clear-l 1e9)
                      (foreach pt in-pts
                        (setq x-rng (MBBG:GetXRangeAtY temp-poly (cadr pt)))
                        (if x-rng
                          (progn
                            (setq xl (car x-rng) xr (cadr x-rng))
                            (setq cl (- (car pt) xl) cr (- xr (car pt)))
                            (if (< cr 0.0) (setq max-over-r (max max-over-r (- cr))))
                            (if (< cl 0.0) (setq max-over-l (max max-over-l (- cl))))
                            (setq min-clear-l (min min-clear-l cl)
                                  min-clear-r (min min-clear-r cr))
                          )
                        )
                      )
                      ;; 침범량 자동 보정량 계산
                      (if (or (> max-over-r 0.0) (> max-over-l 0.0))
                        (if (> max-over-r max-over-l)
                          (setq corr-x (+ max-over-r (/ (- min-clear-l max-over-r) 2.0)))
                          (setq corr-x (- (+ max-over-l (/ (- min-clear-r max-over-l) 2.0))))
                        )
                        (setq corr-x (/ (- min-clear-l min-clear-r) 2.0))
                      )
                    )
                    (setq corr-x 0.0)
                  )

                  (setq final-dx (+ dx corr-x)
                        final-dy dy)

                  ;; [핵심]: 내부 부품은 고정하고, 외곽 모형(틀) 객체들만 역이동!
                  (foreach o move-out-objs
                    (vl-catch-all-apply 'vla-Move (list o (vlax-3d-point '(0.0 0.0 0.0)) (vlax-3d-point (list final-dx final-dy 0.0))))
                  )

                  ;; 최종 간섭 여유 확인
                  (if (and out-pts (>= (length out-pts) 3))
                    (progn
                      (setq temp-poly (mapcar '(lambda (p) (list (+ (car p) final-dx) (+ (cadr p) final-dy))) out-pts))
                      (setq all-inside T final-min-clear 1e9)
                      (foreach pt in-pts
                        (setq x-rng (MBBG:GetXRangeAtY temp-poly (cadr pt)))
                        (if x-rng
                          (progn
                            (setq cl (- (car pt) (car x-rng)) cr (- (cadr x-rng) (car pt)))
                            (setq final-min-clear (min final-min-clear cl cr))
                            (if (or (< cl 0.0) (< cr 0.0)) (setq all-inside nil))
                          )
                        )
                      )
                    )
                  )

                  (princ (strcat "
>> [MBBG 완료] 외곽 모형 역이동 배치 성공!"))
                  (princ (strcat "
   - 외곽 모형 이동량: X = " (rtos final-dx 2 1) " mm, Y = " (rtos final-dy 2 1) " mm"))
                  (if (and out-pts final-min-clear (< final-min-clear 1e8))
                    (princ (strcat "
   - 내부 최소 안전 여백(간섭 제로): " (rtos final-min-clear 2 1) " mm"))
                  )
                  (princ "
   - 내부 부품 및 문자는 원래 위치에 안전하게 고정되었습니다.")
                )
                (princ "
>> 오류: 내부 객체 중 기하 부품을 측정할 수 없습니다.")
              )
            )
            (princ "
>> 선택 취소: 내부에 배치할 객체가 선택되지 않았습니다.")
          )
        )
        (princ "
>> 오류: 외곽 기준 모형의 경계를 측정할 수 없습니다.")
      )
    )
    (princ "
>> 선택 취소: 외곽 기준 모형이 선택되지 않았습니다.")
  )

  ;; 메모리 자원 해제
  (setq ss-out nil ss-in nil move-out-objs nil out-pts nil in-pts nil)
  (vla-EndUndoMark acDoc)
  (setvar 'CMDECHO 1)
  (princ)
)
;;; [END: CURRENT ACTIVE - v1.3 MODIFIED (2026-10-22 18:54:30)]]


(defun c:MBBUI ( / dcl_file fp dcl_id action thk_val )
  (setq dcl_file (vl-filename-mktemp "mbb.dcl"))
  (setq fp (open dcl_file "w"))
  (write-line "mbb_dialog : dialog {" fp)
  (write-line "  label = \"GEOMINFO Engineering System\";" fp)
  (write-line "  : boxed_column {" fp)
  (write-line "    label = \"Plate Thickness (T)\";" fp)
  (write-line "    : edit_box {" fp)
  (write-line "      key = \"thk_val\";" fp)
  (write-line "      label = \"Thickness:\";" fp)
  (write-line "      edit_width = 10;" fp)
  (write-line "      value = \"4.5\";" fp)
  (write-line "    }" fp)
  (write-line "  }" fp)
  (write-line "  : boxed_column {" fp)
  (write-line "    label = \"Bounding Box Commands\";" fp)
  (write-line "    : button { key = \"btn_mbb\"; label = \"1. MBB (Smart Net Weight Box)\"; }" fp)
  (write-line "    : button { key = \"btn_mbb1\"; label = \"2. MBB1 (Individual Box)\"; }" fp)
  (write-line "    : button { key = \"btn_mbb2\"; label = \"3. MBB2 (WCS + 5mm Box)\"; }" fp)
  (write-line "  }" fp)
  (write-line "  : boxed_column {" fp)
  (write-line "    label = \"Circle & Center Commands\";" fp)
  (write-line "    : button { key = \"btn_mbbc\"; label = \"4. MBBC (Min Bounding Circle)\"; }" fp)
  (write-line "    : button { key = \"btn_mbbci\"; label = \"5. MBBCI (Max Inscribed Circle)\"; }" fp)
  (write-line "    : button { key = \"btn_mbbw\"; label = \"6. MBBW (Centroid & Weight)\"; }" fp)
  (write-line "  }" fp)
  (write-line "  : boxed_column {" fp)
  (write-line "    label = \"Utilities\";" fp)
  (write-line "    : button { key = \"btn_pclose\"; label = \"7. PCLOSE (Close Polylines)\"; }
  }
  : boxed_column {
    label = \"Alignment & Placement\";
    : button { key = \"btn_mbbg\"; label = \"8. MBBG (Equal Margin Align)\"; }" fp)
  (write-line "  }" fp)
  (write-line "  spacer;" fp)
  (write-line "  ok_cancel;" fp)
  (write-line "}" fp)
  (close fp)
  
  (setq dcl_id (load_dialog dcl_file))
  (if (not (new_dialog "mbb_dialog" dcl_id))
    (progn 
      (if (findfile dcl_file) (vl-file-delete dcl_file))
      (princ "
Error: DCL 대화상자를 생성할 수 없습니다.")
      (exit)
    )
  )
  
  (if *MBB_THK*
      (set_tile "thk_val" (rtos *MBB_THK* 2 1))
      (set_tile "thk_val" "4.5")
  )
  
  (action_tile "thk_val" "(setq thk_val $value)")
  (action_tile "btn_mbb" "(done_dialog 1)")
  (action_tile "btn_mbb1" "(done_dialog 2)")
  (action_tile "btn_mbb2" "(done_dialog 3)")
  (action_tile "btn_mbbc" "(done_dialog 4)")
  (action_tile "btn_mbbci" "(done_dialog 5)")
  (action_tile "btn_mbbw" "(done_dialog 6)")
  (action_tile "btn_pclose" "(done_dialog 7)")
  (action_tile "btn_mbbg" "(done_dialog 8)")
  (action_tile "cancel" "(done_dialog 0)")
  
  (setq action (start_dialog))
  (unload_dialog dcl_id)
  (if (findfile dcl_file) (vl-file-delete dcl_file))
  
  (if (and thk_val (/= thk_val ""))
      (setq *MBB_DCL_THK* (atof thk_val))
      (setq *MBB_DCL_THK* nil)
  )
  
  (cond
    ((= action 1) (c:MBB))
    ((= action 2) (c:MBB1))
    ((= action 3) (c:MBB2))
    ((= action 4) (c:MBBC))
    ((= action 5) (c:MBBCI))
    ((= action 6) (c:MBBW))
    ((= action 7) (c:PCLOSE))
    ((= action 8) (c:MBBG))
  )
  (setq *MBB_DCL_THK* nil)
  (princ)
)

;; 한글 입력 오타 방지 별칭 (ㅡㅠㅠㅕㅑ = MBBUI)
(defun c:ㅡㅠㅠㅕㅑ () (c:MBBUI))
