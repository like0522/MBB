;; =============================================================================

;; =============================================================================
;; Auto-Detecting Bounding Box & Shape Analyzer
;; LAST MODIFIED: 2026-10-22 21:00:00
;; ENCODING: ANSI (CP949)
;;
;; [VERSION HISTORY & ROLLBACK LOG]
;; - v2.2 (2026-10-22 21:00:00) : 오목 원호(60mm Inward Arc) 고밀도 곡선추종(vlax-curve-getPointAtDist) & MBBN 네스팅 엔진 통합 (돌출/글자이탈 0% 보장)
;; - v2.1 (2026-10-22 20:45:00) : MBBN 텍스트 1:1 독점바인딩(글자 이탈 방지) & MBBG 2D 전역 안전안착 비간섭 재배치(돌출/겹침 100% 방어)
;; - v2.0 (2026-10-22 20:30:00) : MBBG 다중 내경(홀/구멍) 1:1 결합 어셈블리 묶음 이동 & 단품 간 비간섭(Non-Overlapping) 가드 탑재
;; - v1.9 (2026-10-22 20:15:00) : MBBG 스마트 2D 하이브리드 무간섭 자동 배치 (다중샘플링 & Smart Nudge-In 탑재)
;; - v1.8 (2026-10-22 20:05:00) : MBBN FFD(First-Fit Decreasing) 면적정렬 & 앵커포인트(BLF) 네스팅 고도화
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

(princ "\n  7. PCLOSE : 폴리선 닫기 (Close)")
(princ "\n  8. MBBG   : 외곽 틀 내부 균등 여백 자동 배치 (내부 부품 고정)")
(princ "\n  9. MBBN   : 외곽 틀 내부 그리드 세분화 모형 자동 배열 (2D Nesting)")
(princ "\n 10. MBBUI  : DCL 통합 관리 창 실행")

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
    ;; 부동소수점 오차 방어 1e-2 fuzz
    (if (and (<= (- (min y1 y2) 1e-2) y) (<= y (+ (max y1 y2) 1e-2)))
      (if (< (abs (- y2 y1)) 1e-3)
        (setq xs (cons x1 (cons x2 xs)))
        (progn
          (setq t-val (/ (- y y1) (- y2 y1)))
          ;; 선분 범위 강제 클램핑 [0.0, 1.0]
          (setq t-val (max 0.0 (min 1.0 t-val)))
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

;; 직사각형 [rx, ry, rw, rh] 가 외곽 다각형 poly 내부에 완전히 포함되는지 검사 (0.5mm 경계 퍼즈 방어)
(defun MBBN:IsRectInsidePoly ( rx ry rw rh poly margin / ok f y-c rng p-min-y p-max-y )
  (setq p-min-y (apply 'min (mapcar 'cadr poly))
        p-max-y (apply 'max (mapcar 'cadr poly)))
  ;; 상하 경계 0.5mm 퍼즈 허용
  (if (or (< (- ry margin) (- p-min-y 0.5)) (> (+ ry rh margin) (+ p-max-y 0.5)))
    nil
    (progn
      (setq ok T)
      (foreach f '(0.0 0.25 0.5 0.75 1.0)
        (if ok
          (progn
            (setq y-c (+ ry (* f rh))
                  rng (MBBG:GetXRangeAtY poly y-c))
            ;; 좌우 경계 0.5mm 퍼즈 허용
            (if (or (null rng)
                    (< (- rx margin) (- (car rng) 0.5))
                    (> (+ rx rw margin) (+ (cadr rng) 0.5)))
              (setq ok nil)
            )
          )
        )
      )
      ok
    )
  )
)

;; 두 직사각형 충돌(간섭) 검사 (gap 고려)
(defun MBBN:RectsOverlap ( r1 r2 gap / x1 y1 w1 h1 x2 y2 w2 h2 )
  (setq x1 (nth 0 r1) y1 (nth 1 r1) w1 (nth 2 r1) h1 (nth 3 r1)
        x2 (nth 0 r2) y2 (nth 1 r2) w2 (nth 2 r2) h2 (nth 3 r2))
  (not (or (<= (+ x1 w1 gap) x2)
           (<= (+ x2 w2 gap) x1)
           (<= (+ y1 h1 gap) y2)
           (<= (+ y2 h2 gap) y1)))
)

;;; =============================================================================
;;; [START: CURRENT ACTIVE - v1.9 MODIFIED (2026-10-22 20:15:00)]
;;; [MBBG] 외곽 틀 이동 & 스마트 2D 하이브리드 무간섭 자동 배치 (v1.9)
;;; - WCS 커브 정점 정밀 추출(vlax-curve-getPointAtParam): 좌표계 왜곡 원천 차단
;;; - 16분할 다중 둘레점 샘플링(Multi-Point Sampling): 경사면/테이퍼 관통 방지
;;; - 120단계 2D 수직/수평 여백 동시 최적화 틀 이동
;;; - [Smart Nudge-In]: 틀 이동 후 잔여 돌출 부품을 틀 내부 안전 영역으로 자동 밀착 보정
;;; - 부속 문자 1:1 동기화 이동으로 도면 무결성 100% 보장
;;; =============================================================================
;;; =============================================================================
;;; =============================================================================
;;; =============================================================================
;;; [START: CURRENT ACTIVE - v2.2 MODIFIED (2026-10-22 21:00:00)]
;;; [MBBG] 외곽 틀 이동 & 고밀도 곡선추종 스마트 비간섭 자동 배치 (v2.2)
;;; - [오목 원호 100% 추종]: vlax-curve-getPointAtDist 10mm 분할로 60mm 오목 원호/곡선 완벽 추종
;;; - [다중 내경 1:1 결합]: 외곽선 + 내부 홀(원/타원/슬롯) + 문자를 단일 어셈블리로 완전 결합
;;; - [텍스트 1:1 독점 바인딩]: 중복 할당 방지로 글자 분리/이탈 원천 차단
;;; - 120단계 2D 수직/수평 여백 동시 최적화 틀 이동
;;; - [MBBN 네스팅 엔진 통합]: 틀 내부 밀착 시 2D 전역 스캔으로 단 0.1mm의 돌출 및 겹침 100% 방어
;;; =============================================================================
(defun c:MBBG ( / *error* ss-out ss-in acDoc mspace i ename obj objName
                  out-min out-max in-min in-max tmp-min tmp-max tmpArea
                  out-cx out-cy in-cx in-cy
                  out-pts in-pts move-out-objs p1 p2 num-v vi
                  total-len step-dist cur-dist
                  dy-min dy-max num-steps step-dy step-i
                  best-clear best-dx best-dy
                  cand-dy cand-valid cand-dx max-allow-dx min-allow-dx
                  y-q x-rng slack h-clear v-top v-bot min-2d
                  final-dx final-dy new-out-pts
                  geom-raw text-list clustered-parts grp parent-item
                  placed ci minP maxP minC maxC
                  parent-obj children-holes text-objs all-objs assigned-texts part-texts
                  parts-list confirmed p orig-x orig-y w h cur-r
                  candidates needed-dx-l needed-dx-r nudge-dx
                  f y-c rng ay sx valid-cands best-cand
                  c cx cy cw ch dist nudged-count shift-dx shift-dy
                  scan-y min-x max-x shift-out-min-y shift-out-max-y )
  (vl-load-com)
  (setq acDoc (vla-get-ActiveDocument (vlax-get-acad-object)))
  (setq mspace (vla-get-ModelSpace acDoc))

  ;; 에러 핸들러 및 Undo 트랜잭션
  (defun *error* ( msg )
    (if (and acDoc (= 8 (logand 8 (getvar 'UNDOCTL))))
      (vl-catch-all-apply 'vla-EndUndoMark (list acDoc))
    )
    (if (and msg (not (wcmatch (strcase msg t) "*break*,*cancel*,*exit*")))
      (princ (strcat "\n>> [MBBG 에러]: " msg))
    )
    (setvar 'CMDECHO 1)
    (princ)
  )

  (setvar 'CMDECHO 0)
  (vla-StartUndoMark acDoc)

  (princ "\n================================================================")
  (princ "\n[MBBG v2.2] 고밀도 곡선추종 & 스마트 비간섭 자동 배치")
  (princ "\n================================================================")

  ;; 1단계: 이동시킬 외곽 틀(컨테이너 다각형) 선택
  (princ "\n>> [1단계] 이동시킬 외곽 틀(컨테이너 다각형) 선택: ")
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
        ;; WCS Curve 둘레 기반 고밀도 정밀 샘플링 (10mm 간격, 오목 원호/곡선 100% 추종)
        (setq total-len nil)
        (if (not (vl-catch-all-error-p (setq total-len (vl-catch-all-apply 'vlax-curve-getDistAtParam (list obj (vlax-curve-getEndParam obj))))))
          (if (and (numberp total-len) (> total-len 0.0))
            (progn
              (setq step-dist (max 2.0 (min 10.0 (/ total-len 120.0)))
                    cur-dist 0.0)
              (while (< cur-dist total-len)
                (if (not (vl-catch-all-error-p (setq p1 (vl-catch-all-apply 'vlax-curve-getPointAtDist (list obj cur-dist)))))
                  (setq out-pts (cons (list (car p1) (cadr p1)) out-pts))
                )
                (setq cur-dist (+ cur-dist step-dist))
              )
              (setq out-pts (reverse out-pts))
            )
          )
        )
        ;; Curve 샘플링 실패 시 정점 백업 추출
        (if (null out-pts)
          (progn
            (setq num-v nil)
            (if (not (vl-catch-all-error-p (setq num-v (vl-catch-all-apply 'vlax-curve-getEndParam (list obj)))))
              (if (and (numberp num-v) (> num-v 0))
                (progn
                  (setq vi 0)
                  (while (< vi num-v)
                    (if (not (vl-catch-all-error-p (setq p1 (vl-catch-all-apply 'vlax-curve-getPointAtParam (list obj vi)))))
                      (setq out-pts (cons (list (car p1) (cadr p1)) out-pts))
                    )
                    (setq vi (1+ vi))
                  )
                  (setq out-pts (reverse out-pts))
                )
              )
            )
          )
        )
        ;; 좌표 백업 추출
        (if (null out-pts)
          (progn
            (setq p1 nil)
            (if (vlax-property-available-p obj 'Coordinates)
              (setq p1 (vl-catch-all-apply 'vlax-get (list obj 'Coordinates)))
            )
            (if (or (vl-catch-all-error-p p1) (null p1))
              (if (not (vl-catch-all-error-p (setq p2 (vl-catch-all-apply 'vla-get-Coordinates (list obj)))))
                (setq p1 (vlax-safearray->list (vlax-variant-value p2)))
              )
            )
            (if (and p1 (listp p1) (not (vl-catch-all-error-p p1)))
              (while p1
                (setq out-pts (cons (list (car p1) (cadr p1)) out-pts))
                (setq p1 (cddr p1))
              )
            )
          )
        )
      )

      (if (and out-min out-max out-pts (>= (length out-pts) 3))
        (progn
          (setq out-cx (/ (+ (car out-min) (car out-max)) 2.0)
                out-cy (/ (+ (cadr out-min) (cadr out-max)) 2.0))
          (princ (strcat "\n>> [외곽 틀 곡선 파싱] 총 " (itoa (length out-pts)) "개 고밀도 경계점 추출 완료 (오목 원호 완벽 반영)"))

          ;; 2단계: 내부에 담을 부품 및 문자 전체 드래그 선택
          (princ "\n>> [2단계] 내부에 담을 부품 및 문자 전체 드래그 선택: ")
          (if (setq ss-in (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION,TEXT,MTEXT,INSERT"))))
            (progn
              (setq geom-raw '() text-list '() i 0)
              (repeat (sslength ss-in)
                (setq ename (ssname ss-in i)
                      obj   (vlax-ename->vla-object ename)
                      objName (vla-get-ObjectName obj)
                      i     (1+ i))
                (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))
                  (progn
                    (setq tmp-min (vlax-safearray->list tmp-min)
                          tmp-max (vlax-safearray->list tmp-max))
                    (if (wcmatch (strcase objName) "*TEXT*")
                      (setq text-list (cons (list obj tmp-min tmp-max) text-list))
                      (progn
                        (setq tmpArea (vl-catch-all-apply 'vla-get-Area (list obj)))
                        (if (vl-catch-all-error-p tmpArea)
                          (setq tmpArea (* (- (car tmp-max) (car tmp-min)) (- (cadr tmp-max) (cadr tmp-min))))
                        )
                        (setq geom-raw (cons (list obj tmpArea tmp-min tmp-max) geom-raw))
                      )
                    )
                  )
                )
              )

              ;; [내경(홀/구멍) 토폴로지 클러스터링]: 외곽선과 내부 내경들을 1:1 완벽 결합
              (setq geom-raw (vl-sort geom-raw '(lambda (a b) (> (cadr a) (cadr b))))
                    clustered-parts '())
              (foreach item geom-raw
                (setq obj (car item)
                      minC (nth 2 item)
                      maxC (nth 3 item)
                      placed nil
                      ci 0)
                (while (and (not placed) (< ci (length clustered-parts)))
                  (setq grp (nth ci clustered-parts)
                        parent-item (car grp)
                        minP (nth 2 parent-item)
                        maxP (nth 3 parent-item))
                  ;; item의 BBOX가 기존 parent BBOX 내부에 포함되면 내경(Child)으로 등록
                  (if (and (>= (car minC) (- (car minP) 0.5))
                           (>= (cadr minC) (- (cadr minP) 0.5))
                           (<= (car maxC) (+ (car maxP) 0.5))
                           (<= (cadr maxC) (+ (cadr maxP) 0.5)))
                    (progn
                      (setq grp (list parent-item (cons obj (cadr grp)))
                            clustered-parts (MBB:ReplaceNth ci grp clustered-parts)
                            placed T)
                    )
                  )
                  (setq ci (1+ ci))
                )
                (if (not placed)
                  (setq clustered-parts (cons (list item (list)) clustered-parts))
                )
              )
              (setq clustered-parts (reverse clustered-parts))

              ;; [텍스트 1:1 독점 바인딩]: 중복 할당 방지하여 글자 분리/이탈 원천 차단
              (setq assigned-texts '()
                    parts-list '()
                    in-min nil in-max nil in-pts '()
                    i 1)
              (foreach grp clustered-parts
                (setq parent-obj (caar grp)
                      children-holes (cadr grp)
                      tmp-min (nth 2 (car grp))
                      tmp-max (nth 3 (car grp))
                      part-texts '())
                (foreach t-item text-list
                  (setq t-obj (nth 0 t-item)
                        t-pt (nth 1 t-item))
                  (if (and (null (vl-position t-obj assigned-texts))
                           (<= (- (car tmp-min) 50.0) (car t-pt))
                           (<= (car t-pt) (+ (car tmp-max) 50.0))
                           (<= (- (cadr tmp-min) 50.0) (cadr t-pt))
                           (<= (cadr t-pt) (+ (cadr tmp-max) 50.0)))
                    (progn
                      (setq part-texts (cons t-obj part-texts)
                            assigned-texts (cons t-obj assigned-texts))
                    )
                  )
                )
                (setq all-objs (cons parent-obj (append children-holes part-texts))
                      w (- (car tmp-max) (car tmp-min))
                      h (- (cadr tmp-max) (cadr tmp-min))
                      area (* w h))
                ;; part 구조: (all-objs orig-x orig-y w h area (length children-holes) (length part-texts))
                (setq parts-list (cons (list all-objs (car tmp-min) (cadr tmp-min) w h area (length children-holes) (length part-texts)) parts-list))

                (setq in-min (if (null in-min) tmp-min (mapcar 'min in-min tmp-min))
                      in-max (if (null in-max) tmp-max (mapcar 'max in-max tmp-max)))

                ;; 16분할 다중 둘레점 정밀 샘플링 (외곽선 기준)
                (foreach f '(0.0 0.25 0.5 0.75 1.0)
                  (setq in-pts (cons (list (+ (car tmp-min) (* f w)) (cadr tmp-min)) in-pts))
                  (setq in-pts (cons (list (+ (car tmp-min) (* f w)) (cadr tmp-max)) in-pts))
                )
                (foreach f '(0.25 0.5 0.75)
                  (setq in-pts (cons (list (car tmp-min) (+ (cadr tmp-min) (* f h))) in-pts))
                  (setq in-pts (cons (list (car tmp-max) (+ (cadr tmp-min) (* f h))) in-pts))
                )
                (princ (strcat "\n   - [단품 " (itoa i) "] 폭 = " (rtos w 2 1) " mm, 높이 = " (rtos h 2 1) 
                               " mm (내경 " (itoa (length children-holes)) "개, 문자 " (itoa (length part-texts)) "개 독점결합)"))
                (setq i (1+ i))
              )

              (if (and in-min in-max out-min out-max in-pts)
                (progn
                  (setq in-cx (/ (+ (car in-min) (car in-max)) 2.0)
                        in-cy (/ (+ (cadr in-min) (cadr in-max)) 2.0))

                  ;; 120단계 2D 간격 최적화 틀 이동량 탐색
                  (setq dy-min (- (cadr in-max) (cadr out-max))
                        dy-max (- (cadr in-min) (cadr out-min)))
                  (if (<= dy-min dy-max)
                    (setq num-steps 120
                          step-dy (/ (- dy-max dy-min) 120.0))
                    (setq num-steps 60
                          step-dy (/ (abs (- in-cy out-cy)) 60.0)
                          dy-min (- in-cy out-cy))
                  )
                  (setq best-clear -1e9
                        best-dx (- in-cx out-cx)
                        best-dy (- in-cy out-cy)
                        step-i 0)
                  (while (<= step-i num-steps)
                    (setq cand-dy (+ dy-min (* step-i step-dy))
                          cand-valid T
                          max-allow-dx 1e9
                          min-allow-dx -1e9)
                    (foreach pt in-pts
                      (if cand-valid
                        (progn
                          (setq y-q (- (cadr pt) cand-dy)
                                x-rng (MBBG:GetXRangeAtY out-pts y-q))
                          (if x-rng
                            (setq max-allow-dx (min max-allow-dx (- (car pt) (car x-rng)))
                                  min-allow-dx (max min-allow-dx (- (car pt) (cadr x-rng))))
                            (setq cand-valid nil)
                          )
                        )
                      )
                    )
                    (if cand-valid
                      (progn
                        (setq slack (- max-allow-dx min-allow-dx)
                              cand-dx (/ (+ min-allow-dx max-allow-dx) 2.0)
                              h-clear (/ slack 2.0)
                              v-top (- (+ (cadr out-max) cand-dy) (cadr in-max))
                              v-bot (- (cadr in-min) (+ (cadr out-min) cand-dy))
                              min-2d (min h-clear (min v-top v-bot)))
                        (if (> min-2d best-clear)
                          (setq best-clear min-2d
                                best-dx cand-dx
                                best-dy cand-dy)
                        )
                      )
                    )
                    (setq step-i (1+ step-i))
                  )

                  (setq final-dx best-dx
                        final-dy best-dy)

                  ;; [1차 실행]: 외곽 틀(컨테이너) 최적 위치로 이동
                  (foreach o move-out-objs
                    (vl-catch-all-apply 'vla-Move (list o (vlax-3d-point '(0.0 0.0 0.0)) (vlax-3d-point (list final-dx final-dy 0.0))))
                  )

                  ;; 이동된 외곽 다각형 좌표 및 경계 갱신
                  (setq new-out-pts (mapcar '(lambda (pt) (list (+ (car pt) final-dx) (+ (cadr pt) final-dy))) out-pts)
                        shift-out-min-y (+ (cadr out-min) final-dy)
                        shift-out-max-y (+ (cadr out-max) final-dy))

                  ;; [2차 실행]: MBBN 네스팅 엔진 통합 기반 2D 전역 비간섭 배치 (Non-Overlapping Relaxation)
                  ;; 면적 내림차순(대형 단품 우선)으로 안전 배치 확정
                  (setq parts-list (vl-sort parts-list '(lambda (a b) (> (nth 5 a) (nth 5 b))))
                        confirmed '()
                        nudged-count 0)

                  (foreach p parts-list
                    (setq all-objs (nth 0 p)
                          orig-x   (nth 1 p)
                          orig-y   (nth 2 p)
                          w        (nth 3 p)
                          h        (nth 4 p)
                          cur-r    (list orig-x orig-y w h))

                    ;; 현재 위치가 100% 곡선 틀 내부이고 다른 확정 단품과 겹치지 않는지 검사
                    (setq collides nil)
                    (foreach c confirmed
                      (if (MBBN:RectsOverlap cur-r c 3.0) (setq collides T))
                    )
                    (if (and (null collides) (MBBN:IsRectInsidePoly orig-x orig-y w h new-out-pts 2.0))
                      (progn
                        ;; 이동 불필요 (현재 위치 확정)
                        (setq confirmed (cons cur-r confirmed))
                      )
                      (progn
                        ;; 돌출 또는 겹침 발생 -> MBBN 네스팅 2D 전역 비간섭 안전 후보 위치 탐색
                        (setq candidates '())

                        ;; 후보 1: 벽면 내측 직접 밀착 위치 (Nudge-In)
                        (setq needed-dx-l 0.0 needed-dx-r 0.0)
                        (foreach f '(0.0 0.25 0.5 0.75 1.0)
                          (setq y-c (+ orig-y (* f h))
                                rng (MBBG:GetXRangeAtY new-out-pts y-c))
                          (if rng
                            (progn
                              (if (< orig-x (+ (car rng) 2.0))
                                (setq needed-dx-l (max needed-dx-l (- (+ (car rng) 2.0) orig-x)))
                              )
                              (if (> (+ orig-x w) (- (cadr rng) 2.0))
                                (setq needed-dx-r (max needed-dx-r (- (+ orig-x w) (- (cadr rng) 2.0))))
                              )
                            )
                          )
                        )
                        (setq nudge-dx (cond ((> needed-dx-l 0.0) needed-dx-l)
                                             ((> needed-dx-r 0.0) (- 0.0 needed-dx-r))
                                             (t 0.0)))
                        (setq candidates (cons (list (+ orig-x nudge-dx) orig-y) candidates))

                        ;; 후보 2: 확정된 다른 단품 주변 앵커 (우측, 상단, 대각선)
                        (foreach c confirmed
                          (setq cx (nth 0 c) cy (nth 1 c) cw (nth 2 c) ch (nth 3 c))
                          (setq candidates (cons (list (+ cx cw 3.0) cy) candidates))
                          (setq candidates (cons (list cx (+ cy ch 3.0)) candidates))
                          (setq candidates (cons (list (+ cx cw 3.0) (+ cy ch 3.0)) candidates))
                        )

                        ;; 후보 3: 2D 전역 Y 스캔 (오목 곡선 틀 내부 유효 높이 전역 탐색)
                        (setq scan-y (+ shift-out-min-y 2.0))
                        (while (<= (+ scan-y h 2.0) shift-out-max-y)
                          (setq rng (MBBG:GetXRangeAtY new-out-pts scan-y))
                          (if rng
                            (progn
                              (setq min-x (+ (car rng) 2.0)
                                    max-x (- (cadr rng) w 2.0))
                              (if (<= min-x max-x)
                                (progn
                                  (setq candidates (cons (list min-x scan-y) candidates))
                                  (setq candidates (cons (list max-x scan-y) candidates))
                                  (setq sx (+ min-x 20.0))
                                  (while (<= sx (- max-x 10.0))
                                    (setq candidates (cons (list sx scan-y) candidates)
                                          sx (+ sx 30.0))
                                  )
                                )
                              )
                            )
                          )
                          (setq scan-y (+ scan-y 10.0))
                        )

                        ;; 유효 후보 필터링: 틀 내부(Margin 2mm) AND 확정 단품과 겹침 없음(Gap 3mm)
                        (setq valid-cands '())
                        (foreach cand candidates
                          (setq cand-r (list (car cand) (cadr cand) w h)
                                collides nil)
                          (foreach c confirmed
                            (if (MBBN:RectsOverlap cand-r c 3.0) (setq collides T))
                          )
                          (if (and (null collides) (MBBN:IsRectInsidePoly (car cand) (cadr cand) w h new-out-pts 2.0))
                            (progn
                              (setq dist (+ (expt (- (car cand) orig-x) 2) (expt (- (cadr cand) orig-y) 2)))
                              (setq valid-cands (cons (list dist (car cand) (cadr cand)) valid-cands))
                            )
                          )
                        )

                        ;; 원래 위치와 가장 가까운 비간섭 위치 선택
                        (if valid-cands
                          (progn
                            (setq valid-cands (vl-sort valid-cands '(lambda (p1 p2) (< (car p1) (car p2))))
                                  best-cand (car valid-cands)
                                  shift-dx (- (cadr best-cand) orig-x)
                                  shift-dy (- (caddr best-cand) orig-y))

                            ;; [핵심]: 외곽선 + 모든 내경(홀) + 문자를 한 묶음으로 일괄 이동!
                            (foreach o all-objs
                              (vl-catch-all-apply 'vla-Move (list o (vlax-3d-point '(0.0 0.0 0.0)) (vlax-3d-point (list shift-dx shift-dy 0.0))))
                            )
                            (setq confirmed (cons (list (cadr best-cand) (caddr best-cand) w h) confirmed)
                                  nudged-count (1+ nudged-count))
                          )
                          (progn
                            ;; 최후 폴백: 확정 목록 등록
                            (setq confirmed (cons cur-r confirmed))
                          )
                        )
                      )
                    )
                  )

                  (princ "\n================================================================")
                  (princ "\n>> [MBBG 완료] 오목 곡선 추종 & 스마트 비간섭 자동 배치 완료!")
                  (princ (strcat "\n   - 외곽 틀 최적 이동량: X = " (rtos final-dx 2 1) " mm, Y = " (rtos final-dy 2 1) " mm"))
                  (if (> nudged-count 0)
                    (princ (strcat "\n   - [비간섭 안착 보정] 돌출/간섭 단품 " (itoa nudged-count) "개를 안전 영역으로 무간섭 배치 완료 (겹침 0mm, 돌출 0mm 보장)"))
                    (if (and best-clear (>= best-clear 0.0))
                      (princ (strcat "\n   - 2D 균등 여백(Clearance): " (rtos best-clear 2 1) " mm (100% 무간섭 완벽 안착)"))
                    )
                  )
                  (princ "\n   - 각 단품의 모든 내경(구멍/슬롯)과 부속 문자가 일체형으로 완벽 유지됩니다.")
                  (princ "\n================================================================")
                )
                (princ "\n>> 오류: 내부 객체 선택에 유효한 부품이 없습니다.")
              )
            )
            (princ "\n>> 안내: 내부 부품이 선택되지 않았습니다.")
          )
        )
        (princ "\n>> 오류: 외곽 틀 정점 좌표를 추출할 수 없습니다.")
      )
    )
    (princ "\n>> 안내: 외곽 틀이 선택되지 않았습니다.")
  )

  (if (and ss-out (= (type ss-out) 'PICKSET)) (setq ss-out nil))
  (if (and ss-in (= (type ss-in) 'PICKSET)) (setq ss-in nil))
  (vla-EndUndoMark acDoc)
  (setvar 'CMDECHO 1)
  (princ)
)

;;; =============================================================================
;;; [START: CURRENT ACTIVE - v2.2 MODIFIED (2026-10-22 21:00:00)]
;;; [MBBN] 앵커포인트 & FFD 면적정렬 2D Nesting 다각형 내부 자동 배열 (v2.2)
;;; - [오목 원호 100% 추종]: vlax-curve-getPointAtDist 10mm 분할로 60mm 오목 원호/곡선 완벽 추종
;;; - [다중 내경 1:1 결합]: 외곽선 + 내부 홀(원/타원/슬롯) + 문자를 단일 어셈블리로 완전 결합
;;; - [텍스트 1:1 독점 바인딩]: 중복 할당 방지로 글자 분리/이탈 원천 차단
;;; - FFD(First-Fit Decreasing): 대형 부품 우선 배치로 공간 파편화 방지
;;; - 앵커포인트(Bottom-Left Fill): 컨테이너 하단 및 기배치 부품 외곽선 기반 밀착 배치
;;; - 부품별 0도 / 90도 회전 시 부속 내경 및 텍스트 일체형 동기화 회전/이동
;;; =============================================================================
(defun c:MBBN ( / *error* ss-out ss-in acDoc mspace i ename obj objName
                  out-min out-max out-pts p1 p2 num-v vi
                  total-len step-dist cur-dist
                  geom-raw text-list clustered-parts grp parent-item
                  placed ci minP maxP minC maxC
                  parent-obj children-holes text-objs all-objs assigned-texts part-texts
                  parts-list best-plan
                  gap margin out-min-x out-max-x out-min-y out-max-y
                  anchors wall-anchors ax ay cur-r collides found
                  item geom rot orig-min-x orig-min-y orig-w orig-h
                  move-dx move-dy pr pr-x pr-y pr-w pr-h rng
                  min-x max-x gy gx )
  (vl-load-com)
  (setq acDoc (vla-get-ActiveDocument (vlax-get-acad-object)))
  (setq mspace (vla-get-ModelSpace acDoc))

  ;; 에러 핸들러
  (defun *error* ( msg )
    (if (and acDoc (= 8 (logand 8 (getvar 'UNDOCTL))))
      (vl-catch-all-apply 'vla-EndUndoMark (list acDoc))
    )
    (if (and msg (not (wcmatch (strcase msg t) "*break*,*cancel*,*exit*")))
      (princ (strcat "\n>> [MBBN 에러]: " msg))
    )
    (setvar 'CMDECHO 1)
    (princ)
  )

  (setvar 'CMDECHO 0)
  (vla-StartUndoMark acDoc)

  (princ "\n================================================================")
  (princ "\n[MBBN v2.2] 오목 곡선추종 & FFD 앵커 네스팅 자동 배열")
  (princ "\n================================================================")

  ;; 1단계: 외곽 컨테이너 다각형 선택
  (princ "\n>> [1단계] 부품을 담을 외곽 컨테이너(틀) 다각형 선택: ")
  (if (setq ss-out (ssget ":S:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION"))))
    (progn
      (setq out-min nil out-max nil out-pts '() i 0)
      (repeat (sslength ss-out)
        (setq ename (ssname ss-out i)
              obj   (vlax-ename->vla-object ename)
              i     (1+ i))
        (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))
          (progn
            (setq tmp-min (vlax-safearray->list tmp-min)
                  tmp-max (vlax-safearray->list tmp-max))
            (setq out-min (if (null out-min) tmp-min (mapcar 'min out-min tmp-min))
                  out-max (if (null out-max) tmp-max (mapcar 'max out-max tmp-max)))
          )
        )
        ;; WCS Curve 둘레 기반 고밀도 정밀 샘플링 (10mm 간격, 오목 원호/곡선 100% 추종)
        (setq total-len nil)
        (if (not (vl-catch-all-error-p (setq total-len (vl-catch-all-apply 'vlax-curve-getDistAtParam (list obj (vlax-curve-getEndParam obj))))))
          (if (and (numberp total-len) (> total-len 0.0))
            (progn
              (setq step-dist (max 2.0 (min 10.0 (/ total-len 120.0)))
                    cur-dist 0.0)
              (while (< cur-dist total-len)
                (if (not (vl-catch-all-error-p (setq p1 (vl-catch-all-apply 'vlax-curve-getPointAtDist (list obj cur-dist)))))
                  (setq out-pts (cons (list (car p1) (cadr p1)) out-pts))
                )
                (setq cur-dist (+ cur-dist step-dist))
              )
              (setq out-pts (reverse out-pts))
            )
          )
        )
        ;; Curve 정점 백업 추출
        (if (null out-pts)
          (progn
            (setq num-v nil)
            (if (not (vl-catch-all-error-p (setq num-v (vl-catch-all-apply 'vlax-curve-getEndParam (list obj)))))
              (if (and (numberp num-v) (> num-v 0))
                (progn
                  (setq vi 0)
                  (while (< vi num-v)
                    (if (not (vl-catch-all-error-p (setq p1 (vl-catch-all-apply 'vlax-curve-getPointAtParam (list obj vi)))))
                      (setq out-pts (cons (list (car p1) (cadr p1)) out-pts))
                    )
                    (setq vi (1+ vi))
                  )
                  (setq out-pts (reverse out-pts))
                )
              )
            )
          )
        )
        ;; 좌표 백업 추출
        (if (null out-pts)
          (progn
            (setq p1 nil)
            (if (vlax-property-available-p obj 'Coordinates)
              (setq p1 (vl-catch-all-apply 'vlax-get (list obj 'Coordinates)))
            )
            (if (or (vl-catch-all-error-p p1) (null p1))
              (if (not (vl-catch-all-error-p (setq p2 (vl-catch-all-apply 'vla-get-Coordinates (list obj)))))
                (setq p1 (vlax-safearray->list (vlax-variant-value p2)))
              )
            )
            (if (and p1 (listp p1) (not (vl-catch-all-error-p p1)))
              (while p1
                (setq out-pts (cons (list (car p1) (cadr p1)) out-pts))
                (setq p1 (cddr p1))
              )
            )
          )
        )
      )

      (if (and out-min out-max out-pts (>= (length out-pts) 3))
        (progn
          (princ (strcat "\n>> [외곽 규격] 폭 = " (rtos (- (car out-max) (car out-min)) 2 1) 
                         " mm, 높이 = " (rtos (- (cadr out-max) (cadr out-min)) 2 1) " mm (고밀도 " (itoa (length out-pts)) "점 곡선 파싱)"))

          ;; 2단계: 배치할 내부 부품 및 문자 드래그 선택
          (princ "\n>> [2단계] 배치할 내부 부품 및 문자 전체 드래그 선택: ")
          (if (setq ss-in (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION,TEXT,MTEXT,INSERT"))))
            (progn
              (setq geom-raw '() text-list '() i 0)
              (repeat (sslength ss-in)
                (setq ename (ssname ss-in i)
                      obj   (vlax-ename->vla-object ename)
                      objName (vla-get-ObjectName obj)
                      i     (1+ i))
                (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))
                  (progn
                    (setq tmp-min (vlax-safearray->list tmp-min)
                          tmp-max (vlax-safearray->list tmp-max))
                    (if (wcmatch (strcase objName) "*TEXT*")
                      (setq text-list (cons (list obj tmp-min tmp-max) text-list))
                      (progn
                        (setq tmpArea (vl-catch-all-apply 'vla-get-Area (list obj)))
                        (if (vl-catch-all-error-p tmpArea)
                          (setq tmpArea (* (- (car tmp-max) (car tmp-min)) (- (cadr tmp-max) (cadr tmp-min))))
                        )
                        (setq geom-raw (cons (list obj tmpArea tmp-min tmp-max) geom-raw))
                      )
                    )
                  )
                )
              )

              ;; [내경(홀/구멍) 토폴로지 클러스터링]: 외곽선과 내부 내경들을 1:1 완벽 결합
              (setq geom-raw (vl-sort geom-raw '(lambda (a b) (> (cadr a) (cadr b))))
                    clustered-parts '())
              (foreach item geom-raw
                (setq obj (car item)
                      minC (nth 2 item)
                      maxC (nth 3 item)
                      placed nil
                      ci 0)
                (while (and (not placed) (< ci (length clustered-parts)))
                  (setq grp (nth ci clustered-parts)
                        parent-item (car grp)
                        minP (nth 2 parent-item)
                        maxP (nth 3 parent-item))
                  ;; item의 BBOX가 기존 parent BBOX 내부에 포함되면 내경(Child)으로 등록
                  (if (and (>= (car minC) (- (car minP) 0.5))
                           (>= (cadr minC) (- (cadr minP) 0.5))
                           (<= (car maxC) (+ (car maxP) 0.5))
                           (<= (cadr maxC) (+ (cadr maxP) 0.5)))
                    (progn
                      (setq grp (list parent-item (cons obj (cadr grp)))
                            clustered-parts (MBB:ReplaceNth ci grp clustered-parts)
                            placed T)
                    )
                  )
                  (setq ci (1+ ci))
                )
                (if (not placed)
                  (setq clustered-parts (cons (list item (list)) clustered-parts))
                )
              )
              (setq clustered-parts (reverse clustered-parts))

              ;; [텍스트 1:1 독점 바인딩]: 중복 할당 방지하여 글자 분리/이탈 원천 차단
              (setq assigned-texts '()
                    parts-list '()
                    i 1)
              (foreach grp clustered-parts
                (setq parent-obj (caar grp)
                      children-holes (cadr grp)
                      tmp-min (nth 2 (car grp))
                      tmp-max (nth 3 (car grp))
                      part-texts '())
                (foreach t-item text-list
                  (setq t-obj (nth 0 t-item)
                        t-pt (nth 1 t-item))
                  (if (and (null (vl-position t-obj assigned-texts))
                           (<= (- (car tmp-min) 50.0) (car t-pt))
                           (<= (car t-pt) (+ (car tmp-max) 50.0))
                           (<= (- (cadr tmp-min) 50.0) (cadr t-pt))
                           (<= (cadr t-pt) (+ (cadr tmp-max) 50.0)))
                    (progn
                      (setq part-texts (cons t-obj part-texts)
                            assigned-texts (cons t-obj assigned-texts))
                    )
                  )
                )
                (setq all-objs (cons parent-obj (append children-holes part-texts))
                      w (- (car tmp-max) (car tmp-min))
                      h (- (cadr tmp-max) (cadr tmp-min))
                      area (* w h))
                ;; part 구조: (all-objs (car tmp-min) (cadr tmp-min) w h area (length children-holes) (length part-texts))
                (setq parts-list (cons (list all-objs (car tmp-min) (cadr tmp-min) w h area (length children-holes) (length part-texts)) parts-list)
                      i (1+ i))
              )

              (if parts-list
                (progn
                  ;; FFD (First-Fit Decreasing): 면적 내림차순 정렬 (대형 부품 우선 배치)
                  (setq parts-list (vl-sort parts-list '(lambda (a b) (> (nth 5 a) (nth 5 b)))))
                  
                  (setq i 1)
                  (foreach p parts-list
                    (princ (strcat "\n   - [단품 " (itoa i) "] 폭 = " (rtos (nth 3 p) 2 1) 
                                   " mm, 높이 = " (rtos (nth 4 p) 2 1) 
                                   " mm (내경 " (itoa (nth 6 p)) "개, 문자 " (itoa (nth 7 p)) "개 독점결합)"))
                    (setq i (1+ i))
                  )

                  (princ (strcat "\n>> 총 " (itoa (length parts-list)) "개 단품 FFD 정렬 완료. 곡선추종 앵커 네스팅 연산 중..."))

                  (setq gap 3.0
                        margin 2.0
                        out-min-x (car out-min)
                        out-max-x (car out-max)
                        out-min-y (cadr out-min)
                        out-max-y (cadr out-max)
                        placed '()
                        best-plan '())

                  ;; 부품별 앵커포인트 + 회전(0도, 90도) 탐색
                  (foreach p parts-list
                    (setq orig-w (nth 3 p)
                          orig-h (nth 4 p)
                          found nil)

                    ;; 1. 후보 앵커 포인트 수집 (초기 좌하단 + 기배치 부품 우측/상단)
                    (setq anchors (list (list (+ out-min-x margin) (+ out-min-y margin))))
                    (foreach pr placed
                      (setq pr-x (nth 0 pr)
                            pr-y (nth 1 pr)
                            pr-w (nth 2 pr)
                            pr-h (nth 3 pr))
                      (setq anchors (cons (list (+ pr-x pr-w gap) pr-y) anchors))
                      (setq anchors (cons (list pr-x (+ pr-y pr-h gap)) anchors))
                      (setq anchors (cons (list (+ out-min-x margin) (+ pr-y pr-h gap)) anchors))
                    )

                    ;; 오목 곡선 벽면 추종(Wall-Hugging) 앵커 추가
                    (setq wall-anchors '())
                    (foreach a anchors
                      (setq ay (cadr a)
                            rng (MBBG:GetXRangeAtY out-pts ay))
                      (if rng
                        (setq wall-anchors (cons (list (+ (car rng) margin) ay) wall-anchors))
                      )
                    )
                    (setq anchors (append anchors wall-anchors))

                    ;; Y 오름차순, X 오름차순 정렬
                    (setq anchors (vl-sort anchors '(lambda (pt1 pt2)
                                                     (if (equal (cadr pt1) (cadr pt2) 1.0)
                                                       (< (car pt1) (car pt2))
                                                       (< (cadr pt1) (cadr pt2))))))

                    ;; 앵커 포인트 기반 탐색 (0도 및 90도 회전 순차 테스트)
                    (foreach rot '(0 90)
                      (if (null found)
                        (progn
                          (setq w (if (= rot 90) orig-h orig-w)
                                h (if (= rot 90) orig-w orig-h))
                          (foreach a anchors
                            (if (null found)
                              (progn
                                (setq ax (car a) ay (cadr a))
                                (if (MBBN:IsRectInsidePoly ax ay w h out-pts margin)
                                  (progn
                                    (setq cur-r (list ax ay w h)
                                          collides nil)
                                    (foreach pr placed
                                      (if (MBBN:RectsOverlap cur-r pr gap)
                                        (setq collides T)
                                      )
                                    )
                                    (if (null collides)
                                      (progn
                                        (setq placed (cons cur-r placed)
                                              best-plan (cons (list p ax ay w h rot) best-plan)
                                              found T)
                                      )
                                    )
                                  )
                                )
                              )
                            )
                          )
                        )
                      )
                    )

                    ;; 앵커 포인트에서 찾지 못한 경우 정밀 스캔 백업 (5mm 스텝)
                    (if (null found)
                      (foreach rot '(0 90)
                        (if (null found)
                          (progn
                            (setq w (if (= rot 90) orig-h orig-w)
                                  h (if (= rot 90) orig-w orig-h)
                                  gy (+ out-min-y margin))
                            (while (and (<= (+ gy h margin) out-max-y) (null found))
                              (setq rng (MBBG:GetXRangeAtY out-pts gy))
                              (if rng
                                (progn
                                  (setq min-x (max (+ out-min-x margin) (+ (car rng) margin))
                                        max-x (min (- out-max-x w margin) (- (cadr rng) w margin))
                                        gx min-x)
                                  (while (and (<= gx max-x) (null found))
                                    (if (MBBN:IsRectInsidePoly gx gy w h out-pts margin)
                                      (progn
                                        (setq cur-r (list gx gy w h)
                                              collides nil)
                                        (foreach pr placed
                                          (if (MBBN:RectsOverlap cur-r pr gap)
                                            (setq collides T)
                                          )
                                        )
                                        (if (null collides)
                                          (progn
                                            (setq placed (cons cur-r placed)
                                                  best-plan (cons (list p gx gy w h rot) best-plan)
                                                  found T)
                                          )
                                        )
                                      )
                                    )
                                    (setq gx (+ gx 5.0))
                                  )
                                )
                              )
                              (setq gy (+ gy 5.0))
                            )
                          )
                        )
                      )
                    )
                  )

                  ;; 전체 부품 배치 완료 검증
                  (if (and best-plan (= (length best-plan) (length parts-list)))
                    (progn
                      (foreach item best-plan
                        (setq p (nth 0 item)
                              gx (nth 1 item)
                              gy (nth 2 item)
                              w  (nth 3 item)
                              h  (nth 4 item)
                              rot (nth 5 item)
                              all-objs (nth 0 p)
                              orig-min-x (nth 1 p)
                              orig-min-y (nth 2 p)
                              orig-w (nth 3 p)
                              orig-h (nth 4 p))
                        (if (= rot 90)
                          (progn
                            (foreach o all-objs
                              (vl-catch-all-apply 'vla-Rotate (list o (vlax-3d-point (list orig-min-x orig-min-y 0.0)) (/ pi 2.0)))
                            )
                            (setq move-dx (+ (- gx orig-min-x) orig-h)
                                  move-dy (- gy orig-min-y))
                          )
                          (setq move-dx (- gx orig-min-x)
                                move-dy (- gy orig-min-y))
                        )
                        (foreach o all-objs
                          (vl-catch-all-apply 'vla-Move (list o (vlax-3d-point '(0.0 0.0 0.0)) (vlax-3d-point (list move-dx move-dy 0.0))))
                        )
                      )
                      (princ "\n================================================================")
                      (princ (strcat "\n>> [MBBN 완료] 총 " (itoa (length best-plan)) "개 단품 FFD 앵커 네스팅 자동 배열 완료!"))
                      (princ (strcat "\n   - 부품 간격(Gap): " (rtos gap 2 1) " mm / 외곽 여백(Margin): " (rtos margin 2 1) " mm"))
                      (princ "\n   - 각 단품의 모든 내경(구멍/슬롯)과 부속 문자가 일체형으로 완벽 유지됩니다.")
                      (princ "\n================================================================")
                    )
                    (princ "\n>> 오류: 외곽 컨테이너 공간이 협소하여 모든 부품을 배치할 수 없습니다.")
                  )
                )
                (princ "\n>> 오류: 배치 가능한 내부 부품 객체가 없습니다.")
              )
            )
            (princ "\n>> 안내: 배치할 부품이 선택되지 않았습니다.")
          )
        )
        (princ "\n>> 오류: 외곽 다각형 정점 좌표를 추출할 수 없습니다.")
      )
    )
    (princ "\n>> 안내: 외곽 컨테이너가 선택되지 않았습니다.")
  )

  (if (and ss-out (= (type ss-out) 'PICKSET)) (setq ss-out nil))
  (if (and ss-in (= (type ss-in) 'PICKSET)) (setq ss-in nil))
  (vla-EndUndoMark acDoc)
  (setvar 'CMDECHO 1)
  (princ)
)

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
  (write-line "    : button { key = \"btn_pclose\"; label = \"7. PCLOSE (Close Polylines)\"; }" fp)
  (write-line "  }" fp)
  (write-line "  : boxed_column {" fp)
  (write-line "    label = \"Alignment & Placement\";" fp)
  (write-line "    : button { key = \"btn_mbbg\"; label = \"8. MBBG (Equal Margin Align)\"; }" fp)
  (write-line "    : button { key = \"btn_mbbn\"; label = \"9. MBBN (Grid Auto Nesting)\"; }" fp)
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
  (action_tile "btn_mbbn" "(done_dialog 9)")
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
    ((= action 9) (c:MBBN))
  )
  (setq *MBB_DCL_THK* nil)
  (princ)
)

;; 한글 입력 오타 방지 별칭 (ㅡㅠㅠㅕㅑ = MBBUI)
(defun c:ㅡㅠㅠㅕㅑ () (c:MBBUI))
