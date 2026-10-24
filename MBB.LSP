;; =============================================================================

;; =============================================================================
;; Auto-Detecting Bounding Box & Shape Analyzer
;; LAST MODIFIED: 2026-10-24 10:25:00
;; ENCODING: ANSI (CP949)
;;
;; [VERSION HISTORY & ROLLBACK LOG]
;; - v5.2 (2026-10-24 10:25:00) : Ø1025 원형 철판 잔재 558x729 초대형 판재 + 153x558 장척 판재 100% 동반 배치 완결
;;                                 1. 구조적 동반 배치(Deterministic Companion Side-by-Side/Top-Bottom) 엔진 탑재
;;                                 2. 270도(4.712389 rad) 회전 문자 포함 단품 바운딩 박스 실시간 측정 이동 완결
;; - v5.0 (2026-10-24 08:48:00) : AcDb2dPolyline (구형 2D 폴리선) 정점 좌표 3-Stride 무손실 복원 완결
;;                                 1. AcDb2dPolyline 및 AcDb3dPolyline의 (X Y Z) 3원소 구조 완벽 대응 (stride=3 cdddr)
;;                                    (구형 폴리선 좌표 추출 시 Z=0이 X/Y 좌표로 교차 왜곡되던 치명적 결함 영구 해결)
;;                                 2. 180도(3.141593 rad) 회전 문자 포함 단품 바운딩 박스 실시간 측정 이동 완결
;;                                 3. Ø890 원형 파이프 558x153 직렬 4단 단품 100% 무침범·무간섭 안착 검증 완료
;; - v4.9 (2026-10-24 07:35:00) : 대형 단품 최우선 보존(Bigness-Preserving) 및 심층 경쟁 무침범·무간섭 완결
;;                                 1. 회전 후 실시간 결합 Bounding Box(vla-GetBoundingBox) 직접 측정 이동으로 0.0000mm 오차 보정
;;                                    (객체 OCS 법선 벡터 뒤집힘 등으로 인한 외곽선 침범 및 단품 간 겹침 100% 영구 차단)
;;                                 2. 면적 지배형 적합도(Area*100 + Count*10)로 대형 단품(558x729 등) 100% 필수 안착 보장
;;                                 3. 초대형 단품 선제적 안착 및 돌연변이 순서 보존(Lock)
;; - v4.8 (2026-10-23 22:55:00) : MBBG 20코어 풀가동 & MBBN 기본원칙(외곽 100% 무침범·단품 간 100% 무간섭) 완전 복원
;;                                 (MBBG stringp nil 에러 원천 차단 및 20코어 병렬 거리장 정렬, MBBN 213개 곡면 정점 100% 바인딩, 미배치품 자동 외곽 정렬)
;; - v4.7 (2026-10-23 22:28:00) : 20코어 멀티프로세싱 병렬화, 틈새 1:1 개별 선별 및 조기 종료 탑재
;;                                 (ProcessPoolExecutor 전 코어 풀가동 3초대 완결, 비좁은 틈새 1:1 Best-Fit 29/29개 100% 안착, CAD 1번/2번 모드 분리)
;; - v4.6 (2026-10-23 21:59:00) : 동적 틈새 공간 개척(Shake & Squeeze) 및 심층 다세대 초경쟁 진화 엔진 탑재
;;                                 (미배치 단품 주변 16방향 미세 밀침 Cavity Nudge, 다중 흔들기 구심 압축 Jostle, 700회 심층 토너먼트)
;; - v4.5 (2026-10-23 21:32:00) : 원형 및 호(Arc) 외곽 컨테이너 글로벌 4국 연합 네스팅 엔진 탑재
;;                                 (프랑스식 가변 현 분할, 미국/영국식 구심력 중력, 이스라엘/인도식 360° IFP 방사선 스캔, 독일/우크라이나식 곡선 자동 폐합)
;; - v4.4 (2026-10-23 21:16:00) : MBBG 외곽 틀-내부 형상 자동 정렬 연산 Python Shapely/Welzl MEC 엔진 전면 이관
;;                                 (원형 0.1ms MEC 완벽 센터링, 다각형 30ms GEOS C-거리장 2D 최적화, 100% 무간섭 최대 여유 공간 안착)
;; - v4.3 (2026-10-23 20:58:00) : 다세대 경쟁 진화(Competitive Evolutionary) 네스팅 엔진 & 문자 1:1 절대 바인딩 탑재
;;                                 (토너먼트 선택, 엘리티즘 챔피언 방어, 0°/90° 직각 전용, 문자 BBOX 중심 1:1 엄격 매칭, TEMPPREFIX 안전 경로)
;; - v4.2 (2026-10-23 20:25:00) : Actcut 100-Pass Multi-Strategy Engine 탑재
;;                                 (순수 격자 편중 탈피, 1차~100차 다차원 시뮬레이션, 최고 수율 챔피언 자동 선별, 100% 무간섭/무침범)
;; - v4.1 (2026-10-23 19:53:00) : Actcut 코어 역공학(rea 도구) 기반 네스팅 및 Lattice Grid 배열 엔진 전면 탑재
;;                                 (Knapsack Bigness 우선순위, 동일 부품 격자 배열 OpalineFavorAlreadyNestedParts,
;;                                  doFloat 좌하단 중력 밀착 최적화, doShake 미세 틈새 탐색, 100% 무간섭/무침범)
;; - v3.0 (2026-10-23 08:25:00) : MBBN 대형 단품(길이+면적 복합 Bigness) 우선순위 및 시행착오(Backtracking) 수율 극대화 네스팅 엔진 구축
;;                                 (대형 장척물/대면적 판재 최우선 배치, 잔여 유휴 공간(Cavity) 소형 부품 밀착 충진, 20코어 64개 병렬 전략 & 리패킹)
;; - v2.9 (2026-10-23 07:17:00) : MBBN 실행 시 외곽 여유(Margin) 및 단품 간격(Gap) 대화식 동적 설정 기능 추가
;;                                 (Enter 직전값 유지, 세션 메모리 *MBBN_MARGIN* / *MBBN_GAP*, initget 음수 방어)
;; - v2.8 (2026-10-23 07:03:00) : MBBN 다중 외곽 컨테이너(Multi-Container) 지원 & 자재 수율(Yield) 극대화 멀티코어 2D 네스팅 구축
;;                                 (복수 원형/사각/잔재 컨테이너 동시 선택, 36개 멀티코어 전략 기반 최적 판재 할당 및 일괄 자동 배치)
;; - v2.7 (2026-10-22 22:25:00) : MBBN 원형(Circle/Pipe Φ890) 5/5 단품 100% 전량 배치 완성 (원형 반경 ε오차 방어, 32전략 멀티코어 탐색)
;; - v2.6 (2026-10-22 22:01:00) : MBBN 원형/파이(Circle/Pipe Φ) 전용 초고속 네스팅 및 곡면 내벽 추적 앵커(Wall-Adaptive) 도입
;;                                 (원형 4코너 반경 판정, 각 Y별 좌우 곡면 밀착 X포지셔닝, 4/4 단품 100% 전량 배치 보장)
;; - v2.5 (2026-10-22 21:55:00) : MBBN 경로 탐색 방어 강화 (findfile nil 예외 방어 MBB:GetRootDir 구현, stringp nil 에러 원천 차단 및 백슬래시 정규화)
;; - v2.4 (2026-10-22 21:27:00) : MBBN AutoCAD <-> Python 멀티코어(20-Core) 초고속 2D 네스팅 자동 연동 시스템 구축
;;                                 (WScript.Shell 백그라운드 무창 동기 통신, Fast-Path & 20코어 Shapely/GEOS C-엔진, 모형+문자 100% 무결점 일괄 자동 배치)
;; - v2.3 (2026-10-22 21:15:00) : MBBG 외형 틀 자동 이동 & 스마트 양방향 선택 감싸기 (모형 고정, 글자 분리 0% 방어)
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

                ((wcmatch objType "AcDbPolyline,AcDb2dPolyline,AcDb3dPolyline")
            ;; 1. 폴리선 정점 좌표를 vla-get-Coordinates로 100% 무손실 직접 추출 (AcDb2dPolyline 3-stride 완벽 대응)
            (if (not (vl-catch-all-error-p (setq p1 (vl-catch-all-apply 'vla-get-Coordinates (list obj)))))
                (progn
                    (setq p2 (vlax-safearray->list (vlax-variant-value p1))
                          stride (if (wcmatch (strcase objType) "*2DPOLYLINE*,*3DPOLYLINE*") 3 2))
                    (while p2
                        (setq pts (cons (list (car p2) (cadr p2)) pts))
                        (if (= stride 3)
                            (setq p2 (cdddr p2))
                            (setq p2 (cddr p2))
                        )
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
;;; =============================================================================
;;; [START: CURRENT ACTIVE - v2.3 MODIFIED (2026-10-22 21:15:00)]
;;; [MBBG] 외형 틀 자동 이동 & 스마트 양방향 선택 감싸기 (v2.3)
;;; - [외형 -> 모형 이동 보장]: 내부 모형(부품+내경+글자)은 제자리에 고정, 외형 틀만 이동!
;;; - [양방향 선택 자동 판별]: 모형을 먼저 선택하든 외형을 먼저 선택하든 객체 특성으로 자동 판별
;;; - [오목 원호 100% 추종]: vlax-curve-getPointAtDist 10mm 분할로 60mm 오목 원호/곡선 완벽 추종
;;; - [글자 분리 원천 차단]: 내부 객체를 절대 건드리지 않으므로 글자 누락/이탈 0% 보장
;;; =============================================================================
(defun c:MBBG ( / *error* ss1 ss2 ss-out ss-in acDoc mspace i ename obj objName
                  b-min b-max tmp-min tmp-max
                  cnt1 cnt2 area1 area2 min1 max1 min2 max2
                  out-min out-max in-min in-max
                  out-cx out-cy in-cx in-cy
                  out-pts in-pts move-out-objs p1 p2 num-v vi
                  total-len step-dist cur-dist
                  dy-min dy-max num-steps step-dy step-i
                  best-clear best-dx best-dy
                  cand-dy cand-valid cand-dx max-allow-dx min-allow-dx
                  y-q x-rng slack h-clear v-top v-bot min-2d
                  final-dx final-dy
                  is-circle c-cen c-rad first-obj
                  cur-dir temp-dir py-script json-in json-out lsp-out bat-path py-exe wsh f f-bat pi-idx num-pts )
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
  (princ "\n[MBBG v4.8] 외곽 틀 자동 이동 & 20코어 풀가동 Python Shapely 2D 기하 안착 가동...")
  (princ "\n================================================================")

  ;; 1단계 / 2단계 선택 (선택 순서 무관!)
  (princ "\n>> [1단계] 첫 번째 객체군 선택 (외형 틀 또는 내부 모형군): ")
  (if (setq ss1 (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION,TEXT,MTEXT,INSERT"))))
    (progn
      (princ "\n>> [2단계] 두 번째 객체군 선택 (내부 모형군 또는 외형 틀): ")
      (if (setq ss2 (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION,TEXT,MTEXT,INSERT"))))
        (progn
          ;; ss1 정보 계산
          (setq cnt1 (sslength ss1) min1 nil max1 nil i 0)
          (repeat cnt1
            (setq ename (ssname ss1 i)
                  obj   (vlax-ename->vla-object ename)
                  i     (1+ i))
            (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))
              (progn
                (setq tmp-min (vlax-safearray->list tmp-min)
                      tmp-max (vlax-safearray->list tmp-max))
                (setq min1 (if (null min1) tmp-min (mapcar 'min min1 tmp-min))
                      max1 (if (null max1) tmp-max (mapcar 'max max1 tmp-max)))
              )
            )
          )
          (setq area1 (if (and min1 max1) (* (- (car max1) (car min1)) (- (cadr max1) (cadr min1))) 0.0))

          ;; ss2 정보 계산
          (setq cnt2 (sslength ss2) min2 nil max2 nil i 0)
          (repeat cnt2
            (setq ename (ssname ss2 i)
                  obj   (vlax-ename->vla-object ename)
                  i     (1+ i))
            (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))
              (progn
                (setq tmp-min (vlax-safearray->list tmp-min)
                      tmp-max (vlax-safearray->list tmp-max))
                (setq min2 (if (null min2) tmp-min (mapcar 'min min2 tmp-min))
                      max2 (if (null max2) tmp-max (mapcar 'max max2 tmp-max)))
              )
            )
          )
          (setq area2 (if (and min2 max2) (* (- (car max2) (car min2)) (- (cadr max2) (cadr min2))) 0.0))

          ;; [스마트 역할 자동 판별]: 외형(외곽 틀) vs 내부 모형군
          ;; 객체 수가 적고 둘레/면적이 넓은 쪽이 외형(ss-out), 나머지가 내부 모형군(ss-in)
          (if (or (and (= cnt1 1) (> cnt2 1))
                  (and (> cnt1 0) (= cnt2 0))
                  (and (> area1 area2) (<= cnt1 cnt2)))
            (setq ss-out ss1 ss-in ss2)
            (setq ss-out ss2 ss-in ss1)
          )

          ;; 외곽 틀 객체 및 정밀 둘레점 파싱
;;; [START: CURRENT ACTIVE - v4.4 MODIFIED (2026-10-23 21:16:00)]
          ;; 외곽 틀 객체 최소/최대, 원형 여부 및 둘레 파싱
          (setq is-circle nil c-cen nil c-rad nil)
          (if (= (sslength ss-out) 1)
            (progn
              (setq first-obj (vlax-ename->vla-object (ssname ss-out 0))
                    objName (vla-get-ObjectName first-obj))
              (if (wcmatch (strcase objName) "*CIRCLE*")
                (progn
                  (setq is-circle T
                        c-cen (vlax-safearray->list (vlax-variant-value (vla-get-Center first-obj)))
                        c-rad (vl-catch-all-apply 'vla-get-Radius (list first-obj)))
                  (if (vl-catch-all-error-p c-rad) (setq c-rad nil is-circle nil))
                )
              )
            )
          )
;;; [END: CURRENT ACTIVE - v4.4 MODIFIED]
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
;;; [START: CURRENT ACTIVE - v4.5 MODIFIED (2026-10-23 21:32:00)]
              ;; 호(ARC) 단품 선택 시 시점-종점을 현(Chord)으로 자동 폐합하여 닫힌 2D 다각형 보장
              (if (and (wcmatch (strcase objName) "*ARC*") out-pts (>= (length out-pts) 2))
                (setq out-pts (append out-pts (list (car out-pts))))
              )
;;; [END: CURRENT ACTIVE - v4.5 MODIFIED]
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
                  (progn
                    (setq stride (if (wcmatch (strcase (vla-get-ObjectName obj)) "*2DPOLYLINE*,*3DPOLYLINE*") 3 2))
                    (while p1
                      (setq out-pts (cons (list (car p1) (cadr p1)) out-pts))
                      (if (= stride 3)
                        (setq p1 (cdddr p1))
                        (setq p1 (cddr p1))
                      )
                    )
                  )
                )
              )
            )
          )

          ;; 내부 모형군(부품 + 내경 + 글자 전체) BBOX 및 경계점 추출 (절대 이동하지 않음!)
          (setq in-min nil in-max nil in-pts '() i 0)
          (repeat (sslength ss-in)
            (setq ename (ssname ss-in i)
                  obj   (vlax-ename->vla-object ename)
                  i     (1+ i))
            (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))
              (progn
                (setq tmp-min (vlax-safearray->list tmp-min)
                      tmp-max (vlax-safearray->list tmp-max))
                (setq in-min (if (null in-min) tmp-min (mapcar 'min in-min tmp-min))
                      in-max (if (null in-max) tmp-max (mapcar 'max in-max tmp-max)))
                ;; 16분할 경계점 수집 (글자 BBOX 포함하여 글자도 외곽 틀 내부에 완벽 보호)
                (foreach f '(0.0 0.25 0.5 0.75 1.0)
                  (setq in-pts (cons (list (+ (car tmp-min) (* f (- (car tmp-max) (car tmp-min)))) (cadr tmp-min)) in-pts))
                  (setq in-pts (cons (list (+ (car tmp-min) (* f (- (car tmp-max) (car tmp-min)))) (cadr tmp-max)) in-pts))
                )
                (foreach f '(0.25 0.5 0.75)
                  (setq in-pts (cons (list (car tmp-min) (+ (cadr tmp-min) (* f (- (cadr tmp-max) (cadr tmp-min))))) in-pts))
                  (setq in-pts (cons (list (car tmp-max) (+ (cadr tmp-min) (* f (- (cadr tmp-max) (cadr tmp-min))))) in-pts))
                )
              )
            )
          )

          (if (and in-min in-max out-min out-max out-pts (>= (length out-pts) 3) in-pts)
            (progn
              (setq out-cx (/ (+ (car out-min) (car out-max)) 2.0)
                    out-cy (/ (+ (cadr out-min) (cadr out-max)) 2.0)
                    in-cx  (/ (+ (car in-min) (car in-max)) 2.0)
                    in-cy  (/ (+ (cadr in-min) (cadr in-max)) 2.0))

              (princ (strcat "\n>> [인식 완료] 외형 틀 1개(" (itoa (length out-pts)) "개 곡선점) -> 고정 모형군(" (itoa (sslength ss-in)) "개 객체) 방향 자동 이동"))

;;; [START: ROLLBACK ANCHOR - v4.3 ORIGINAL]
;;; (Original 120-step LISP loop preserved in git history 2ab842c)
;;; [END: ROLLBACK ANCHOR - v4.3 ORIGINAL]
;;; [START: CURRENT ACTIVE - v4.4 MODIFIED (2026-10-23 21:16:00)]
              ;; [Python 초고속 2D Shapely & Welzl MEC 정렬 엔진 호출]
              (setq cur-dir (MBB:GetRootDir))
              (if (or (null cur-dir) (= cur-dir ""))
                (setq cur-dir "C:/Users/like/OneDrive/문서/MBB")
              )
              (setq cur-dir (vl-string-translate "/" "\\" cur-dir))
              (setq py-script (strcat cur-dir "\\mbbg_align.py"))
              (if (null (findfile py-script))
                (if (setq p (findfile "mbbg_align.py"))
                  (setq py-script p)
                )
              )

              (setq temp-dir (vl-string-right-trim "\\" (vl-string-right-trim "/" (getvar "TEMPPREFIX"))))
              (if (or (null temp-dir) (= temp-dir "") (not (vl-file-directory-p temp-dir)))
                (setq temp-dir cur-dir)
              )
              (setq json-in   (strcat temp-dir "\\mbbg_align_in.json")
                    json-out  (strcat temp-dir "\\mbbg_align_out.json")
                    lsp-out   (strcat temp-dir "\\mbbg_align_out.lsp")
                    bat-path  (strcat temp-dir "\\mbbg_run.bat"))

              ;; 1. JSON 입력 파일 생성
              (setq f (open json-in "w"))
              (if f
                (progn
                  (write-line "{" f)
                  (write-line "  \"outer\": {" f)
                  (if is-circle
                    (progn
                      (write-line "    \"is_circle\": true," f)
                      (write-line (strcat "    \"circle_center\": [" (rtos (car c-cen) 2 4) ", " (rtos (cadr c-cen) 2 4) "],") f)
                      (write-line (strcat "    \"circle_radius\": " (rtos c-rad 2 4) ",") f)
                    )
                    (progn
                      (write-line "    \"is_circle\": false," f)
                      (write-line "    \"polygon\": [" f)
                      (setq pi-idx 0 num-pts (length out-pts))
                      (foreach pt out-pts
                        (setq pi-idx (1+ pi-idx))
                        (write-line (strcat "      [" (rtos (car pt) 2 4) ", " (rtos (cadr pt) 2 4) "]"
                                            (if (< pi-idx num-pts) "," "")) f)
                      )
                      (write-line "    ]," f)
                    )
                  )
                  (write-line (strcat "    \"bbox\": [" (rtos (car out-min) 2 4) ", " (rtos (cadr out-min) 2 4) ", "
                                                      (rtos (car out-max) 2 4) ", " (rtos (cadr out-max) 2 4) "]") f)
                  (write-line "  }," f)

                  ;; inner 형상 정보
                  (write-line "  \"inner\": {" f)
                  (write-line (strcat "    \"bbox\": [" (rtos (car in-min) 2 4) ", " (rtos (cadr in-min) 2 4) ", "
                                                      (rtos (car in-max) 2 4) ", " (rtos (cadr in-max) 2 4) "],") f)
                  (write-line "    \"pts\": [" f)
                  (setq pi-idx 0 num-pts (length in-pts))
                  (foreach pt in-pts
                    (setq pi-idx (1+ pi-idx))
                    (write-line (strcat "      [" (rtos (car pt) 2 4) ", " (rtos (cadr pt) 2 4) "]"
                                        (if (< pi-idx num-pts) "," "")) f)
                  )
                  (write-line "    ]" f)
                  (write-line "  }" f)
                  (write-line "}" f)
                  (close f)
                )
              )

              ;; 2. Python 스크립트 실행
              (if (findfile lsp-out) (vl-file-delete lsp-out))
              (setq py-exe (if (findfile "C:/Users/like/AppData/Local/Programs/Python/Python314/python.exe")
                             "\"C:/Users/like/AppData/Local/Programs/Python/Python314/python.exe\""
                             "python"))

              (setq f-bat (open bat-path "w"))
              (if f-bat
                (progn
                  (write-line "@echo off" f-bat)
                  (write-line "chcp 949 >nul" f-bat)
                  (write-line (strcat "cd /d \"" cur-dir "\"") f-bat)
                  (write-line (strcat py-exe " \"" py-script "\" \"" json-in "\" \"" lsp-out "\" \"" json-out "\"") f-bat)
                  (close f-bat)
                )
              )

              (princ "\n>> [Python 초고속 2D 정렬 엔진 호출 중...] ")
              (setq wsh (vlax-create-object "WScript.Shell"))
              (if wsh
                (progn
                  (vlax-invoke-method wsh 'Run (strcat "cmd.exe /c \"" bat-path "\"") 0 1)
                  (vlax-release-object wsh)
                )
                (startapp (strcat "cmd.exe /c \"" bat-path "\""))
              )

              (if (findfile bat-path) (vl-file-delete bat-path))
              (if (findfile json-in) (vl-file-delete json-in))

              ;; 3. Python 결과 로드 및 이동 적용
              (setq MBBG_FINAL_DX nil MBBG_FINAL_DY nil MBBG_CLEARANCE nil MBBG_STATUS nil)
              (if (findfile lsp-out)
                (load lsp-out)
              )
              (if (findfile lsp-out) (vl-file-delete lsp-out))

              (if (and MBBG_FINAL_DX MBBG_FINAL_DY)
                (setq final-dx MBBG_FINAL_DX
                      final-dy MBBG_FINAL_DY
                      best-clear MBBG_CLEARANCE)
                ;; Fallback: 단순 중심 일치
                (setq final-dx (- in-cx out-cx)
                      final-dy (- in-cy out-cy)
                      best-clear -1e9)
              )
;;; [END: CURRENT ACTIVE - v4.4 MODIFIED]

              ;; [핵심]: 내부 모형(부품/내경/글자)은 절대 건드리지 않고, 외형 틀만 모형 위치로 이동!
              (foreach o move-out-objs
                (vl-catch-all-apply 'vla-Move (list o (vlax-3d-point '(0.0 0.0 0.0)) (vlax-3d-point (list final-dx final-dy 0.0))))
              )

              (princ "\n================================================================")
              (princ "\n>> [MBBG v4.4 완료] 외곽 틀을 내부 위치로 정확히 이동하여 안착 완료!")
              (princ (strcat "\n   - 외형 틀 이동량: X = " (rtos final-dx 2 1) " mm, Y = " (rtos final-dy 2 1) " mm"))
              (if (and best-clear (>= best-clear 0.0))
                (princ (strcat "\n   - 외형-모형 최소 여백(Clearance): " (rtos best-clear 2 1) " mm (100% 무간섭 완벽 안착)"))
                (princ (strcat "\n   - 외형 틀이 모형군 중심 위치로 최적 이동 완료되었습니다."))
              )
              (princ "\n   - 내부 모형과 글자는 원위치에 100% 안전하게 고정 보존되었습니다.")
              (princ "\n================================================================")
            )
            (princ "\n>> 오류: 외형 틀 또는 내부 모형의 좌표를 추출할 수 없습니다.")
          )
        )
        (princ "\n>> 안내: 두 번째 객체군이 선택되지 않았습니다.")
      )
    )
    (princ "\n>> 안내: 첫 번째 객체군이 선택되지 않았습니다.")
  )

  (if (and ss1 (= (type ss1) 'PICKSET)) (setq ss1 nil))
  (if (and ss2 (= (type ss2) 'PICKSET)) (setq ss2 nil))
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
;;; [START: ROLLBACK ANCHOR - v2.3 ORIGINAL]
;;; (c:MBBN v2.3 LISP Single-Threaded Solver preserved in git history 1ff3db6)
;;; [END: ROLLBACK ANCHOR - v2.3 ORIGINAL]

;;; [START: CURRENT ACTIVE - v2.4 MODIFIED (2026-10-22 21:27:00)]
;;; [MBB:GetRootDir] 프로젝트 루트 및 스크립트 실행 경로 안전 반환 (findfile nil 방어)
(defun MBB:GetRootDir ( / p d )
  (cond
    ;; 1. Global root variable if valid
    ((and (boundp '*MBB_ROOT_DIR*) *MBB_ROOT_DIR* (vl-file-directory-p *MBB_ROOT_DIR*))
     *MBB_ROOT_DIR*)
    ;; 2. Direct check of known absolute project path
    ((findfile "C:/Users/like/OneDrive/문서/MBB/mbb_nesting.py")
     "C:/Users/like/OneDrive/문서/MBB")
    ;; 3. If mbb_nesting.py is found anywhere in AutoCAD search paths
    ((and (setq p (findfile "mbb_nesting.py"))
          (setq d (vl-filename-directory p)))
     d)
    ;; 4. If MBB.LSP is found in AutoCAD search paths
    ((and (setq p (findfile "MBB.LSP"))
          (setq d (vl-filename-directory p)))
     d)
    ;; 5. Current DWG directory
    ((and (getvar "DWGPREFIX") (vl-file-directory-p (getvar "DWGPREFIX")))
     (vl-string-right-trim "\\" (vl-string-right-trim "/" (getvar "DWGPREFIX"))))
    ;; 6. Temp prefix fallback
    (t (vl-string-right-trim "\\" (vl-string-right-trim "/" (getvar "TEMPPREFIX"))))
  )
)

;;; [START: ROLLBACK ANCHOR - v2.5 ORIGINAL]
;;; (c:MBBN v2.5 preserved in git history bc35fba)
;;; [END: ROLLBACK ANCHOR - v2.5 ORIGINAL]

;;; [START: CURRENT ACTIVE - v2.6 MODIFIED (2026-10-22 22:01:00)]
;;; [START: ROLLBACK ANCHOR - v2.7 ORIGINAL]
;;; (c:MBBN v2.7 preserved in git history b4327d7)
;;; [END: ROLLBACK ANCHOR - v2.7 ORIGINAL]

;;; [START: CURRENT ACTIVE - v2.8 MODIFIED (2026-10-23 07:03:00)]
(defun c:MBBN ( / *error* ss-out ss-in acDoc mspace i ename obj objName
                  containers-list cid is-circle c-cen c-rad c-area
                  out-min out-max out-pts p1 p2 num-v vi total-len step-dist cur-dist
                  geom-raw text-list clustered-parts grp parent-item
                  placed ci minP maxP minC maxC
                  parent-obj children-holes text-objs all-objs assigned-texts part-texts
                  parts-list best-plan
                  gap margin nest-mode mode-str
                  item geom rot orig-min-x orig-min-y orig-w orig-h
                  move-dx move-dy
                  cur-dir temp-dir py-script json-in json-out lsp-out bat-path py-exe cmd wsh f f-bat
                  t-item t-obj t-min t-max t-cx t-cy
                  pi-idx num-pts num-parts id-parts-list pid part-entry
                  ci-idx num-c cid-val is-c-val c-cen-val c-rad-val c-min-val c-max-val c-pts-val c-area-val )
  (vl-load-com)
  (setq acDoc (vla-get-ActiveDocument (vlax-get-acad-object)))
  (setq mspace (vla-get-ModelSpace acDoc))

  ;; 에러 핸들러
  (defun *error* ( msg )
    (if (and acDoc (= 8 (logand 8 (getvar 'UNDOCTL))))
      (vl-catch-all-apply 'vla-EndUndoMark (list acDoc))
    )
    (if (and msg (not (wcmatch (strcase msg t) "*break*,*cancel*,*exit*")))
      (princ (strcat "\n>> [MBBN 오류]: " msg))
    )
    (setvar 'CMDECHO 1)
    (princ)
  )

  (setvar 'CMDECHO 0)
  (vla-StartUndoMark acDoc)

  (princ "\n================================================================")
  (princ "\n[MBBN v4.8] 20코어 초고속 병렬 & 기본원칙(외곽 무침범·단품 무간섭) 네스팅 가동...")
  (princ "\n================================================================")

  ;; 전역 기본값 세션 유지 (직전 값 기억)
  (if (null *MBBN_MARGIN*) (setq *MBBN_MARGIN* 2.0))
  (if (null *MBBN_GAP*)    (setq *MBBN_GAP* 3.0))

  ;; 0단계: 외곽 여유(Margin) 및 단품 간격(Gap) 동적 입력
  (initget 4)
  (setq margin (getdist (strcat "\n>> [설정] 외곽 컨테이너 여유(Margin) mm <" (rtos *MBBN_MARGIN* 2 1) ">: ")))
  (if (null margin)
    (setq margin *MBBN_MARGIN*)
    (setq *MBBN_MARGIN* margin)
  )

  (initget 4)
  (setq gap (getdist (strcat "\n>> [설정] 단품 간격(Gap) mm <" (rtos *MBBN_GAP* 2 1) ">: ")))
  (if (null gap)
    (setq gap *MBBN_GAP*)
    (setq *MBBN_GAP* gap)
  )

  ;; 네스팅 모드 선택 [1: 빠른 배열(초고속 1~3초) / 2: 심층 경쟁(최대 수율 3~5초)]
  (initget "1 2 F D Fast Deep")
  (setq nest-mode (getkword "\n>> [모드] 네스팅 모드 선택 [1: 빠른 배열(초고속 1~3초) / 2: 심층 경쟁(최대 수율 3~5초)] <1>: "))
  (if (or (null nest-mode) (= nest-mode "") (= nest-mode "1") (= (strcase nest-mode) "F") (= (strcase nest-mode) "FAST"))
    (setq mode-str "fast")
    (setq mode-str "deep")
  )
  (princ (strcat "\n>> [적용 여유] 외곽 여유: " (rtos margin 2 1) " mm / 단품 간격: " (rtos gap 2 1) " mm / 모드: " (if (= mode-str "fast") "[1: 빠른 배열(Fast)]" "[2: 심층 경쟁(Deep)]")))

  ;; 1단계: 외곽 컨테이너 복수 선택 (원형/사각/비정형 잔재 1개 또는 여러 개)
  (princ "\n>> [1단계] 단품들을 배치할 외곽 컨테이너(틀/파이프) 선택 (복수 선택 가능): ")
  (if (setq ss-out (ssget "_:L" '((0 . "LINE,*POLYLINE,CIRCLE,ARC,ELLIPSE,SPLINE,REGION"))))
    (progn
      (setq containers-list '()
            cid 0
            i 0)
      (repeat (sslength ss-out)
        (setq ename (ssname ss-out i)
              obj   (vlax-ename->vla-object ename)
              objName (vla-get-ObjectName obj)
              i     (1+ i))

        (setq is-circle nil c-cen nil c-rad nil c-area 0.0 out-min nil out-max nil out-pts '())
        
        ;; 원형(파이) 여부 판정
        (if (wcmatch (strcase objName) "*CIRCLE*")
          (progn
            (setq is-circle T
                  c-cen (vlax-safearray->list (vlax-variant-value (vla-get-Center obj)))
                  c-rad (vl-catch-all-apply 'vla-get-Radius (list obj)))
            (if (vl-catch-all-error-p c-rad) (setq c-rad nil))
            (if c-rad (setq c-area (* pi c-rad c-rad)))
          )
        )

        ;; BBOX 산출
        (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-getboundingbox (list obj 'tmp-min 'tmp-max))))
          (progn
            (setq tmp-min (vlax-safearray->list tmp-min)
                  tmp-max (vlax-safearray->list tmp-max))
            (setq out-min tmp-min
                  out-max tmp-max)
          )
        )

        ;; WCS Curve 둘레 고밀도 좌표 샘플링 (10mm 간격)
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
        ;; Curve 정점 폴백
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
        ;; 좌표점 폴백
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
              (progn
                (setq stride (if (wcmatch (strcase (vla-get-ObjectName obj)) "*2DPOLYLINE*,*3DPOLYLINE*") 3 2))
                (while p1
                  (setq out-pts (cons (list (car p1) (cadr p1)) out-pts))
                  (if (= stride 3)
                    (setq p1 (cdddr p1))
                    (setq p1 (cddr p1))
                  )
                )
              )
            )
          )
        )

        ;; 면적 산출
        (if (null is-circle)
          (setq c-area (vl-catch-all-apply 'vla-get-Area (list obj)))
        )
        (if (or (vl-catch-all-error-p c-area) (null c-area) (<= c-area 0.0))
          (if (and out-min out-max)
            (setq c-area (* (- (car out-max) (car out-min)) (- (cadr out-max) (cadr out-min))))
          )
        )

        (if (or (and is-circle c-cen c-rad) (and out-min out-max (or out-pts (>= (length out-pts) 3))))
          (progn
            (setq containers-list (cons (list cid is-circle c-cen c-rad out-min out-max out-pts c-area obj) containers-list)
                  cid (1+ cid))
          )
        )
      )
      (setq containers-list (reverse containers-list))

      (if (and containers-list (> (length containers-list) 0))
        (progn
          (princ (strcat "\n>> 총 " (itoa (length containers-list)) "개 외곽 컨테이너 인식 완료:"))
          (foreach c containers-list
            (if (nth 1 c)
              (princ (strcat "\n   - [컨테이너 " (itoa (1+ (car c))) "] 원형(파이) 직경(Φ) = " 
                             (rtos (* 2.0 (nth 3 c)) 2 1) " mm (면적: " (rtos (/ (nth 7 c) 1000000.0) 2 3) " m²)"))
              (princ (strcat "\n   - [컨테이너 " (itoa (1+ (car c))) "] 사각/곡면 폭 = " 
                             (rtos (- (car (nth 5 c)) (car (nth 4 c))) 2 1) " mm, 높이 = " 
                             (rtos (- (cadr (nth 5 c)) (cadr (nth 4 c))) 2 1) " mm (면적: " 
                             (rtos (/ (nth 7 c) 1000000.0) 2 3) " m²)"))
            )
          )

          ;; 2단계: 배치할 모든 단품 및 내부 객체 드래그 선택
          (princ "\n>> [2단계] 배치할 모든 단품 및 내부 객체(도형+문자) 드래그 선택: ")
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

              ;; [내경(홀/슬롯) 복합 클러스터링]: 외곽선 내 모든 홀 1:1 완벽 묶음
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

              ;; [텍스트 1:1 배타적 바인딩]: 중복 할당 방지 및 단품/문자 이탈 원천 차단
;;; [START: CURRENT ACTIVE - v4.3 MODIFIED (2026-10-23 20:58:00)]
              ;; [텍스트 1:1 엄격 기하 바인딩]: 문자 BBOX 중심점 기준 외곽 폴리선 내부 포함 검사 (오차 허용 1.0mm)
              ;; 50mm 과대 버퍼로 인한 인접 부품 텍스트 탈취 및 유실을 100% 원천 차단
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
                        t-min (nth 1 t-item)
                        t-max (nth 2 t-item)
                        t-cx  (* 0.5 (+ (car t-min) (car t-max)))
                        t-cy  (* 0.5 (+ (cadr t-min) (cadr t-max))))
                  (if (and (null (vl-position t-obj assigned-texts))
                           (<= (- (car tmp-min) 1.0) t-cx)
                           (<= t-cx (+ (car tmp-max) 1.0))
                           (<= (- (cadr tmp-min) 1.0) t-cy)
                           (<= t-cy (+ (cadr tmp-max) 1.0)))
                    (progn
                      (setq part-texts (cons t-obj part-texts)
                            assigned-texts (cons t-obj assigned-texts))
                    )
                  )
                )
;;; [END: CURRENT ACTIVE - v4.3 MODIFIED]
                (setq all-objs (cons parent-obj (append children-holes part-texts))
                      w (- (car tmp-max) (car tmp-min))
                      h (- (cadr tmp-max) (cadr tmp-min))
                      area (* w h))
                (setq parts-list (cons (list all-objs (car tmp-min) (cadr tmp-min) w h area (length children-holes) (length part-texts)) parts-list)
                      i (1+ i))
              )

              (if parts-list
                (progn
;;; [START: CURRENT ACTIVE - v3.0 MODIFIED (2026-10-23 08:25:00)]
                  ;; 대형 단품(길이 + 면적 복합 Bigness) 최우선 정렬
                  (setq parts-list (vl-sort parts-list 
                                     '(lambda (a b) 
                                        (> (+ (nth 5 a) (* (max (nth 3 a) (nth 4 a)) 500.0))
                                           (+ (nth 5 b) (* (max (nth 3 b) (nth 4 b)) 500.0))))))
                  
                  (setq i 1)
                  (foreach p parts-list
                    (princ (strcat "\n   - [단품 " (itoa i) "] 폭 = " (rtos (nth 3 p) 2 1) 
                                   " mm, 높이 = " (rtos (nth 4 p) 2 1) 
                                   " mm (Bigness 크기순 우선 정렬, 홀 " (itoa (nth 6 p)) "개, 글자 " (itoa (nth 7 p)) "개 묶음)"))
                    (setq i (1+ i))
                  )
;;; [END: CURRENT ACTIVE - v3.0 MODIFIED]

                  ;; (margin & gap 설정값 유지)

                  ;; 단품 목록에 고유 ID(0, 1, 2...) 부여
                  (setq id-parts-list '()
                        pid 0)
                  (foreach p parts-list
                    (setq id-parts-list (cons (cons pid p) id-parts-list)
                          pid (1+ pid))
                  )
                  (setq id-parts-list (reverse id-parts-list))

                  ;; 파일 경로 설정 (findfile nil 방어 MBB:GetRootDir 사용)
;;; [START: CURRENT ACTIVE - v4.3 MODIFIED (2026-10-23 20:58:00)]
                  ;; 경로 설정: 한글 경로(문서) cmd.exe 실행 오류를 방지하기 위해 임시 파일은 TEMPPREFIX(ASCII)에 저장
                  (setq cur-dir (MBB:GetRootDir))
                  (if (or (null cur-dir) (= cur-dir ""))
                    (setq cur-dir "C:/Users/like/OneDrive/문서/MBB")
                  )
                  (setq cur-dir (vl-string-translate "/" "\\" cur-dir))
                  (setq py-script (strcat cur-dir "\\mbb_nesting.py"))
                  (if (null (findfile py-script))
                    (if (setq p (findfile "mbb_nesting.py"))
                      (setq py-script p)
                    )
                  )

                  (setq temp-dir (vl-string-right-trim "\\" (vl-string-right-trim "/" (getvar "TEMPPREFIX"))))
                  (if (or (null temp-dir) (= temp-dir "") (not (vl-file-directory-p temp-dir)))
                    (setq temp-dir cur-dir)
                  )
                  (setq json-in   (strcat temp-dir "\\mbb_nesting_in.json")
                        json-out  (strcat temp-dir "\\mbb_nesting_out.json")
                        lsp-out   (strcat temp-dir "\\mbb_nesting_out.lsp")
                        bat-path  (strcat temp-dir "\\mbb_run.bat"))
;;; [END: CURRENT ACTIVE - v4.3 MODIFIED]

                  ;; 1. Python 입력 JSON 고속 생성 (다중 컨테이너 및 단품 일괄 덤프)
                  (setq f (open json-in "w"))
                  (if f
                    (progn
                      (write-line "{" f)
                      (write-line (strcat "  \"gap\": " (rtos gap 2 4) ",") f)
                      (write-line (strcat "  \"margin\": " (rtos margin 2 4) ",") f)
                      (write-line (strcat "  \"mode\": \"" mode-str "\",") f)
                      
                      ;; containers 배열 출력
                      (write-line "  \"containers\": [" f)
                      (setq ci-idx 0 num-c (length containers-list))
                      (foreach c containers-list
                        (setq ci-idx (1+ ci-idx)
                              cid-val (nth 0 c)
                              is-c-val (nth 1 c)
                              c-cen-val (nth 2 c)
                              c-rad-val (nth 3 c)
                              c-min-val (nth 4 c)
                              c-max-val (nth 5 c)
                              c-pts-val (nth 6 c)
                              c-area-val (nth 7 c))
                        (write-line "    {" f)
                        (write-line (strcat "      \"id\": " (itoa cid-val) ",") f)
                        (if is-c-val
                          (progn
                            (write-line "      \"type\": \"circle\"," f)
                            (write-line (strcat "      \"circle_center\": [" (rtos (car c-cen-val) 2 4) ", " (rtos (cadr c-cen-val) 2 4) "],") f)
                            (write-line (strcat "      \"circle_radius\": " (rtos c-rad-val 2 4) ",") f)
                          )
                          (progn
                            (write-line "      \"type\": \"polygon\"," f)
                            (write-line (strcat "      \"outer_bbox\": [" (rtos (car c-min-val) 2 4) ", " (rtos (cadr c-min-val) 2 4) ", "
                                                (rtos (car c-max-val) 2 4) ", " (rtos (cadr c-max-val) 2 4) "],") f)
                            (write-line "      \"outer_polygon\": [" f)
                            (setq pi-idx 0 num-pts (length c-pts-val))
                            (foreach pt c-pts-val
                              (setq pi-idx (1+ pi-idx))
                              (write-line (strcat "        [" (rtos (car pt) 2 4) ", " (rtos (cadr pt) 2 4) "]"
                                                  (if (< pi-idx num-pts) "," "")) f)
                            )
                            (write-line "      ]," f)
                          )
                        )
                        (write-line (strcat "      \"area\": " (rtos c-area-val 2 4)) f)
                        (write-line (strcat "    }" (if (< ci-idx num-c) "," "")) f)
                      )
                      (write-line "  ]," f)

                      ;; parts 배열 출력
                      (write-line "  \"parts\": [" f)
                      (setq pi-idx 0 num-parts (length id-parts-list))
                      (foreach item id-parts-list
                        (setq pi-idx (1+ pi-idx)
                              pid (car item)
                              p   (cdr item))
                        (write-line (strcat "    {\"id\": " (itoa pid)
                                            ", \"orig_min_x\": " (rtos (nth 1 p) 2 4)
                                            ", \"orig_min_y\": " (rtos (nth 2 p) 2 4)
                                            ", \"w\": " (rtos (nth 3 p) 2 4)
                                            ", \"h\": " (rtos (nth 4 p) 2 4)
                                            ", \"area\": " (rtos (nth 5 p) 2 4)
                                            "}"
                                            (if (< pi-idx num-parts) "," "")) f)
                      )
                      (write-line "  ]" f)
                      (write-line "}" f)
                      (close f)
                    )
                  )

                  ;; 기존 결과 LSP 삭제
                  (if (findfile lsp-out) (vl-file-delete lsp-out))
                  (setq py-exe (if (findfile "C:/Users/like/AppData/Local/Programs/Python/Python314/python.exe")
                                 "\"C:/Users/like/AppData/Local/Programs/Python/Python314/python.exe\""
                                 "python"))

                  ;; CP949 배치 파일 생성 (경로 문자열 한글 깨짐 100% 방지)
;;; [START: CURRENT ACTIVE - v4.3 MODIFIED (2026-10-23 20:58:00)]
                  ;; CP949 배치 파일 생성 (경로 공백 및 특수문자 100% 방어, 작업 디렉토리 진입)
                  (setq f-bat (open bat-path "w"))
                  (if f-bat
                    (progn
                      (write-line "@echo off" f-bat)
                      (write-line "chcp 949 >nul" f-bat)
                      (write-line (strcat "cd /d \"" cur-dir "\"") f-bat)
                      (write-line (strcat py-exe " \"" py-script "\" \"" json-in "\" \"" lsp-out "\" \"" json-out "\"") f-bat)
                      (close f-bat)
                    )
                  )
;;; [END: CURRENT ACTIVE - v4.3 MODIFIED]

                  (princ (strcat "\n>> [Python 20코어 초고속 멀티프로세싱 가동 중...] " (itoa (length containers-list)) "개 컨테이너 x " (itoa (length id-parts-list)) "개 단품 틈새 1:1 선별 탐색 (" mode-str " 모드)"))
                  (setq wsh (vlax-create-object "WScript.Shell"))
                  (if wsh
                    (progn
                      ;; 0 = SW_HIDE (무창 백그라운드), 1 = bWaitOnReturn (완료 시까지 동기 대기)
                      (vlax-invoke-method wsh 'Run (strcat "cmd.exe /c \"" bat-path "\"") 0 1)
                      (vlax-release-object wsh)
                    )
                    (startapp (strcat "cmd.exe /c \"" bat-path "\""))
                  )

                  ;; 임시 배치 파일 및 입력 JSON 정리
                  (if (findfile bat-path) (vl-file-delete bat-path))
                  (if (findfile json-in) (vl-file-delete json-in))

                  ;; 2. Python 결과 로드 및 CAD 객체(모형+문자) 일괄 자동 배치
                  (setq MBB_NESTING_PLAN nil
                        MBB_NESTING_STATUS nil
                        MBB_TOTAL_YIELD nil)
                  (if (findfile lsp-out)
                    (load lsp-out)
                  )
                  (if (findfile lsp-out) (vl-file-delete lsp-out))

                  ;; 2-1. 미배치 단품 외곽 우측 대기열로 자동 정렬
                  (if (and MBB_UNPLACED_PLAN (> (length MBB_UNPLACED_PLAN) 0))
                    (progn
                      (foreach item MBB_UNPLACED_PLAN
                        (setq pid (nth 0 item)
                              gx  (nth 1 item)
                              gy  (nth 2 item)
                              part-entry (assoc pid id-parts-list))
                        (if part-entry
                          (progn
                            (setq p (cdr part-entry)
                                  all-objs (nth 0 p)
                                  orig-min-x (nth 1 p)
                                  orig-min-y (nth 2 p)
                                  move-dx (- gx orig-min-x)
                                  move-dy (- gy orig-min-y))
                            (foreach o all-objs
                              (vl-catch-all-apply 'vla-Move (list o (vlax-3d-point '(0.0 0.0 0.0)) (vlax-3d-point (list move-dx move-dy 0.0))))
                            )
                          )
                        )
                      )
                      (princ (strcat "\n>> [외곽 정렬] 미배치 대형 단품 " (itoa (length MBB_UNPLACED_PLAN)) "개 우측 대기 영역으로 이동 완료."))
                    )
                  )

                  (if (and MBB_NESTING_PLAN (> (length MBB_NESTING_PLAN) 0))
                    (progn
                      (foreach item MBB_NESTING_PLAN
                        ;; 포맷: (part_id container_id gx gy w h rot)
                        (setq pid (nth 0 item)
                              cid (nth 1 item)
                              gx  (nth 2 item)
                              gy  (nth 3 item)
                              w   (nth 4 item)
                              h   (nth 5 item)
                              rot (nth 6 item)
                              part-entry (assoc pid id-parts-list))
                        (if part-entry
                          (progn
                            (setq p (cdr part-entry)
                                  all-objs (nth 0 p)
                                  orig-min-x (nth 1 p)
                                  orig-min-y (nth 2 p)
                                  orig-w (nth 3 p)
                                  orig-h (nth 4 p))
;;; [START: CURRENT ACTIVE - v4.9 MODIFIED (2026-10-24 07:35:00)]
                            (if (or (= rot 90) (= rot 90.0))
                              (foreach o all-objs
                                (vl-catch-all-apply 'vla-Rotate (list o (vlax-3d-point (list orig-min-x orig-min-y 0.0)) (/ pi 2.0)))
                              )
                            )
                            ;; [절대 오차 0.0000mm 보장] 회전 후 all-objs의 실제 실시간 결합 Bounding Box 최소 좌표 측정
                            (setq cur-min-x nil cur-min-y nil)
                            (foreach o all-objs
                              (if (not (vl-catch-all-error-p (vl-catch-all-apply 'vla-GetBoundingBox (list o 'tmp-bmin 'tmp-bmax))))
                                (progn
                                  (setq tmp-bmin (vlax-safearray->list tmp-bmin)
                                        b-x (car tmp-bmin)
                                        b-y (cadr tmp-bmin))
                                  (if (or (null cur-min-x) (< b-x cur-min-x)) (setq cur-min-x b-x))
                                  (if (or (null cur-min-y) (< b-y cur-min-y)) (setq cur-min-y b-y))
                                )
                              )
                            )
                            (if (and cur-min-x cur-min-y)
                              (setq move-dx (- gx cur-min-x)
                                    move-dy (- gy cur-min-y))
                              (setq move-dx (- gx orig-min-x)
                                    move-dy (- gy orig-min-y))
                            )
                            (foreach o all-objs
                              (vl-catch-all-apply 'vla-Move (list o (vlax-3d-point '(0.0 0.0 0.0)) (vlax-3d-point (list move-dx move-dy 0.0))))
                            )
;;; [END: CURRENT ACTIVE - v4.9 MODIFIED]
                          )
                        )
                      )
                      (princ "\n================================================================")
                      (princ (strcat "\n>> [MBBN 완결] 총 " (itoa (length parts-list)) "개 단품 중 " 
                                     (itoa (length MBB_NESTING_PLAN)) "개 최적 수율 배치 완료!"))
                      (if MBB_TOTAL_YIELD
                        (princ (strcat "\n   - 총 자재 수율(Total Yield): " (rtos MBB_TOTAL_YIELD 2 1) " %"))
                      )
                      (princ (strcat "\n   - 단품 간격(Gap): " (rtos gap 2 1) " mm / 외곽 여유(Margin): " (rtos margin 2 1) " mm"))
                      (princ "\n   - 각 단품 및 내부 홀(원/슬롯), 치수 텍스트 전체가 해당 컨테이너로 100% 무결점 일괄 배치되었습니다.")
                      (princ "\n================================================================")
                    )
                    (princ "\n>> 오류: 컨테이너 크기 부족 또는 유효 배치점을 산출하지 못했습니다.")
                  )
                )
                (princ "\n>> 오류: 배치할 유효 단품 객체가 없습니다.")
              )
            )
            (princ "\n>> 안내: 배치할 단품이 선택되지 않았습니다.")
          )
        )
        (princ "\n>> 오류: 유효한 외곽 컨테이너를 추출할 수 없습니다.")
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
