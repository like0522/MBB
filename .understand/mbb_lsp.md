# MBB AutoLISP 프론트엔드 완벽 해부 (MBB.LSP)

> **파일 명칭**: `MBB.LSP`  
> **최종 수정 일시**: 2026-10-26 22:34:14  
> **인코딩 표준**: ANSI (CP949)  
> **설계 표준**: 25년 경력 CAD/AutoLISP 수석 엔지니어 규격 (Local 변수 완전 격리, Undo 트랜잭션 보장, WCS 좌표계 방어)

---

## 1. 개요 및 역할 (Overview & Role)
`MBB.LSP`는 AutoCAD 사용자 환경에서 실행되는 통합 프론트엔드입니다. 도면 내 엔티티(경계선, 폴리선, 텍스트, 치수)를 직접 다루며, 사용자와 상호작용(DCL UI / Command)하고, 백엔드 Python 엔진(`mbb_nesting.py`, `mbbg_align.py`)과 데이터를 완벽하게 교환합니다.

---

## 2. 제공 명령어 일람 (User Commands Catalog)

| 명령어 | 주요 기능 및 역할 | 연동 백엔드 / 특징 |
| :--- | :--- | :--- |
| **`c:MBBN`** | **통합 2D 판재 네스팅 엔진 (핵심)**<br>잔재 컨테이너와 부재들을 일괄 선택하여 네스팅 수행 | Python `mbb_nesting.py` 연동 (WScript.Shell)<br>JSON 입출력 및 결과 LISP 자동 로드 |
| **`c:MBBG`** | **Group-Box 정렬 및 정밀 BBOX 생성**<br>도면 내 분산된 부재 그룹들을 사각 박스로 정렬 | Python `mbbg_align.py` 연동<br>20-Core 병렬 회전 정렬 |
| **`c:MBB`** | **최소 경계 상자(Minimum Bounding Box) 생성**<br>단품의 최소 외접 사각 박스 및 중량/단중 자동 표기 | AutoLISP 자체 연산<br>면적, 무게, 텍스트 태깅 |
| **`c:MBB1`** | **단일 객체 클러스터링 및 계층형 BBOX**<br>인접한 객체들을 하나의 부재로 자동 클러스터링 | AutoLISP 자체 클러스터링 알고리즘 |
| **`c:MBB2`** | **두께(Thickness) 입력형 강판 정밀 BBOX**<br>철판 두께(T)를 입력받아 실 중량(kg) 정밀 산출 | 비중 7.85 적용 강판 무게 자동 산출 |
| **`c:MBBC`** | **최소 외접원 (Minimum Enclosing Circle, MEC)**<br>부재를 모두 포함하는 가장 작은 원형 잔재 계산 | Welzl 알고리즘 AutoLISP 자체 구현 |
| **`c:MBBCI`** | **원호/곡선 세그먼트 정밀 분석 MEC**<br>곡선 정점들을 샘플링하여 진원형 컨테이너 산출 | 정밀 원호 세그먼트 분할 |
| **`c:MBBW`** | **Region 기반 최소 외접원 및 면적 분석** | AcDbRegion 변환 기반 기하 분석 |
| **`c:PCLOSE`** | **열린 폴리선 일괄 강제 닫기 (Force Close)**<br>미세하게 열린 선분을 허용오차 내에서 결합 및 닫힘 처리 | 네스팅 전 지오메트리 위생화 필수 유틸 |
| **`c:MBBUI`** | **DCL 대화상자 기반 그래픽 사용자 인터페이스**<br>초보자도 클릭 한 번으로 모든 옵션 제어 | DCL 대화상자 동적 생성 및 호출 |

---

## 3. 핵심 서브루틴 및 기하 추출 엔진

### 3.1 원호/Bulge 정밀 샘플링 (`MBB:GetPartContourEx`)
AutoCAD 폴리선(`AcDbPolyline`)의 원호 세그먼트(Bulge)를 정밀한 다각형 좌표로 근사 변환하여 Python Shapely로 전달합니다.
- **Bulge 판정**: `(assoc 42 ent)` 값이 0이 아니면 원호 구간으로 인식.
- **정밀 분할**: 원호의 곡률에 맞추어 12~24개 정점으로 균등 분할 추출.
- **좌표계 변환**: `(trans pt ename 0)` 및 `(trans pt 1 0)`을 통해 OCS/UCS 왜곡을 원천 차단하고 순수 WCS 좌표로 변환.

### 3.2 안전 트랜잭션 및 에러 핸들링
```lisp
(defun c:MBBN ( / *error* ss-out ss-in acDoc mspace ... )
  (defun *error* (msg)
    ;; 시스템 변수 복원
    (if old-osmode (setvar "OSMODE" old-osmode))
    (if old-cmdecho (setvar "CMDECHO" old-cmdecho))
    ;; 단일 트랜잭션 롤백
    (if acDoc (vla-EndUndoMark acDoc))
    (princ (strcat "\n[MBBN 중단] " msg))
    (princ)
  )
  ;; 단일 트랜잭션 시작
  (setq acDoc (vla-get-ActiveDocument (vlax-get-acad-object)))
  (vla-StartUndoMark acDoc)
  ...
  ;; 정상 종료 시 트랜잭션 완료
  (vla-EndUndoMark acDoc)
)
```

### 3.3 백엔드 Python 오프로드 통신 규격
AutoLISP에서 Python을 실행할 때 도스창(Black CMD Window) 깜빡임을 없애고 CAD 정지를 방지하기 위해 `WScript.Shell`을 사용합니다.
```lisp
(setq wscript (vlax-create-object "WScript.Shell"))
;; 0: 도스창 숨김(Hidden), 1: 완료 대기(bWaitOnReturn = True)
(vlax-invoke-method wscript "Run" cmd-str 0 1)
(vlax-release-object wscript)
```
- 실행 완료 후 결과 파일(`mbb_nesting_out.lsp`)이 정상 생성되었는지 `findfile`로 확인하고, 즉시 `(load result_lsp)`를 호출하여 전역 리스트 `MBB_NESTING_PLAN`을 메모리에 적재합니다.

---

## 4. 배치 결과 CAD 반영 메커니즘 (Entity Transformation)

1. **객체 그룹핑 보존**:
   - 단품의 외곽 폴리선과 내부 텍스트(예: `700-1_16tx110x120- 4 EA`)는 동일한 바운딩 박스 중심을 기준으로 1:1 결합되어 함께 이동합니다.
2. **2D 평면 변환 (`vla-TransformBy` / `vla-Move` & `vla-Rotate`)**:
   - `rot = 90`인 경우 원점을 기준으로 90도 회전(`(/ pi 2)`) 후 대상 (gx, gy) 좌표로 평행 이동.
   - 텍스트 객체 역시 회전 각도에 맞추어 `Alignment` 기준점을 유지하며 도면 가독성을 보존합니다.
