# 신규 State Estimator 인수인계 노트 (`new-state-estimator` 브랜치)

작성일: 2026-08-13, 2026-08-14 업데이트(원격 pull 반영) · 목적: 담당 인턴 퇴사에 따른 업무 인수인계
관련 문서: `NEW_STATE_ESTIMATOR_REVIEW.md`(전체 아키텍처/수식 분석, PDF 버전 있음),
`PR_REVIEW_FINDINGS.md`(PR 코멘트용으로 정리된 초기 항목 1~11 — 이 문서가 최신판이므로 상충 시 이 문서 우선)

이 문서는 위 문서들 + 클래스 단위로 하나씩 짚어본 대화 내용을 **하나로 합쳐서, 다음 사람이 이어받기
쉽게** 정리한 것이다. "각 클래스가 뭘 하는지" + "뭘 고쳐야 하는지"를 클래스별로 나란히 둔다.

**⚠️ 2026-08-14 브랜치 업데이트 중요 공지**: 로컬 브랜치가 원격보다 15개 커밋 뒤처져 있어 `git pull
origin new-state-estimator`로 fast-forward 했다(충돌 없음). 이 pull에 **PR #1421
(`new-estimator/joint-kf-split-impl`)** 이 포함되어 있어, 기존에 리뷰했던 통짜 `JointLevelKFPreFilter.java`
(2586줄)가 **`JointKFState`/`JointKFPrediction`/`JointKFUpdate`/`JointKFBiasUpdate`/`JointKFParameters`
5개로 쪼개졌다** — §2-24번(옛 문서, "클래스가 너무 크다")이 실제로 반영된 것. 아래 §1.1/§2는 전부 이
새 구조 기준이다. **작업 시작 전에 반드시 로컬 브랜치가 원격과 동기화돼 있는지 확인할 것.**

---

## 0. 아키텍처 한 줄 요약

```
관절/IMU 원시값 → JointLevelKFPreFilter (오케스트레이터, §1.1 참고)
                       │ (bias-corrected ω, a)
                       ▼
                  InvariantEKF (SE_k(3) 우불변 EKF: base pose/vel + 접촉점 N개)
                       │
        ┌──────────────┴──────────────┐
        ▼                              ▼
InvariantEKFStateEstimator      InvariantMainStateEstimator
(보조/평가용, controller 안 구동)  (메인 승격, rootJoint/관절/CoM 실제로 씀)
```

기존 DRC 추정기(`PelvisLinearStateUpdater` + `JointStateUpdater`)는 이 브랜치에서 **삭제되지 않고
그대로 남아있으며**, `AlphaComplementaryPreFilter`라는 얇은 어댑터로 새 인터페이스 뒤에 감싸져
`SwitchableJointLevelSource`를 통해 새 `JointLevelKFPreFilter`와 **런타임 A/B 전환**이 가능하다
(다만 §2-19번 — 이 전환은 한쪽 방향에서만 배선돼 있음).

---

## 1. 클래스별 레퍼런스

### 1.1 관절 레벨 (`jointLevel` 패키지) — pull 이후 구조

`JointLevelKFPreFilter`가 순수 오케스트레이터로 축소되고(595줄), 실제 로직은 5개 협력자로 분리됐다.
의존 방향은 한쪽으로만: `BiasUpdate → Update → State` (순환 없음).

| 클래스 | 역할 | 핵심 포인트 |
|---|---|---|
| **`JointLevelKFPreFilter`** | 오케스트레이터 (595줄) | 5개 협력자를 조립하고 `ProprioceptivePreFilter` 인터페이스를 구현. `initialize()`/`computeJointState()`(Phase1)/`computeImuBiases()`(Phase2) 및 on-ground 초기화 게이트 소유. `setMatrix`/`symmetrize`/`containsNonFinite` 공유 정적 유틸도 여기. |
| **`JointKFParameters`** | 튜닝값을 전부 `YoDouble`로 노출 | `sigmaTau`, `anchorVar`, `sigmaGyroFloor`, `qaMax`, `condSMax` 등 재빌드 없이 라이브 튜닝 가능. **단, 일부는 "LIVE"라고 문서화돼 있지만 실제로는 생성자에서 한 번만 읽혀서 라이브하지 않음** — §2-14번. |
| **`JointKFState`** | 상태 레이아웃 $x=[q;\dot q;b_\omega]$과 구조 홀더(`Pair`, `FootAnchor`) | `jointsByIndex`/`imusByOrdinal`(배열)가 상태 순서 그 자체, 역방향 조회 맵은 private(절대 순회 금지). 부팅 시 acyclic-IMU-graph 검증, "usable anchor 없으면 부팅 거부" 안전장치 있음. |
| **`JointKFPrediction`** | $F$, $Q$(질량행렬 Schur complement) | $\lambda=M_{ff}-M_{Nf}^\top X$, $Q_a=YY^\top$. `symmetrize(lambda)` 이미 적용됨(§2-8과 대조). |
| **`JointKFUpdate`** | 인코더/직접속도 채널 + 공유 Joseph 코어 | `Channel` enum(ENCODER/ENCODER_VELOCITY/STACKED_GYRO/TEST)으로 문자열 라벨 매칭을 대체. `JointKFBiasUpdate`의 stacked gyro 블록도 이 클래스의 `josephUpdate()`를 공유. 직접속도 채널은 lag-aware 적응형 $R$. |
| **`JointKFBiasUpdate`** | Stacked gyro 업데이트(pair + stance anchor) | 유일하게 IMU 바이어스를 절대값으로 관측하는 채널(`+I3` anchor). `computeImuBiases`는 이제 순수 부기(다음 tick 앵커용 신뢰 발 캐싱)로 축소. |
| **`ProprioceptivePreFilterFactory`** | `NONE`/`ALPHA_COMPLEMENTARY`/`JOINT_KF` 중 선택 조립 | default 없는 exhaustive switch(새 enum 값 추가 시 컴파일 에러 강제). §2-19번(라이브 전환 비대칭)과 연결됨. |
| **`AlphaComplementaryPreFilter`** | 옛 DRC 로직 어댑터 | **새 알고리즘 없음** — 원본 클래스(`IMUBasedJointStateEstimator`+`IMUBiasStateEstimator`)는 그대로 두고 위임 계층만 추가. |
| **`SwitchableJointLevelSource`** | JointKF ↔ Alpha 런타임 전환 | 둘 다 매 tick 계속 tick(warm 유지, bumpless 전환). **IMU 바이어스는 선택과 무관하게 항상 JointKF 고정** — §2-20번(해석 주의). |
| **`ZeroIMUBiasProvider`** | Null-object 바이어스 제공자 | 항상 0 반환. 특별한 이슈 없음. |

**기존 대비 핵심 차이** (변동 없음): 옛 방식은 IMU-pair 자이로 차분을 매 tick 즉석 최소자승으로 풀고
인코더와 고정 알파로 블렌딩(칼만필터도, 바이어스 상태도 없음). 새 방식은 칼만필터화해서 바이어스를
상태에 넣고 신뢰도(공분산)를 명시적으로 추적.

### 1.2 몸체(Base) 레벨 — SE_k(3) InEKF (`invariantEstimator` 패키지, 변동 없음)

| 클래스 | 역할 | 핵심 포인트 |
|---|---|---|
| **`InvariantState`** | $X$(그룹원소)와 $P$(공분산) 저장 | 순수 데이터 클래스. $X$는 $n{=}5{+}N$ 행렬, $P$는 $m{=}9{+}3N$ 접선공간. |
| **`SEK3Utils`** | SE_k(3) 리군 exp/log/adjoint | euclid의 `SO3LieGroupTools`/`SE3LieGroupTools`(k=1)의 일반화판인데 euclid가 아니라 이 저장소에 있음 — §2-17. |
| **`InvariantPropagator`** | predict 단계 | $\Phi$가 상태 무관(우불변 필터 핵심 이점). $P$ 대칭화 누락 — §2-9. |
| **`InvariantUpdater`** | 범용 Joseph-form 보정 (+ conditioning gate, NIS) | 접촉 업데이트와 중력 레벨링이 이 메서드를 공유. |
| **`ContactUpdater`** | 접촉 FK 측정 서브피스 | $H$가 회전 블록 0 — 접촉만으론 자세를 못 봄. 테스트(`ContactUpdaterTest`)가 오라클 기반으로 매우 꼼꼼함, 실질적 결함 없음. |
| **`GravityLevelingUpdater`** | 가속도계 기반 roll/pitch 관측 | 상보필터 참조($\tau=5$s)로 밸런스 컨트롤러와의 양성피드백 방지. **NaN 가드 없음 — §2-4, 최우선 수정 대상.** |
| **`InvariantEKF`** | 배선 전용 오케스트레이터 | `reseedContact()`, `setRotation()`(yaw corrector 훅) 여기 있음. |
| **`FootSwitchContactProbabilityProvider`** | 접촉확률 $p_i$의 실제 원천 | Schmitt trigger(2026-07-17)로 CoP-사각지대 이중 트리거 버그 해결. 물리 상수 하드코드 — §2-11. |
| **`KinematicContactDetector`** | 접촉확률 fallback | main 모드에선 순환참조 위험. |
| **`InvariantContactSource`** | 위 둘 선택 enum | `FOOT_SWITCHES` 기본값. |
| **`FootReferencedYawCorrector`** | yaw drift 완화 휴리스틱 | heuristic seed일 뿐, observability fix 아님. |
| **`TouchdownReseedLatch`** | 재접지 스텝당 최대 1회 제한 | 생성자 NaN 검증 누락 — §2-10. |
| **`InvariantCenterOfMassUpdater`** | CoM 순수 운동학적 계산 | GRF 융합 없음, 저자도 "교체 예정 자리"라 명시. |
| **`InvariantEKFStateEstimator`** | InEKF 실행 + 방대한 진단 (986줄) | 로직은 `doControl()` ~130줄, 나머지는 YoVariable 배선. |
| **`InvariantMainStateEstimator`** | 최종 조립체 | §1.3 표 참고. |

### 1.3 `InvariantEKFStateEstimator` vs `InvariantMainStateEstimator` 차이

| | EKFStateEstimator | MainStateEstimator |
|---|---|---|
| rootJoint/관절 씀 | ❌ (계산만) | ✅ |
| Yaw corrector, CoM, 컨트롤러향 필터 | ❌ | ✅ |
| 관절 프리필터 오케스트레이션 | ❌ | ✅ |
| 관계 | — | 전자를 내부 엔진으로 소유 |

### 1.4 기존(DRC) 대비 변경점 총정리

| | 기존 | 신규 |
|---|---|---|
| 관절 추정 | 즉석 최소자승 + 고정 알파 블렌딩 | 칼만필터, 바이어스가 상태 |
| 몸체 오차 모델 | 가법적(additive) | 우불변(right-invariant), $\Phi$가 상태 무관 |
| 접촉 신뢰 | 이진(trust) | 연속 확률 $p\in[0,1]$, $c^{1-p}$ 스케일링 |
| 프로세스 잡음 | 고정 | 질량행렬 기반, 형상/접촉확률 의존 |
| yaw | 암묵적 신뢰 | 관측 불가를 설계로 인정, 별도 휴리스틱 보정기 존재 |

---

## 2. 수정/확인이 필요한 항목 전체 목록 (우선순위순, 2026-08-14 기준 재정리)

### 🔴 병합 전 반드시 확인

1. **`SIGMA_TAU` 재조정 미완료 — 여전히 유효.** `JointKFPrediction.java:405-407`에 `TODO(retune)` 주석이 그대로 있음. **좋은 소식**: `JointKFParameters`로 라이브 튜닝은 가능해짐(재빌드 불필요) — 하지만 "라이브로 바뀌었다"≠"재교정됐다". NIS 기반 재교정 여부 확인 필요.
2. **베이스 IMU/관절 선정이 검증 없는 구조적 가정 — 두 곳** (구 항목 #2가 확장됨):
   - `JointKFPrediction.java:567-575` `findFloatingBaseJoint`: 트리 루트의 6-DoF 자식을 base로 가정 (DoF 기반, 그나마 구조적).
   - **`JointKFState.java:206-212`: `baseIMU = pairs.get(0).parent` — pair 설정 리스트의 "첫 번째" 항목이라는, 순전히 순서에만 의존하는 훨씬 더 취약한 가정. 저자 스스로 `//WARNING: Need to configure if the root differs.`라고 남김.** 리스트 순서가 바뀌면 조용히 잘못된 IMU가 base로 지정됨. **이번 재검토에서 우선순위를 가장 높게 올린 항목.**
3. **Gap-joint marginalize vs lock 불일치** — `JointKFPrediction.java`(구 `JointLevelKFPreFilter.java:685`). SPEC은 lock, 구현은 marginalize. 토폴로지 검증(assert) 추가 권장.

### 🟠 실제 버그 가능성 있음

4. **`GravityLevelingUpdater.updateGravityReference`에 NaN 가드 없음** — `GravityLevelingUpdater.java:350-375`. `accelNorm < 1.0e-9`가 NaN을 못 거름 → IMU 글리치 한 번이 `gravityReferenceBody`를 영구 오염(IIR이라 회복 불가). `InvariantUpdater`의 conditioning gate는 residual을 안 봐서 이것도 못 거름. **이번 리뷰 전체에서 가장 실질적인 버그 후보 — 최우선 수정 권장.**
   ```java
   if (!Double.isFinite(accelNorm) || accelNorm < 1.0e-9) return;
   ```
5. **부팅 시 `computeJointState()`가 실제 경과시간 없이 최대 3번 연속 호출** — `InvariantMainStateEstimator`의 `initializeEstimator()`→`initialize()`→`doControl()` 순서로 첫 tick에 연속 호출, 매번 `predict()` 실행(가드 없음). 부팅 시 공분산이 최대 3배 부풀려질 수 있음. 의도된 것인지 확인 필요.
6. **`AlexEstimatorLogReplay`(별도 저장소 `alex`)가 `buildAndFloorSigma`의 타이밍 계약을 잘못 알고 있음** — `JointKFBiasUpdate.java:353-357`이 "Sigma는 생성자가 아니라 첫 `buildStackedMeasurement()`에서 lazy하게 굳는다"고 명시하며 "`AlexEstimatorLogReplay`가 이 오류를 반복한다"고 저자가 직접 지적. 실제로 `alex/src/main/java/us/ihmc/alex/logAnalysis/AlexEstimatorLogReplay.java:115`에 "construction, so ... buildAndFloorSigma sees them"이라는 **틀린 가정**이 그대로 남아있음(확인됨). A/B 노이즈 오버라이드 실험이 의도대로 안 먹힐 수 있음. **`ihmc-open-robotics-software`가 아니라 `alex` 저장소를 고쳐야 하는 크로스 레포 항목** — 놓치기 쉬움.

### 🟡 설계 권고 (버그는 아님)

7. 접촉 인플레이션 상수 `c=90` 근거 미문서화 — `InvariantEKFStateEstimator.java:157-158`.
8. Conditioning gate가 자체 이력 없이 이진 스킵 — `InvariantUpdater.java:184-189`.
9. **`InvariantPropagator.predictCovariance`가 $P$를 명시적으로 대칭화 안 함** — `InvariantPropagator.java:184-188`. **대조 근거 추가**: `JointKFUpdate.java:393`와 `JointKFPrediction.java:470`는 이미 `symmetrize()`를 쓰고 있어, 이 패턴이 코드베이스에서 이미 검증된 관례임이 재확인됨. `symmetrize()`를 공유 유틸로 뽑아서 `InvariantPropagator`에도 적용 권장.
10. **생성자 검증 비일관 — 여러 클래스에 반복**:
    - `InvariantPropagator`: 분산 음수 체크 없음.
    - `TouchdownReseedLatch.java:36-37`: NaN이 `rearmProbability >= triggerProbability` 체크를 통과함 → 조용히 "영원히 재접지 안 됨" 상태 가능.
    - `FootSwitchContactProbabilityProvider`: `smoothingAlpha`/임계값 범위 검증 없음.
11. **"LIVE" 파라미터가 실제로는 생성자에서만 읽히는 문제 — 최소 3곳에서 반복되는 시스템적 패턴** (신규 발견, 구 항목 #14 확장):
    - `JointKFState.java:217-218`: `sigmaQdUnfiltered`(foot anchor 폴백용).
    - `JointKFUpdate.java:168`: `encoderVar`(인코더 폴백용).
    - `JointKFUpdate.java:195-196`: `sigmaQdUnfiltered`를 **여기서 또 한 번 별도로** 생성자 시점에 읽음.
    - `FootSwitchContactProbabilityProvider`: 35%/25%/40ms/α=0.8 전부 하드코드.
    - `GravityLevelingUpdater.GRAVITY_REFERENCE_TIME_CONSTANT=5.0`도 하드코드.
    - **권장 조치**: `JointKFParameters`의 모든 필드를 감사(audit)해서 "매 tick 재읽음" vs "생성자에서만 읽음"을 표로 정리하고, "LIVE"라고 문서화된 것 중 실제로 안 그런 게 있으면 통일할 것.
12. **`JointKFBiasUpdate.cacheTrustedFeet()`의 ArrayList 용량 초과 위험** — `JointKFBiasUpdate.java:104,130-138`. `trustedFeet.size()`가 `footAnchors.size()`(사전 할당 용량)를 넘으면 실시간 스레드에서 힙 재할당 발생. 지금 Alex(2발 고정)엔 무해하나 방어 코드 없음.
13. **`JointKFUpdate.Channel.fromLabel()`이 인식 못 한 라벨을 조용히 `TEST`로 처리** — `JointKFUpdate.java:67-73`. 문자열 오타가 예외 없이 잘못된 채널로 새어들어갈 수 있음. `throw new IllegalArgumentException(...)`으로 교체 권장.
14. **`JointKFPrediction`의 `MatrixTools.addDiagonal` 관련 미해결 TODO** — `JointKFPrediction.java:476-479`. `addDiagonal`을 루프 안에서 다른 스칼라로 호출하면 "전체 대각선에 매번 누적"되는 방식일 가능성이 있어(테스트가 실제로 깨졌다고 주석에 명시) 지금은 수동 루프로 우회 중. 우회 코드 자체는 정상 동작하는 것으로 보이나, `MatrixTools.addDiagonal`의 실제 시그니처 확인 및 주석 정리 권장.
15. `InvariantUpdater`가 가변 측정 크기에서 allocation-free 아님을 자체 인정 — `InvariantUpdater.java:29-31`. Alex(N=2 고정)엔 무해, 재사용 시 함정.
16. 자이로 바이어스 클램프 카운터가 아무 데도 안 읽힘 — `InvariantEKFStateEstimator.java:104,240,527`. 상위 알람 배선 권고.
17. 관절 KF ↔ InEKF 진단 상관관계 없음 — 합성 진단 YoBoolean 하나 권고.
18. **`InvariantState.setRotation`이 `JointLevelKFPreFilter.setMatrix`(다른 패키지)에 의존 — 아직 해결 안 됨.** `InvariantState.java:122`. 메서드 이름은 `set_matrix`→`setMatrix`로 camelCase 정정됐지만(§2-22 참고), **크로스 패키지 결합 자체는 그대로**. `JointLevelKFPreFilter.java:368-369` 주석이 이 의존을 스스로 인정: "main-source code outside this package already calls (invariantEstimator/InvariantState)". `SEK3Utils` 같은 공용 유틸로 옮기는 걸 권장.
19. **`ProprioceptivePreFilterFactory`/`AvatarEstimatorThreadFactory`의 라이브 전환 배선이 비대칭** (신규 발견) — `JOINT_KF`를 초기 선택하면 `AvatarEstimatorThreadFactory`가 추가로 `AlphaComplementaryPreFilter`를 만들어 `SwitchableJointLevelSource`로 감싸 라이브 전환이 가능해지지만, `ALPHA_COMPLEMENTARY`를 초기 선택하면 이 래핑이 아예 안 일어나 **전환 불가**(영구 알파 고정). 의도된 것인지 확인 필요. 관련 미해결 TODO도 있음: `ProprioceptivePreFilterFactory.java:55` `//TODO: Robert wants to move away from factories...` — 팀 내 논의가 아직 안 끝난 것으로 보임, 팀에 확인 필요.
20. **`SwitchableJointLevelSource`에서 `ALPHA_COMPLEMENTARY` 선택 시 순수 옛 파이프라인이 재현되는 게 아님(해석 주의)** (신규 발견) — IMU 바이어스가 선택과 무관하게 항상 `jointKF`에서 나오므로(`SwitchableJointLevelSource.java:111-121`), `ALPHA_COMPLEMENTARY` 모드는 "옛 관절 추정 + 새 KF 바이어스"라는 하이브리드다. A/B 테스트 결과 해석 시 오해 소지 있음 — 클래스 Javadoc에 명시 권장.
21. **`SEK3Utils`의 저장소 위치 재검토 여지** — euclid 저장소(`lie-group-additions` 브랜치)에 이미 `SO3LieGroupTools`/`SE3LieGroupTools`(k=1)가 있고, 커밋 로그(`f6b2a6f6`)를 보면 원저자가 "SEK3_Utils를 위해" euclid를 고치러 갔던 흔적까지 있는데 정작 `SEK3Utils`(k≥1 일반화판) 자신은 euclid로 안 옮겨짐. 당장 옮기라는 건 아니고, 이관 계획이 있는지 질문형으로 확인 권장.

### 🟢 스타일 / nit (동작 무관)

22. FQN 인라인 호출 — `AvatarEstimatorThreadFactory.java:490,492,500`. import 3줄 추가로 해결.
23. `InvariantState.java:109-111`에 죽은 주석 코드 남아있음 — 삭제만 하면 됨.
24. `InvariantState.java:233` 주석의 구간 표기 오류(`[0,N]`→`[0,N)`) — 한 글자만 수정.
25. **`AlphaComplementaryPreFilter.java:77-78`과 `JointLevelKFPreFilter.java:206-208` 둘 다** null 파라미터 가드에 `UnsupportedOperationException`을 씀(후자는 "Mirrors AlphaComplimentaryFilter..." 주석이 확인해주듯 의도적으로 복제된 패턴) — `IllegalArgumentException`이 맞는 타입. **두 파일 동시 수정 권장.** `JointLevelKFPreFilter.java:206`엔 저자 자신의 `//TODO: this function should be removed and the factory should handle this part.`도 있어 팀에 방향성 확인 필요.
26. `InvariantContactSource`의 스위치문이 두 case 간 비대칭 — `AvatarEstimatorThreadFactory.java:518-525`. 명시적으로 만드는 게 안전.
27. `InvariantEKF.reseedContact()` 내부에서 로컬 `Vector3D` 즉석 할당(177줄), 미리 할당된 스크래치 필드 4개를 두고도 — 재접지가 드물게만 실행돼 심각하진 않음.
28. `InvariantEKFStateEstimator`(986줄) 진단 YoVariable 선언부가 로직 대비 너무 큼 — 후속 리팩터링으로 분리 검토(별도 이슈로, 이번 PR엔 비권장).
29. `JointLevelKFPreFilter.java:195` 주석 오타 2개(`AlphaComplimentaryFilter`, `createForKinematicsEstiamtor`) — 동작 무관.
30. `ContactUpdaterTest.java:29`에 스텁 시절 문구("fill in the @Test bodies")가 그대로 남음 — 실제로는 9개 테스트 전부 완성돼 있어 오해 소지. 삭제 또는 정정 권장.

### ✅ 이번 pull로 이미 해결된 것 (참고용, 재작업 불필요)

- `JointLevelKFPreFilter`(2586줄)가 5개 클래스로 분리됨 — 구 §2-24("클래스가 너무 크다") 해결.
- `set_matrix` → `setMatrix`로 camelCase 정정됨 — 단, 크로스 패키지 결합 자체는 §2-18로 여전히 유효.
- `SIGMA_TAU`를 포함한 다수 상수가 `JointKFParameters`를 통해 라이브 튜닝 가능해짐 — 단, 재교정 완료 여부는 별개(§2-1).
- 자이로 측정잡음 0-플로어는 이미 완화되어 있었음(구 §A.4, 변동 없음).

---

## 3. 다음 사람이 시작할 때 참고할 것

- **작업 시작 전 필수**: `git fetch origin --prune && git status` 로 로컬 브랜치가 원격 `new-state-estimator`와 동기화됐는지 확인. 이번에도 15개 커밋이 밀려 있었음 — 자주 벌어지는 일로 보임.
- **필독 순서**: 이 문서 → `NEW_STATE_ESTIMATOR_REVIEW.md`(수식 포함 전체 분석, PDF 있음) → `jointLevel/CHANGES.md`(관절 KF 설계 이력) → `invariantEstimator/CONTACT_DETECTION.md`(접촉확률 설계 이력).
- **테스트 지도**: `TEST_SUITE_MAP.md` 참고 (단, pull 이후 테스트 파일 구성이 바뀌었을 수 있어 최신 상태 재확인 권장 — `JointKFLiveParameterTest.java`, `JointLevelKFStateOrderPropertyTest.java` 등 신규 테스트 파일 존재 확인됨).
- **자동 코드리뷰 결과**: `ihmc-state-estimation/src/main` 34개 파일 기준 정확성 버그 0건(2026-08-13, effort=high, pull 이전 스냅샷). 위 목록은 그 이후 수동으로 클래스 하나씩 짚으며 추가로 찾은 것들이며, pull 이후 파일들은 아직 자동 리뷰를 다시 안 돌림 — 필요시 재실행 권장.
- **가장 먼저 손댈 것 추천**:
  1. §2-4 (`GravityLevelingUpdater` NaN 가드) — 코드 3줄, 위험이 가장 명확함.
  2. §2-2 (`JointKFState.baseIMU = pairs.get(0).parent`) — 코드 수정은 조금 더 필요하지만, 저자 스스로 WARNING을 남긴 구조적 취약점이라 우선순위가 높음.
  3. §2-1/§2-3 (재조정/토폴로지 검증)은 코드가 아니라 확인·커뮤니케이션 작업.
