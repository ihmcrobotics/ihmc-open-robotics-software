# PR 리뷰 상세 항목 — `new-state-estimator` vs `develop`

작성일: 2026-08-13 · 대상 브랜치: `new-state-estimator` (base: `develop`)
리뷰 범위: `ihmc-state-estimation/src/main` (자동 코드리뷰, effort=high) + 수동 검토
자동 코드리뷰(정확성 버그 기준) 결과: **런타임 정확성 결함 0건**. 아래 항목은 정확성 버그가 아니라
"미완료 재조정/검증", "설계상 권고", "스타일" 항목이며, PR 승인 전 확인이 필요한 순서로 정리했다.

---

## A. 병합 전 확인 필요 (원저자가 코드에 TODO로 남긴 것)

### 1. `SIGMA_TAU` 재조정 누락

**위치**: `jointLevel/JointLevelKFPreFilter.java:82`

```java
private static final double SIGMA_TAU = 5.0;
```

**문제**: 이 값은 Rev.1(locked-base) 시절 캘리브레이션된 값인데, Rev.2에서 프로세스 노이즈 공식이

$$
Q_a=\sigma_\tau^2 M_{jj}^{-2} \;\longrightarrow\; Q_a=\sigma_\tau^2\Lambda^{-2},\qquad
\Lambda=M_{jj}-M_{jb}M_{bb}^{-1}M_{bj}
$$

로 바뀌었다 (`updateProcessNoiseFromMassMatrix`, 1285줄 부근 주석에 `TODO(retune): SPEC §8`로 직접
명시). $\Lambda \preceq M_{jj}$이므로 $\Lambda^{-2}\succeq M_{jj}^{-2}$ — 즉 **같은 5.0이라는 숫자를
그대로 썼는데 실제 유효 프로세스 노이즈는 커졌다.** 필터가 예전 대비 "관절이 더 자유롭게 움직인다"고
과신하게 되어, quiet-standing에서는 필요 이상으로 인코더를 덜 믿고 노이즈를 흡수할 수 있다.

**어떻게 고쳐야 하나**: 코드 수정이 아니라 **재교정(calibration)** 문제. Quiet-standing/걷기 로그에서
자이로 블록 NIS(`getNormalizedInnovationSquared`)를 뽑아 $\chi^2$ 밴드 안에 들어오도록
`SIGMA_TAU`(및 111-121줄의 per-joint `ALPHA_VALUES` 배열)를 재조정해야 한다. 이를 하지 않고
머지한다면, 최소한 PR 설명/코드 주석에 "재조정 전"이라는 경고를 명확히 남기도록 요청할 것.

---

### 2. 베이스 관절 식별 가정이 테스트로 검증 안 됨

**위치**: `jointLevel/JointLevelKFPreFilter.java:1072-1079`

```java
private static JointReadOnly findFloatingBaseJoint(RigidBodyBasics treeRoot)
{
   for (JointReadOnly childJoint : treeRoot.getChildrenJoints())
   {
      if (childJoint.getDegreesOfFreedom() == 6)
         return childJoint;
   }
   return null;
}
```

**문제**: "트리 루트의 자식 중 6-DoF인 첫 번째 관절 = free-flyer(base)"라고 **가정**한다. Alex 모델에서
elevator → SixDoFJoint → pelvis 구조라면 맞지만, 로봇 모델에 6-DoF 관절이 여러 개 있거나(예: 향후
모델에 6-DoF 가상 관절이 추가되는 경우) 첫 번째로 찾은 게 실제 floating base가 아니면 **조용히 틀린
관절을 base로 잡고 이후 Schur complement 전체가 잘못 계산**된다. `null` 반환 시엔 근처 코드에서
fixed-base 폴백으로 빠지지만, "잘못된 6-DoF 관절을 찾았지만 null은 아닌" 케이스는 감지되지 않는다.

**어떻게 고쳐야 하나**:
- 단위 테스트에 **Alex 실제 로봇 모델**을 넣어 `findFloatingBaseJoint`가 실제로 pelvis SixDoFJoint를
  찾는지 검증하는 케이스 추가 (`JointLevelKFTestFixture` 등 기존 테스트가 합성 모델 위주인지 확인).
- 방어적으로, 6-DoF 자식이 **둘 이상**이면 지금처럼 첫 번째를 조용히 쓰지 말고 `LogTools.error`로
  모호성을 경고하거나 예외를 던지도록 수정.

---

### 3. Gap-joint marginalize vs lock 불일치

**위치**: `jointLevel/JointLevelKFPreFilter.java:685`, `1088-1101` (판단 지점), `390`, `706` (주석/로그)

```java
if (!jointToIndex.containsKey(spanningJoint)) // unfiltered => gap joint => marginalize it
```

**문제**: SPEC 문서(`jointKF_derivation.md` §3.2)는 필터 대상이 아닌 관절을 **잠금(lock)** 하라고
규정하는데, 실제 구현(`collectSpanningJoints`)은 이를 **한계화(marginalize, 자유롭게 취급)** 한다.
`CompositeRigidBodyMassMatrixCalculator`가 관절을 잠근 채로 그 하위 서브트리 관성을 유지할 수 없어서
생긴 구현 제약인데, 이는 **SPEC과 다른 물리적 가정을 쓰는 것**이다. Alex 실물엔 gap joint가 없어 지금은
두 방식이 수학적으로 동일하지만(390줄 주석), **그 사실 자체가 코드로 검증되지 않고 사람이 눈으로
확인해야 하는 상태**다.

**어떻게 고쳐야 하나**: 부팅 시(또는 `allocate()` 시점) `numberOfNuisanceDOF`가 정확히 6(베이스만)인지,
즉 gap joint 개수가 0인지 assert/warn하는 코드를 추가:

```java
int gapJoints = numberOfNuisanceDOF - 6;
if (gapJoints > 0)
   LogTools.warn("JointLevelKFPreFilter: " + gapJoints + " gap joint(s) detected — "
                + "SPEC assumes locking, implementation marginalizes. Confirm this is intended.");
```

706줄 근처에 이미 `gapJoints > 0` 분기로 `info` 레벨 로그가 있지만, gap joint가 실제로 없다는 걸
보장하는 로직은 아니다. Alex가 아닌 로봇(팔 마니퓰레이션 추가 등)에 재사용될 때 이 가정이 조용히 깨질
수 있으니, 최소 `warn` 이상으로 격상하거나 CI에 토폴로지 검증 테스트 추가를 권한다.

---

### 4. (해제) 자이로 측정잡음 0-플로어 — 이미 완화되어 있음

**위치**: `jointLevel/JointLevelKFPreFilter.java:160-161`, `1476-1503`

```java
private static final double SIGMA_GYRO_FLOOR = 1.0e-6;        // (0.001 rad/s)^2 per axis (safety net)
private static final double SIGMA_GYRO_FLOOR_TRACE = 3.0e-6;
```

과거 리뷰에서 "미해결"로 분류했던 항목이지만 재확인 결과 **이미 처리되어 있다**. `buildAndFloorSigma()`
(1476줄)가 매 IMU의 `getAngularVelocityNoiseCovariance()` trace가 플로어 밑이면 `LogTools.error`/`warn`을
찍고 `SIGMA_GYRO_FLOOR * I3`로 대체한다(1489-1499줄). **수정 불필요.** 다만 이 로그
(`"fix the sensor SensorNoiseParameters at the source"`, 1492줄)가 실제 하드웨어에서 찍히고 있는지는
운영 중 확인해볼 가치가 있다 (찍힌다면 `SensorNoiseParameters` 미설정이라는 근본 원인이 아직 안 고쳐진
것).

---

### 5. 발목-토크 사각지대 (mid-transfer dropout, 600ms)

**위치**: `invariantEstimator/CONTACT_DETECTION.md:153, 232` (§5.3)

**문제**: 실측 로그로 문서화되어 있음 — 실제로 체중이 실린 지지발인데도 발목 피치 토크 기반 힘
추정치가 CoP가 발목 아래를 지나는 순간 붕괴해서 $p_L=0$이 **600ms** 동안 지속된다. 이 구간 동안
`InvariantEKFStateEstimator`는 그 발을 90배 노이즈로 뮤트한 채 반대 발 하나로만 앵커링한다.
2026-07-17 수정(Schmitt trigger)은 **재접지 chattering만 줄였을 뿐**, 이 600ms 구간 자체는 그대로다.

**어떻게 고쳐야 하나**: 이번 PR 범위에서 코드 한 줄로 고칠 문제가 아니다(원인이 센서 레벨 — 발목
토크→힘 매핑의 구조적 사각지대). 문서에도 "future work, not the provider"라고 명시되어 있다. **PR
리뷰 관점 권고**: 이 알려진 한계를 완화할 모니터링(예: 한 발이 90×/9.5× 인플레이션 상태로 몇 ms 이상
지속되면 카운트하는 YoVariable)이 이번 PR에 포함돼 있는지 확인하고, 없다면 후속 이슈로 트래킹할 것.

---

## B. 설계상 권고 (버그는 아니지만 병합 전 검토 가치 있음)

### 6. 접촉 인플레이션 상수 `c=90`의 근거 미문서화

**위치**: `invariantEstimator/InvariantEKFStateEstimator.java:157-158`

```java
private double swingMeasurementInflation = 9.0e1;          // R_i  ×= inflation^(1−p)
private double swingSlipInflation = 9.0e1;                 // σ_{c,i}² ×= inflation^(1−p)
```

**문제**: $c^{1-p}$ 형태 자체(왜 지수형인지)와 밑수 90이 어디서 나왔는지(실험/그리드서치/직관적
선택인지) 코드·문서 어디에도 없다. `setSwingMeasurementInflation`/`setSwingSlipInflation`
(872-880줄)로 런타임에 바꿀 수 있게 되어 있는 걸 보면 "적당히 크게 잡아놓고 튜닝은 나중에"로 보인다.

**어떻게 고쳐야 하나**: 코드 수정보다 **문서화 요청**. 90이라는 값을 선택한 근거(민감도 분석 결과,
혹은 "그냥 충분히 크게"였다면 그 사실 자체)를 주석으로 남기도록 요청할 것. 특히 이 값이 NIS 기준으로
검증된 적이 있는지가 중요.

### 7. Conditioning gate가 자체 이력 없이 이진 스킵

**위치**: `invariantEstimator/InvariantUpdater.java:184-189`

```java
if (conditionProxy > COND_S_MAX)
{
   normalizedInnovationSquared = Double.NaN;
   gateSkipCount++;
   return; // ill-conditioned S: skip the whole update, state and P untouched
}
```

**문제**: `cond(S)`가 `COND_S_MAX=1e9` 경계 근처에서 tick마다 오르내리면, 이 게이트가 켜짐/꺼짐을
반복해 접촉 업데이트가 간헐적으로 스킵될 수 있다. 접촉확률 chattering과 같은 패턴인데, 이쪽엔 대응하는
debounce가 없다.

**어떻게 고쳐야 하나**: 당장 고칠 필요는 없음(`COND_S_MAX`가 매우 크게 잡혀 있어 실제로 자주
트리거되진 않을 것으로 보임). `gateSkipCount`가 연속으로 튀는지 로그/YoVariable로 관찰 가능한지
확인하고, 운영 중 chattering이 관측되면 `TouchdownReseedLatch`처럼 별도 dwell 추가를 검토하라고
코멘트에 남길 것.

### 8. `InvariantUpdater`가 가변 측정 크기에서 allocation-free 아님을 자체 명시

**위치**: `invariantEstimator/InvariantUpdater.java:29-31`

```java
* <p>The measurement dimension (H's row count) may vary between calls — e.g. when the number of
* active contacts changes — so the measurement-sized work matrices are reshaped per call; this class
* is therefore not strictly allocation-free under a changing measurement size.</p>
```

**문제**: 현재 Alex는 접촉점 N=2 고정이라 실질적 문제는 없지만, 이 클래스를 다른 로봇(다족, 손 접촉
추가)에 재사용하면 **실시간 루프에서 GC 스톨이 발생할 수 있다**는 걸 클래스 스스로 인정하고 있다.

**어떻게 고쳐야 하나**: 이번 PR에서 고칠 필요 없음(Alex 한정 요구사항). 재사용 시 함정이 될 수 있으니,
클래스 Javadoc에 "N이 런타임에 바뀌는 로봇에 재사용하려면 최대 N 기준으로 워밍업하는 로직을 추가해야
함"이라는 한 줄을 남겨두라고 권고.

### 9. 자이로 바이어스 클램프 카운터가 아무 데도 소비되지 않음

**위치**: `invariantEstimator/InvariantEKFStateEstimator.java:104, 240, 527`

```java
private static final double MAX_GYRO_BIAS = 0.02; // rad/s per axis
...
private final YoInteger yoGyroBiasClampCount = new YoInteger("invariantGyroBiasClampCount", registry);
...
yoGyroBiasClampCount.set(yoGyroBiasClampCount.getValue() + 1);   // 527줄
```

**문제**: 98-103줄 주석에 저자 스스로 "클램프는 이미 발동한 트립와이어이지 고정책이 아니다. 카운트가
늘어난다면 게이지(gauge)가 다시 망가진 것"이라고 써놨는데, 이 카운터를 **읽는 코드는 파일 안에
없다** — YoVariable로 로그에는 남지만, 값이 임계치를 넘었을 때 경고/모드 전환 등 상위 로직으로
이어지는 배선이 없다. 즉 오퍼레이터가 SCS2 GUI를 직접 열어서 이 변수를 봐야만 재발을 알아챌 수 있다.

**어떻게 고쳐야 하나**:

```java
private static final int GYRO_BIAS_CLAMP_ALARM_THRESHOLD = ...; // 예: 연속 N tick
```

같은 임계치를 두고, tick당 클램프 발생 시 별도 `YoBoolean invariantGyroBiasClampAlarm`을 세팅해 상위
상태 머신(혹은 최소한 로그 레벨을 `warn` 이상으로)에서 감지 가능하게 만들 것을 권한다. 최소 버전으로는
연속 N tick 이상 클램프가 걸리면 `LogTools.warn`을 한 번 찍는 정도로도 충분하다.

### 10. `JointLevelKFPreFilter`와 `InvariantEKF` 간 진단 상관관계 없음

**위치**: 아키텍처 전반 (`InvariantMainStateEstimator.java:516` 근처에서 두 필터의 레지스트리가
합쳐지긴 함)

**문제**: 두 필터는 "bias-corrected ω, a"라는 단방향 인터페이스로만 연결된다. 각 필터의
`gateSkipCount`류 진단이 각자 YoVariable로는 노출되지만, "관절 KF가 게이트를 스킵한 tick과 InEKF가
게이트를 스킵한 tick이 겹치는지"를 보려면 오퍼레이터가 두 로그를 수동으로 맞춰봐야 한다.

**어떻게 고쳐야 하나**: 즉시 고칠 버그는 아니고 관측성(observability) 개선 제안. `InvariantMainStateEstimator`가
이미 두 필터를 다 들고 있으니, "이번 tick에 관절 KF와 InEKF 둘 중 하나라도 게이트 스킵이 있었는지"를
나타내는 합성 YoBoolean 하나만 추가해도 디버깅이 훨씬 쉬워질 것으로 보인다.

---

## C. 스타일 (동작 무관)

### 11. FQN 인라인 호출

**위치**: `ihmc-avatar-interfaces/src/main/java/us/ihmc/avatar/AvatarEstimatorThreadFactory.java:490, 492, 500`

```java
if (preFilter instanceof us.ihmc.stateEstimation.jointLevel.JointLevelKFPreFilter)
{
   ProprioceptivePreFilter alpha = us.ihmc.stateEstimation.jointLevel.AlphaComplementaryPreFilter.createForKinematicsEstimator(...);
   preFilter = new us.ihmc.stateEstimation.jointLevel.SwitchableJointLevelSource(preFilter, alpha, preFilterRegistry);
}
```

**문제**: `us.ihmc.stateEstimation.jointLevel.JointLevelKFPreFilter` / `AlphaComplementaryPreFilter` /
`SwitchableJointLevelSource` 세 곳만 완전정규명(FQN)으로 박혀 있고, 같은 패키지의 다른 클래스
(`ProprioceptivePreFilter`, `ProprioceptivePreFilterFactory`, 69-70줄)는 정상 import 되어 있다.
이름 충돌 없음을 확인했으므로 순수한 스타일 불일치.

**어떻게 고쳐야 하나**:

```java
import us.ihmc.stateEstimation.jointLevel.AlphaComplementaryPreFilter;
import us.ihmc.stateEstimation.jointLevel.JointLevelKFPreFilter;
import us.ihmc.stateEstimation.jointLevel.SwitchableJointLevelSource;
```

를 추가하고 490/492/500줄의 FQN을 단순 클래스명으로 교체 (동작 변화 없음).

---

## 우선순위 요약

| 우선순위 | 항목 | 조치 |
|---|---|---|
| **높음 (승인 전 확인)** | 1. `SIGMA_TAU` 재조정 여부 | NIS 기반 재교정 완료 여부 확인, 미완료면 PR에 명시 |
| **높음** | 2. 베이스 관절 식별 가정 | Alex 실물 모델 테스트 케이스 존재 여부 확인 |
| **높음** | 3. Gap-joint 판단 | 토폴로지 검증(assert/warn) 존재 여부 확인 |
| 해제 | 4. 자이로 잡음 플로어 | 이미 완화됨, 조치 불필요 |
| 중간 | 5. 발목-토크 사각지대 | 후속 이슈 트래킹 권장 (이번 PR 범위 아님) |
| 낮음 | 6~10 | 권고/관측성 개선, 후속 PR 가능 |
| 낮음 (nit) | 11. FQN import 정리 | 원하면 바로 적용 가능 |
