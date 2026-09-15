# 신규 State Estimation 스택 분석 보고서 (`new-estimator/*` 브랜치)

작성일: 2026-08-10 · 대상 브랜치: `new-estimator/joint-level-fixes` (base: `develop`)
대상 모듈: `ihmc-state-estimation/.../stateEstimation/{jointLevel, invariant_estimator}`

이 문서는 브랜치에 새로 추가된 두 계층 — **① 관절 레벨 전처리 필터(`JointLevelKFPreFilter`)** 와
**② 몸체(base) 레벨 불변 확장 칼만 필터(`InvariantEKF`/SE_k(3) InEKF)** — 를 기존 DRC 추정기
(`PelvisLinearStateUpdater` + 단순 `JointStateUpdater`)와 비교 분석하고, 핵심 수식을 코드 위치와
함께 정리하며, 코드에 이미 기록된 이슈들과 자체 검토로 발견한 개선점을 정리한다.

---

## 0. 아키텍처 한눈에 보기

```
                    ┌───────────────────────────────────────────────────┐
 raw encoder/IMU →  │ JointLevelKFPreFilter  (관절 q, qd + IMU bias 추정) │ → 필터링된 q, qd, IMU bias
                    │  us.ihmc.stateEstimation.jointLevel                │
                    └───────────────────────────────────────────────────┘
                                          │ (bias-corrected ω, a)
                                          ▼
                    ┌───────────────────────────────────────────────────┐
                    │ InvariantEKF (SE_k(3) 우불변 EKF, base pose/vel     │
                    │  + N개 접촉점)                                      │
                    │  us.ihmc.stateEstimation.invariant_estimator       │
                    └───────────────────────────────────────────────────┘
                                          │
                    ┌─────────────────────┴─────────────────────┐
                    ▼                                             ▼
      InvariantEKFStateEstimator                      InvariantMainStateEstimator
      (보조/평가용, controller 미구동)                  (메인 추정기로 승격, rootJoint 구동)
```

기존 DRC 스택은 `JointStateUpdater`(단순 통과) + `PelvisLinearStateUpdater`(선형 KF, IMU-키네마틱스
상보 필터형)로 구성되어 있었다. 새 스택은 이를 **두 개의 독립된 EKF로 분리**했다:
관절 공간(P-A, "pre-filter/anchor") 필터와 몸체 공간의 SE_k(3) 불변 필터. 두 필터는 "관절 EKF의
바이어스 보정 IMU 출력"이라는 단일 인터페이스로만 결합되어 있어, 서로 독립적으로 튜닝/대체 가능하다는
것이 설계 의도다.

---

## 1. 기존(DRC) 추정기 대비 핵심 차이

| 항목 | 기존 (`PelvisLinearStateUpdater` + `JointStateUpdater`) | 신규 (`JointLevelKFPreFilter` + `InvariantEKF`) |
|---|---|---|
| 관절 상태 | 센서값을 그대로 통과(alpha filter 정도) | **관절별 KF**: q, qd, (pair별) IMU bias를 상태로 추정 |
| 몸체 표현 | 쿼터니언 자세 + world 속도, 좌표별 개별 필터 | **SE_k(3) 리 군(Lie group)** 원소 하나로 (R, v, p, 접촉점들) 통합 표현 |
| 필터 오차 정의 | 가법적(additive) 오차 | **우불변(right-invariant)** 오차 X̂ = exp(ξ)·X — 선형화 오차가 궤적에 무관 |
| 접촉 처리 | Foot switch → 이진 trust → anchor 교체 | **연속 확률 p∈[0,1]** → 측정잡음/프로세스잡음의 연속적 스케일링 (soft mute) |
| 헤딩(yaw) | 암묵적으로 절대 신뢰 | 접촉 FK 업데이트가 회전 자코비안 0 → yaw 관측 불가를 **설계로 인정**하고 별도 `FootReferencedYawCorrector`/중력 레벨링으로 보완 |
| 프로세스 잡음 | 고정 파라미터 | **질량행렬 기반**(Schur complement) 구성 형상 의존 Q, 접촉별 슬립 잡음 가변 |
| 자이로 사용 | 개별 필터에 흡수 | 관절 EKF에서 **pair 자이로 diff + stance anchor를 하나의 stacked Joseph update**로 결합 |
| 진단/일관성 | 제한적 | NIS(정규화 혁신 제곱) 상시 계산, conditioning gate, YoVariable 다수 노출 |

---

## 2. 핵심 알고리즘 1 — `JointLevelKFPreFilter` (관절 레벨 KF)

파일: `jointLevel/JointLevelKFPreFilter.java` (2584 lines), 설계 배경: `jointLevel/CHANGES.md`

### 2.1 상태 및 측정 개념

IMU-pair(부모/자식 IMU가 관절 체인으로 연결된 쌍)마다 자이로 차분을 이용한 관측 모델:

$$
z_{\omega,ab} = J_{ab}(\hat q)\,\dot q_{S_{ab}} + b_{\omega,a} - b_{\omega,b} + v
$$

여기서 $J_{ab}$는 IMU a→b 경로의 관절 자코비안, $S_{ab}$는 경로 관절 부분집합, $b_{\omega,\cdot}$는 IMU별
자이로 바이어스(state), $v$는 측정 잡음. 코드: `JointLevelKFPreFilter.buildStackedMeasurement()`.

### 2.2 Rev.2 변경 ① — 프로세스 잡음: 부동 베이스 Schur complement

기존(Rev.1, locked-base 가정):

$$
Q_a = \sigma_\tau^2\, M_{jj}(q)^{-2}
$$

신규(Rev.2, `CHANGES.md` §Change 1, `updateProcessNoiseFromMassMatrix`):

$$
Q_a = \sigma_\tau^2\, \Lambda(q)^{-2}, \qquad
\Lambda = M_{jj} - M_{jb}\,M_{bb}^{-1}\,M_{bj}
$$

$\Lambda$는 6-DoF 부동 베이스 블록에 대한 Schur complement — 즉 "베이스가 자유롭게 반동할 때" 관절이
단위 토크당 얼마나 더 가속되는지를 반영한다. $\Lambda \preceq M_{jj}$ (PSD 순서)이므로
$\Lambda^{-2} \succeq M_{jj}^{-2}$: **동일 $\sigma_\tau$에서 유효 프로세스 잡음이 증가**하며, 근위(hip 등)
관절일수록 영향이 크다.

베이스와 필터링 대상 관절 사이에 "필터되지 않은 관절(gap joint)"이 끼어있는 경우 이를 잠그지(lock) 않고
**한계화(marginalize)**하여 $\Lambda = M_{ff} - M_{fN} M_{NN}^{-1} M_{Nf}$ ($N$ = 베이스+gap joints)로
일반화했다. Alex 실물 토폴로지에는 gap joint가 없어 위 두 식이 동일해진다 (코드/문서에 명시된 사람 확인
필요 항목, §5.1 참조).

- 코드: `findFloatingBaseJoint`, `collectSpanningJoints`, `updateProcessNoiseFromMassMatrix`
- 회전자 관성 보정: $\Lambda_{\text{eff}} = \Lambda + \mathrm{diag}(n_i^2 J_{\mathrm{rotor},i})$ (`JointLevelKFPreFilter.java:131-146`)
- 검증: `JointLevelKFMassMatrixNoiseTest` (독립 참조 구현과 1e-9 수준 일치)

### 2.3 Rev.2 변경 ② — 자이로 측정: per-pair 순차 업데이트 → 1회 stacked update

기존(Rev.1): pair마다 개별 Joseph update + 별도 phase-2 stance anchor 업데이트.

신규(Rev.2, `CHANGES.md` §Change 2/3): $E$개 pair + $K$개(0~2) 활성 stance anchor를 하나로 쌓아
한 번에 업데이트:

$$
z_g \in \mathbb{R}^{3(E+K)}, \qquad
H_g = \big[\, 0 \mid J_{\text{stack}}(\hat q) \mid L(\hat q) \,\big], \qquad
R_g = L\,\Sigma\,L^\top + \mathrm{blkdiag}(0_{\text{pairs}}, \Sigma_\epsilon)
$$

$L$은 자이로 바이어스 열에 대응하는 **회전 edge-incidence 연산자**이며, 동시에 바이어스 프로세스가
아니라 **자이로 백색잡음** $\Sigma$를 innovation 공간으로 옮기는 연산자이기도 하다. 따라서 pair별
block-diagonal $R$을 쓰면 공유 IMU 샘플(예: 별 모양 pelvis IMU 토폴로지, 양발 지지 시 공유 베이스
샘플)의 교차공분산을 이중 계산하는 오류가 생긴다 — $R_g = L\Sigma L^\top$가 이를 정확히 반영한다.

- 코드: `buildStackedMeasurement()`, `computeJointState()` (단일 `josephUpdate(H_g, z_g, R_g, ...)`)
- 검증: `JointLevelKFStackedOracleTest` — 독립 참조 KF(베이스 각속도를 nuisance로 marginalize)와 20회
  랜덤 tick에서 1e-6 수준 일치.

### 2.4 조인트 KF에 남아있는 미해결(open) 항목 (원저자 표기, `CHANGES.md` 하단)

- **`SIGMA_TAU` 재조정 미실시**: Schur complement 전환으로 $Q_a$가 커졌음에도 $\sigma_\tau=5\,\mathrm{N\cdot m}$
  (구 Rev.1 대비 인터림 값)는 재교정되지 않음. Quiet-standing/walking NIS로 재조정 필요.
- **베이스-렌치 확장 보류**: quiet-standing NIS가 여전히 낮으면 다음 항은 $\sigma_\tau$ 재조정이 아니라
  $-\Lambda^{-1} M_{jb} M_{bb}^{-1}$를 통한 베이스 렌치 불확실성 항.
- **자이로 잡음 플로어 이슈** (`JointLevelKFPreFilter.java:147-` 부근): 하드웨어에서
  `getAngularVelocityNoiseCovariance()`가 `SensorNoiseParameters`가 설정되지 않으면 0을 반환 → 순수
  바이어스 행의 innovation covariance 하한이 사라져 $\lambda_{\min}(S)$가 붕괴(로그에서 2.155e-12
  관측)하고 Joseph $KRK^\top$ 항이 발산. **잡음 플로어를 코드에서 강제**하도록 처리한 것으로 보이나,
  센서 파라미터가 실제 설정되지 않는 근본 원인은 별도 확인 필요.

---

## 3. 핵심 알고리즘 2 — `InvariantEKF` (SE_k(3) 우불변 EKF)

파일: `InvariantState.java`, `InvariantPropagator.java`, `InvariantUpdater.java`, `SEK3_Utils.java`,
`ContactUpdater.java`, `GravityLevelingUpdater.java`, `InvariantEKF.java`

### 3.1 상태 표현

```
        [ R  v  p  p_c0 ... p_c(N-1) ]
    X = [ 0  1                       ]  ∈ SE_k(3),   k = 2 + N
        [ 0     1                    ]
        [ .          .               ]
        [ 0             1            ]
```

접촉점 개수 $N$개를 포함한 하나의 $(3+k)\times(3+k)$ 행렬로 회전, 속도, 위치, 접촉위치를 **동일한
회전 $R$을 공유**하는 열들로 표현한다 (코드: `InvariantState.java`). 공분산은 접선공간
$[\delta\varphi;\delta v;\delta p;\delta p_{c_0};\dots] \in \mathbb{R}^{9+3N}$ 순서.
**바이어스는 상태에서 제외**되어 있으며(상위 관절 EKF 책임으로 명시적으로 분리, `InvariantState.java:14`),
이는 IMU bias 추정 책임을 앞 단(§2)에 완전히 위임한다는 설계 결정이다.

### 3.2 전파(예측) — 정확한 이산시간 적분

$\varphi = \omega\,\Delta t$일 때 (`InvariantPropagator.predictMean`):

$$
R^+ = R\cdot\exp(\varphi), \qquad
v^+ = v + g\,\Delta t + R\,\Gamma_1(\varphi)\,a\,\Delta t, \qquad
p^+ = p + v\,\Delta t + \tfrac12 g\,\Delta t^2 + R\,\Gamma_2(\varphi)\,a\,\Delta t^2
$$

$\Gamma_0=\exp(\varphi)$(=SO(3) exp), $\Gamma_1$=왼쪽 자코비안, $\Gamma_2$는 2차 적분 계수 —
Euler 적분이 아닌 **각속도 구간 내 정확한(closed-form) 적분**을 사용해 고속 회전 시 정확도를 확보한다.
접촉점은 정지(static) 모델: $d_i^+=d_i$.

공분산은 $P^+ = \Phi P \Phi^\top + Q_d$, $\Phi=\exp(A_c\Delta t)$ (중력만 의존, 상태 독립적)이고

$$
Q_d = \Phi\,(\mathrm{Ad}_X\,Q_c\,\mathrm{Ad}_X^\top)\,\Phi^\top\,\Delta t
$$

로, 프로세스 잡음을 **그룹 adjoint로 world frame으로 옮긴 후** 이산화한다 — 이는 우불변 필터의 정석적
구성으로, "world-centric" 정의를 코드 주석에 명시(`InvariantPropagator.java:14-20`).
접촉 슬립 잡음 $\sigma_{c,i}^2$는 접촉확률 $p_i$에 따라 매 tick 재설정된다(§4).

### 3.3 업데이트 — Joseph form + 조건수 게이트 + NIS

일반화된 우불변 보정(`InvariantUpdater.update`):

$$
S = HPH^\top + R,\quad K=PH^\top S^{-1},\quad c=K\cdot\text{residual}
$$
$$
\hat X^+ = \exp_G(-c)\,\hat X, \qquad P^+ = (I-KH)P(I-KH)^\top + KRK^\top
$$

Joseph form을 사용해 수치적으로 $P$의 PSD성을 보존한다. 추가로 신규 도입된 부분:

- **조건수 게이트**: Cholesky 분해로 $S$의 PD성과 $\mathrm{cond}(S)\approx(\max L_{ii}/\min L_{ii})^2$를
  고유분해 없이 근사, $10^9$ 초과 시 **업데이트 전체를 스킵**(state/P 불변). 코드:
  `InvariantUpdater.java:156-189`.
- **NIS 상시 계산**: $\mathrm{NIS}=r^\top S^{-1} r$, $\chi^2_z$ 분포를 따라야 하므로 필터 일관성
  모니터링에 사용 (`InvariantEKFStateEstimator`의 `CONSISTENCY_CONFIDENCE=0.95` 밴드).

### 3.4 측정 모델 A — 접촉 forward-kinematics (`ContactUpdater`)

$$
h_{C_i}(q) = R^\top(p_{C_i}-p_B) + J_{C_i}(q)\,w_q
$$

우불변 형태로 재작성하면 world frame residual이 **상태에 무관한 상수 자코비안**을 갖는다:

$$
\text{residual} = \hat R\,y - (\hat d_i - \hat p) \;\approx\; \delta p - \delta p_{c_i} = H\xi,
\qquad H = [\,0 \mid 0 \mid {+}I \mid \cdots {-}I(\text{contact }i) \cdots\,]
$$

$R_{\text{world}} = \hat R\, N_i\, \hat R^\top$, $N_i = J_{C_i}\Sigma_q J_{C_i}^\top$ (관절 인코더 잡음의
전파). **회전 블록이 항상 0** — 즉 접촉 측정만으로는 자세를 전혀 관측하지 못함이 설계상 명시적으로
문서화되어 있다(`ContactUpdater.java` Javadoc). 이 구조적 한계가 §3.5의 중력 레벨링을 필요로 한다.

### 3.5 측정 모델 B — 중력 레벨링(신규, `GravityLevelingUpdater`)

$\hat g_{\text{body}} = \hat R^\top e_z$ 예측에 대해, 정지(quasi-static) 가정 하 가속도계 방향
$\hat f = a/\|a\|$를 측정으로 사용:

$$
\text{residual} \approx -\hat R^\top[e_z]_\times\,\delta\varphi = H_\varphi\,\delta\varphi,\qquad
H_\varphi = [-\hat R^\top e_y \mid {+}\hat R^\top e_x \mid 0]
$$

$H_\varphi$의 계수는 2 (null space가 중력 방향, 즉 yaw) → **yaw-safe by construction**. 비등방
측정잡음(anisotropic)으로 roll은 신뢰하고 pitch는 덜 신뢰:

$$
R = \sigma_{\text{roll}}^2 I_3 + (\sigma_{\text{pitch}}^2-\sigma_{\text{roll}}^2)\,\hat u_p\hat u_p^\top,
\qquad \hat u_p = \mathrm{normalize}(e_y\times \hat g_{\text{body}})
$$

전후방 보행 가속이 pitch 축 specific force로 오인되는 것을 방지하기 위함. **실제 측정값은 순간
가속도계 방향이 아니라 상보필터로 만든 기준 $\hat g_{\text{ref}}$**:

$$
\hat g_{\text{ref}}^- = \hat g_{\text{ref}} - \Delta t\,(\omega_{\text{raw}}\times \hat g_{\text{ref}}), \qquad
\hat g_{\text{ref}} = \alpha\,\hat g_{\text{ref}}^- + (1-\alpha)\,\hat f,\quad \alpha=e^{-\Delta t/\tau},\ \tau=5\,\mathrm{s}
$$

$\tau=5\,\mathrm{s}$는 로봇 좌우 균형 모드 주파수 $\omega_b\approx3.1\,\mathrm{rad/s}$에서 15배
저감($1/\sqrt{1+(\omega_b\tau)^2}\approx0.065$)을 달성하도록 실측 로그 기반으로 선택됨.
(§5.2에서 이 부분이 수정된 **자기 잠금(self-latching) 버그**의 재발 방지책임을 설명.)

---

## 4. 접촉 확률(soft contact) 처리

파일: `ContactProbabilityProvider`, `FootSwitchContactProbabilityProvider`, `KinematicContactDetector`,
`TouchdownReseedLatch`, 설계 문서: `invariant_estimator/CONTACT_DETECTION.md`

기존 DRC 추정기는 이진 trust(로드 비율 Schmitt trigger + dwell + 활공(slip) 감지)만 있었다. 신규
스택은 **연속 확률 $p_i\in[0,1]$** 을 인터페이스로 채택하고, 두 개의 지수형 스케일 knob으로 필터에
주입한다 ($c=90$):

$$
R_i(p) = c^{1-p}\,R_i^{\text{FK}} \in [1,90]\cdot R_i^{\text{FK}}, \qquad
\sigma_{c,i}^2(p) = c^{1-p}\,\sigma_c^2\big|_{\text{contact}}
$$

즉 접촉 업데이트는 **항상 적용**되지만 신뢰도에 따라 잡음이 연속적으로 부풀려지는 "soft mute" 방식.
$p<0.5$/$p\ge0.5$ 하드 게이트(양발 미접촉 시 base 속도 0 고정, 재접지(reseed) 트리거 등)도 병존한다.

**2026-07-17 수정 이력이 특히 중요**: 관절 토크 기반 foot switch가 발목 피치 토크의 CoP 통과 지점에서
추정력이 붕괴하는 구조적 사각지대가 있어, 한 번의 물리적 착지에서 $p$가 45회 중 다수 0.5를 오르내리며
재접지(re-seed)가 스텝당 최대 2회 발생 → 베이스 속도에 ~0.1 m/s 임펄스가 유입되는 문제가 있었다. 해법은
구 DRC 추정기의 로드비율 히스테리시스(0.35 진입/0.25 유지) + 40ms dwell을 **Schmitt trigger**로 이식하고,
`TouchdownReseedLatch`에 100ms 재무장(re-arm) dwell을 추가하여 물리적 착지당 최대 1회 재접지로 제한한
것 (`CONTACT_DETECTION.md` §5).

---

## 5. 발견된 문제점 / 개선이 필요한 부분

원저자가 코드/문서에 이미 명시한 미해결 항목(①~⑥)과, 본 검토에서 추가로 식별한 항목(⑦~⑪)을 함께 정리한다.

### 5.1 원저자 표기 미해결(open) 항목 — 우선순위 상

1. **`SIGMA_TAU` 미재조정** (`JointLevelKFPreFilter.java:78-83`, `CHANGES.md`): Schur complement 전환으로
   프로세스 잡음이 구조적으로 증가($\Lambda^{-2}\succeq M_{jj}^{-2}$)했음에도 값 자체는 그대로 이월됨.
   Quiet-standing/걷기 NIS 재교정 전에는 필터가 실제보다 과신(over-confident)하거나 과소신(under-confident)
   상태일 수 있다. **권고**: 병합 전 하드웨어 NIS 로그 기반 재교정을 게이팅 조건으로 둘 것.
2. **베이스 관절 식별 가정** (`findFloatingBaseJoint`): "트리 루트의 6-DoF 자식 = free-flyer"라는 가정이
   Alex 모델에 대해 확인되지 않았다고 코드에 명시(`CHANGES.md` "review by hand"). **권고**: 단위 테스트에
   Alex 실제 모델 토폴로지 검증 케이스 추가.
3. **Gap-joint marginalize vs lock 판단 필요**: SPEC은 잠금(lock)을 가정하나 구현은 한계화(marginalize).
   Alex 실물에 gap joint가 없어 현재는 등가이지만, 향후 IMU/필터 대상 조인트 구성이 바뀌면 차이가 발생.
   **권고**: 토폴로지 검증 assertion을 부팅 시 자동 체크로 승격.
4. **자이로 측정잡음 0 플로어**: `SensorNoiseParameters` 미설정 시 $\Sigma=0$이 되어 $\lambda_{\min}(S)$
   붕괴 위험(§2.4). 현재 코드에서 완화 처리를 했더라도, **근본 원인(파라미터 미설정)** 자체가 운영 중
   재발하지 않도록 부팅 시 경고/차단이 필요.
5. **가속도계 발목-토크 사각지대(mid-transfer dropout, 600ms)는 여전히 미해결** (`CONTACT_DETECTION.md`
   §5.3): 제공자(provider) 레벨 debounce로는 구조적으로 해결 불가. 근본 수정은 총 수직력 정규화 또는
   CoP 융합 등 **센서 레벨** 변경이 필요 — 아직 착수되지 않음.
6. **중력 레벨링의 quasi-static 게이트 자기잠금 버그는 수정되었으나 유사 패턴 재발 위험**: 게이트가
   필터 자신의 추정 상태($\hat R^\top e_z$)를 참조하면 오차가 커질수록 게이트가 닫히는 불안정 궤환이
   발생한다는 것이 실증됨(`GravityLevelingUpdater.java:286-300`). 수정 후에도 **다른 게이트/휴리스틱을
   추가할 때 이 패턴(추정치를 게이트 조건에 사용)을 반드시 재검토**해야 한다.

### 5.2 본 검토에서 추가 식별한 항목

7. **매직 넘버 $c=90$의 근거 불명확** (`CONTACT_DETECTION.md` §1.1): 접촉 신뢰도 knob의 밑수 90이
   어떤 최적화/실험으로 도출되었는지 문서에 없다. $c^{1-p}$ 함수 형태 자체(지수형)도 대안(예: 선형,
   로지스틱) 대비 정당성이 제시되지 않았다. **권고**: 최소한 $c$에 대한 민감도 분석(선정 배경) 기록.
8. **조건수 게이트($\mathrm{cond}(S)>10^9$)가 "업데이트 전체 스킵"이라는 이진 동작**: 임계값 부근에서
   업데이트가 갑자기 켜짐/꺼짐을 반복하면 §4의 접촉 신뢰도 chattering과 유사한 불연속 거동을 유발할 수
   있다. NIS/조건수 게이트에도 §4의 Schmitt-trigger식 히스테리시스를 적용하는 것을 검토할 가치가 있다.
9. **`InvariantUpdater`가 "측정 크기가 바뀌면 allocation-free 하지 않다"고 스스로 명시**
   (`InvariantUpdater.java:29-31`): 접촉 개수가 런타임에 바뀌는 시나리오(예: 향후 다족 로봇, 손 접촉
   추가)에서는 실시간성 보장이 깨진다. 현재 Alex(2족, N=2 고정)에서는 문제가 없으나, 이 가정이 향후
   재사용 시 함정이 될 수 있음을 문서화해 둘 필요가 있다.
10. **`InvariantEKFStateEstimator`의 자이로 바이어스 클램프(`MAX_GYRO_BIAS=0.02 rad/s`)는 "이미 발동한
    트립와이어"로 스스로 규정**(`InvariantEKFStateEstimator.java:93-103`)하면서도 상시 활성 로직으로
    남아있음. 클램프가 계속 발동 중이라면(카운터 상승) 근본 원인(§2.4의 바이어스 관측성/게이지 복원)이
    재발했다는 뜻인데, 이를 자동으로 알람화(예: 임계 초과 시 안전 정지/모드 전환)하는 로직은 보이지 않음.
    **권고**: `invariantGyroBiasClampCount` 상승 시 상위 상태 머신에 통보하는 경로 추가.
11. **두 필터(JointLevelKF, InvariantEKF) 간 결합은 오직 "bias-corrected ω, a" 단방향 인터페이스** —
    InvariantEKF의 접촉/중력 업데이트 결과(예: 특정 접촉의 NIS 급증, conditioning 게이트 스킵)가 관절
    필터 쪽으로 피드백되지 않는다. 두 필터가 독립적으로 발산할 경우 서로의 이상을 감지할 방법이 제한적.
    **권고**: 최소한 InvariantEKF의 게이트-스킵/NIS-이상 신호를 관절 필터 진단과 나란히 로깅해 상관관계
    분석이 가능하도록 할 것 (현재도 YoVariable로 개별 노출은 되어 있으나, 상호 참조용 결합 지표는 없음).

### 5.3 테스트 커버리지 관점

`TEST_SUITE_MAP.md`, 다수의 `*Test.java` (Schur 대수, stacked oracle, reseed latch, contact
provider chattering 등)가 상당히 촘촘하게 구성되어 있고, 각 회귀에 "독립적으로 재구현한 참조값과
비교"하는 오라클 테스트 패턴을 일관되게 적용한 점은 품질 면에서 긍정적이다. 다만:

- 하드웨어 로그 기반 A/B 재현(`AlexEstimatorLogReplay`)은 스크립트/문서로만 존재하고 CI에는 포함되지
  않는 것으로 보임 — 회귀 시 수동 재실행 필요.
- `SIGMA_TAU`, $c=90$, `COND_S_MAX=1e9` 등 다수의 튜닝 상수가 **단위 테스트로 보호되지 않는 매직
  넘버**로 남아있다. 값 자체의 회귀(누군가 실수로 변경)를 잡는 "상수 고정 테스트"는 없다.

---

## 6. 결론 및 권고 요약

새 스택은 (a) 질량행렬 기반 형상 의존 프로세스 잡음, (b) 정확한 SE_k(3) 우불변 오차모델, (c) 연속 접촉
신뢰도, (d) 오라클 기반의 탄탄한 회귀 테스트라는 점에서 기존 DRC 추정기 대비 이론적으로 더 원칙적인
설계다. 반면:

1. **병합 전 필수**: `SIGMA_TAU` 재조정(§5.1-1), Alex 실물 토폴로지 가정 검증(§5.1-2/3), 자이로 잡음
   플로어 근본 원인 확인(§5.1-4).
2. **알려졌지만 미해결**: 발목 토크 사각지대(§5.1-5)는 센서 레벨 개선 전까지 잔존 리스크로 남는다 —
   운영 시 모니터링 지표(anchor-only 지속시간 등)를 대시보드화할 것을 권고.
3. **구조적 개선 후보**: 게이트/클램프류 로직에 히스테리시스 일관 적용(§5.2-8), 클램프 발동을 상위
   안전 로직에 연결(§5.2-10), 두 필터 간 진단 상관 로깅(§5.2-11).

전반적으로 이 브랜치는 "문제를 만들어낸" 변경이라기보다, 원저자들이 하드웨어 로그로 실증한 결함들을
반복적으로 고쳐온 이력(커밋 로그의 `gravity/pitch possible fix`, `clamp biases`, `raw gravity
fallback` 등)이 잘 드러난다. 다만 다수의 `TODO(retune)`/"review by hand" 항목이 **미결 상태로 병합
대상 브랜치에 남아있다는 점**이 가장 큰 리스크이며, 이 문서의 §5.1 항목들을 병합 체크리스트로 삼을
것을 권고한다.
