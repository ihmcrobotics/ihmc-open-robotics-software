# Estimator test-suite map — `jointLevel` + `invariantEstimator`

Complete map of the JUnit 5 test suite for the two estimator packages in `ihmc-state-estimation`,
written to support a 1:1 port of the suite to Python (so a Python re-implementation of the
estimator can be validated against the same properties, scenarios, and tolerances).

- Source root: `ihmc-state-estimation/src/test/java/us/ihmc/stateEstimation/`
- **176 test methods across 28 test classes** (92 in `jointLevel`, 84 in `invariantEstimator`),
  plus 4 test-support classes with no tests (`JointLevelKFTestFixture`, `JointLevelKFTestSupport`,
  `SettableTestIMU`, `SettableTestSensorMap`).
- Run the Java suite (JDK 17, from the workspace root):
  ```
  ./gradlew -p ihmc-open-robotics-software :ihmc-state-estimation:ihmc-state-estimation-test:test \
    --tests "us.ihmc.stateEstimation.jointLevel.*" \
    --tests "us.ihmc.stateEstimation.invariantEstimator.*" --rerun-tasks
  ```
- Mapped as of 2026-07-21 on branch `new-estimator/joint-debug`.

## Suite at a glance

### `us.ihmc.stateEstimation.jointLevel` (JointLevelKFPreFilter — joint-space KF over the IMU tree)

| Class | Tests | Focus | Python portability |
|---|---|---|---|
| `JointLevelKFStateTest` | 10 | State layout `[q; q̇; b_ω]`, encoder-seeded init, prior covariance blocks | Full |
| `JointLevelKFPredictTest` | 7 | Time update `x⁻=Fx`, `P⁻=FPFᵀ+Q`, bias-block growth, determinism | Full |
| `JointLevelKFUpdateTest` | 6 | Joseph-form update vs explicit-inverse reference KF, PSD, shrink | Full |
| `JointLevelKFMeasurementTest` | 11 | Encoder Jacobian `[I|0]`, pair-gyro `z=ω_c−Rω_p`, stacked H bias columns = L, R from gyro noise | Full (needs FK) |
| `JointLevelKFTransitionNoiseTest` | 6 | `F=I+A·dt`, CWNA/Van-Loan Q (scalar path), encoder R | Full |
| `JointLevelKFMassMatrixNoiseTest` | 10 | Schur-complement process noise `Qa=Λ_eff⁻¹Σ_τΛ_eff⁻ᵀ`, Van-Loan blocks, config dependence | Full (needs mass matrix) |
| `JointLevelKFFilterTest` | 6 | End-to-end predict→update loop, determinism, encoder tracking, 20k-tick bias convergence | Full |
| `JointLevelKFTrajectoryTest` | 8 | Consistent-sinusoid tracking, NaN hardening, singular-innovation skip, covariance coupling | Full |
| `JointLevelKFStandingStabilityTest` | 5 | Alex002 velocity-covariance blow-up diagnosis; one-tick injection bound (regression gate) | Full |
| `JointLevelKFBiasObservabilityTest` | 4 | Common-mode bias gauge nullspace; stance anchor fixes gauge; unfiltered-ankle anchor | Full (needs FK) |
| `JointLevelKFEncoderNISConsistencyTest` | 4 | Per-joint encoder R wiring + χ²₁ NIS consistency (4000 trials) | Full |
| `JointLevelKFDirectVelocityMeasurementTest` | 5 | Direct q̇ channel wiring, adaptive lag inflation of R, χ²₁ NIS | Full |
| `JointLevelKFStackedReferenceTest` | 2 | **Decisive reference**: stacked update == nuisance-ω_base-marginalized raw-gyro reference KF | Full (needs FK/Jacobians) |
| `JointLevelKFRotorAndGramTest` | 3 | Rotor-inertia Weyl floor, Gram-form Qa vs dense reference, rotor table lookup | Full |
| `JointLevelKFSingularInnovationDiagnosticTest` | 2 | Near-singular S diagnostic names the physical measurement | Adapt (message text) |
| `JointLevelKFPreFilterAllocationTest` | 3 | Hot-path allocation-free guard (JVM) + finiteness smoke | Only `testHotPathStaysFinite` |

### `us.ihmc.stateEstimation.invariantEstimator` (right-invariant EKF on SE_k(3))

| Class | Tests | Focus | Python portability |
|---|---|---|---|
| `SEK3UtilsTest` | 6 | SE_k(3) exp/log round-trip (k=1..3), SE(3) agreement at k=1, adjoint identities | Full |
| `InvariantStateTest` | 7 | Group element (5+N)×(5+N), tangent layout, round-trips, index bounds | Full |
| `InvariantPropagatorTest` | 8 | Closed-form free fall / gravity comp / rotation composition; exact log-linear error propagation | Full |
| `InvariantUpdaterTest` | 6 | Contact update residual reduction, NIS quadratic form, χ²(3) mean (4000 samples) | Full |
| `InvariantEKFTest` | 7 | Wiring/delegation bit-for-bit to propagator/updater, validation, e2e error reduction | Full |
| `InvariantEKFReseedTest` | 3 | Touchdown re-seed: PSD preservation, `P_dd=P_pp+RNRᵀ`, zero-release | Full |
| `ContactUpdaterTest` | 9 | FK measurement `y=Rᵀ(d−p)`, constant H `[0|+I|−I]`, R rotation, encoder-noise mapping | Full |
| `GravityLevelingUpdaterTest` | 14 | Roll/pitch leveling: anisotropic R, quasi-static gates, complementary gravity reference (τ=5 s) | Full |
| `FootSwitchContactProbabilityProviderTest` | 10 | Schmitt-trigger contact trust (0.25/0.35 band, 40 ms dwell), EMA probability, trust modes | Full |
| `TouchdownReseedLatchTest` | 7 | Fire-once-per-swing latch state machine (0.5/0.1 band, 100-tick dwell) | Full |
| `InvariantMainStateEstimatorTest` | 5 | Full-pipeline integration on synthetic humanoid (static, rotation, hanging, free fall, poisoned IMU) | Adapt (needs robot model) |
| `InvariantEstimatorAllocationTest` | 2 | Hot-path allocation-free guard (JVM) | Not portable |

---

## jointLevel test infrastructure & unit tests

### JointLevelKFTestFixture.java
- **Purpose**: Package-private shared harness for the `JointLevelKFPreFilter` unit tests, ported from a JAX reference (`invariant-estimation/jointKF`). Builds a synthetic IMU-pair setup: a random floating revolute-joint chain (Mecano `RandomFloatingRevoluteJointChain`), hand-built `IMUSensorReadOnly`s (`TestIMU`) with controllable gyro readings + PD covariances, and a mutable per-joint encoder sensor map (`TestSensorMap`). Also provides the matrix assertion helpers (allclose / symmetric / PSD via eigenvalues) and deterministic matrix generators (`spd`, `identity`, `scaledIdentity`) used across all test files.
- **Shared setup / constants**:
  - `DT = 1.0e-3` (filter timestep).
  - `IMU_BIAS_PROCESS_VAR = 1.0e-4` — each `TestIMU`'s bias process-noise covariance defaults to `1e-4 · I₃`.
  - State layout: `x = [q ; q_dot ; b_omega] ∈ R^{2n+3m}` where `n` = number of filtered joints, `m` = number of distinct IMUs (bias is per-IMU, not per-pair). `dim = 2n + 3m`.
  - `TestIMU` covariances: `biasProcessNoiseCovariance = 1e-4·I₃`; `angularVelocityNoiseCovariance = 1e-4·I₃` (gyro MEASUREMENT noise, kept separate from bias); `genericCovariance = 1e-4·I₃`; `linearAcceleration = (0,0,9.81)`; `orientation = identity quaternion`; `angularVelocity = (0,0,0)` initially.
- **Robot construction (`build`)**:
  - `Random random = new Random(seed)` — fixed seed per fixture.
  - Joint axes: for joint `i`, axis = unit vector along X if `i%3==0`, Y if `i%3==1`, Z if `i%3==2` (cyclic X,Y,Z,X,Y,Z…).
  - `RandomFloatingRevoluteJointChain(random, axes)`; `chain.nextState(random, CONFIGURATION, VELOCITY)` randomizes q and q̇; `updateFramesRecursively()`.
  - IMUs placed on `joints.get(imuBodyJointIndex[k]).getSuccessor()` (the link/body after that joint), body-fixed measurement frame = link's body-fixed frame.
  - Pair parameters: `IMUBasedJointStateEstimatorParameters("pair", true, parentImuName, childImuName, 0.0, 0.0)`.
  - Foot = successor of `footJointIndex` (if ≥0) else successor of `imuBodyJointIndex[footImuIndex]`.
  - Filter constructed with: sensorMap, pairParameters, feet, optional elevator (for mass-matrix process noise, else null), encoder position/velocity noise-STD lookups (nullable), velocity break-frequency lookup (nullable), useDirectVelocityMeasurement flag, DT, YoRegistry("test").
- **`shapes(baseSeed)`** — the parametrization (Java analogue of reference `SHAPES`), a list of 4 fixtures with seeds `baseSeed+1..+4`:
  1. `singlePair(baseSeed+1, numJoints=10, parent=1, child=9)` → n=8, m=2, 1 pair.
  2. `singlePair(baseSeed+2, 6, 1, 5)` → n=4, m=2, 1 pair.
  3. `singlePair(baseSeed+3, 4, 0, 3)` → n=3, m=2, 1 pair.
  4. `twoPairs(baseSeed+4, 10, a=1, b=5, c=9)` → n=8, m=3, 2 pairs (pairs (a,b),(b,c) share middle IMU b; foot on c link).
  - Note: `n` = number of joints strictly between parent-link and child-link along the chain (child index − parent index − 1). Encoder-only m=0 case is intentionally absent.
- **`shapesMassMatrix(baseSeed)`** — same 4 shapes but with the elevator handed to the filter (activates `Qa = σ_τ² M(q)⁻²` mass-matrix process noise instead of scalar-CWNA fallback).
- **`applyConsistentMotion(qTrue[], qdTrue[])`** (reference for measurement-consistency): zeros the root joint twist (static base), zeros all joints' qd, then for each filtered joint `i` sets `q=qTrue[i]`, `qd=qdTrue[i]`, and encoder = qTrue[i]; `updateFramesRecursively()`; then sets each IMU's gyro to its link's body-frame angular velocity via `refreshGyroFromTwist`. Because the pair measurement is `ω_child − R·ω_parent`, the zeroed base cancels and the relative gyro equals `J_ang·q̇` at the true state.
- **`refreshGyroFromTwist(imu)`**: reads the link body-fixed frame's twist, changes frame to the IMU measurement frame, sets gyro to its angular part.
- **Assertion / reference helpers** (must be replicated in Python):
  - `assertAllClose(actual, expected, tol, msg)`: shape check then elementwise `|a−e| ≤ tol`.
  - `assertSymmetric(a, tol, msg)`: `|a[r,c] − a[c,r]| ≤ tol` for upper triangle.
  - `assertPositiveSemiDefinite(a, msg)`: symmetric eigenvalues (EJML `EigenDecomposition`), require `min_eig ≥ −1e-6·max(max_eig, 1.0)`.
  - `symmetricEigenvalues(a)`: real parts of eigenvalues.
  - `block(a, row0, col0, rows, cols)`: submatrix extraction.
  - `spd(size, seed)`: **deterministic SPD generator** — fill `m` with `m.data[i] = sin(i + 1.0 + seed)` (row-major, i=0..size²−1); `a = m·mᵀ`; then add `size` to each diagonal (`a += size·I`) → symmetric positive-definite. (This is the reference for all seeded priors/noises; replicate exactly.)
  - `identity(size)`, `scaledIdentity(size, value)`, `trace(a)`.
- **Port notes**: Mecano `RandomFloatingRevoluteJointChain` + `nextState(random, CONFIGURATION, VELOCITY)` with a fixed `java.util.Random(seed)` — the Python port must reproduce the SAME random chain geometry and state, OR the port must abstract the geometry so tests can inject known transforms. This is the hardest cross-language dependency (Java `Random` LCG + Mecano link geometry). EJML matrices → NumPy. YoRegistry → no-op / registry stub. `IMUBasedJointStateEstimatorParameters` constructor signature `(name, enabled, parentImuName, childImuName, 0.0, 0.0)`. `TestIMU.setBiasProcessNoiseCovarianceNonFinite()` sets `[0,0]=NaN`. The `spd` and `sin`-based `genericH` references are language-agnostic and should be ported bit-for-bit.

### JointLevelKFTestSupport.java
- **Purpose**: Test-only bridge exposing the package-private `JointLevelKFPreFilter` constructors to tests in other packages. No tests, no assertions.
- **Contents**: `newPreFilter(sensorMap, pairParameters, feet, dt, parentRegistry)` and an overload adding `rootBody` (enables mass-matrix process noise `Qa = σ_τ²·M(q)⁻²`).
- **Port notes**: Pure plumbing; in Python this is just direct constructor access — no equivalent needed unless the port keeps a package-visibility distinction.

### SettableTestIMU.java
- **Purpose**: Public, reusable settable `IMUSensorReadOnly` for estimator tests (across packages). Like `TestIMU` but the specific force (linear acceleration) is settable — needed for gravity-consistent readings at tilt / free fall.
- **Setup / constants**: body-fixed measurement frame = link's body-fixed frame; `angularVelocity=(0,0,0)`; `linearAcceleration=(0,0,9.81)` (stationary specific force at identity, settable); `orientation=identity`; `biasProcessNoiseCovariance = 1e-4·I₃`; `genericCovariance = 1e-4·I₃`. All noise-covariance getters (orientation, angular-velocity, linear-accel, linear-accel-bias) return `genericCovariance` (`1e-4·I₃`); only `getAngularVelocityBiasProcessNoiseCovariance` returns the (corruptible) `biasProcessNoiseCovariance`.
- **Methods**: `setAngularVelocity(x,y,z)`, `setLinearAcceleration(x,y,z)` / `(Vector3DReadOnly)`, `setBiasProcessNoiseCovarianceNonFinite()` (sets `[0,0]=NaN`), `getMeasurementLinkBody()`.
- **Port notes**: Not exercised by the four @Test files here (used by other-package tests). Provide an equivalent settable IMU stub. Note gravity default 9.81 and 1e-4 covariance.

### SettableTestSensorMap.java
- **Purpose**: Public reusable settable `SensorOutputMapReadOnly`: serves a fixed IMU list plus settable per-joint encoder state (position/velocity/effort). Reusable across test packages.
- **Setup**: builds a `JointState(name)` per supplied joint (position/velocity/effort default 0, acceleration always 0, isJointEnabled always true). `getForceSensorOutputs()`=null; all time getters return 0.
- **Methods**: `setPosition(joint, q)`, `setVelocity(joint, qd)`.
- **Port notes**: Simple sensor-map stub; not directly used by the four test files (those use the fixture's inner `TestSensorMap`). Note the inner `TestSensorMap` in the fixture only exposes `setPosition` (no velocity) — the filter never reads encoder velocity.

### JointLevelKFStateTest.java
- **Purpose**: Ported from `test_state.py`. Locks in the bias-augmented state layout `x=[q; q_dot; b_omega] ∈ R^{2n+3m}`, encoder-seeded initial mean, and the initial-covariance block structure / prior confidences.
- **Shared setup / constants**: `INIT_POS_VAR=1.0e-6`, `INIT_VEL_VAR=1.0`, `INIT_BIAS_VAR=2.5e-3`. Prior ordering intent: position (encoders trusted) < bias < velocity (unknown at init). No `@BeforeEach`; each test builds fixtures via `shapes(seed)` or `singlePair(...)`.

#### testInitStateShapes
- **Scenario**: For each of `shapes(100L)`, call `filter.initialize()`.
- **Checks**: `getStateDimension() == 2n+3m`; state vector `x` is `dim × 1`; covariance `P` is `dim × dim`. (assertEquals exact for integer dims.)
- **Port notes**: fixed seed base 100L → sub-seeds 101..104.

#### testInferredDims
- **Scenario**: `shapes(200L)`.
- **Checks**: `n > 0`; `m ≥ 2` (a pair needs ≥2 IMUs); `2n+3m == f.dim`.

#### testInitStateDefaultsZero
- **Scenario**: `singlePair(1L, 8, 1, 7)` (n=5, m=2); set every filtered joint's encoder to 0.0; `initialize()`.
- **Checks**: every entry of `x` is exactly 0.0 (tol 0.0) — q from encoders (0), q̇ and bias default 0.

#### testQ0Seed
- **Scenario**: `singlePair(2L, 8, 1, 7)`; set encoder `i` to `0.11·(i+1)` for i=0..n−1; `initialize()`.
- **Checks**: `x[i] == encoders[i]` for the position segment, tol `1e-12` — position seeded from encoders in state order.

#### testXOrdering
- **Scenario**: `singlePair(3L, 8, 1, 7)`; all encoders = 1.0; `initialize()`.
- **Checks**: q segment (i=0..n−1) == 1.0 (tol 1e-12); q̇ segment (n..2n−1) == 0.0 (tol 0.0); bias segment (2n..dim−1) == 0.0 (tol 0.0). Verifies exact `[q; q_dot; b_omega]` ordering.

#### testMarginalBlocksMatchP
- **Scenario**: `shapes(300L)`; `initialize()`.
- **Checks**: diagonal of P: positions == `1e-6`, velocities == `1.0`, biases == `2.5e-3`, each tol `1e-12`. Verifies block prior variances.

#### testVelocityBlockExcludesBiasBlock
- **Scenario**: `singlePair(4L, 6, 1, 5)` (n=3, m=2); `initialize()`.
- **Checks**: velocity diagonal entries == `INIT_VEL_VAR=1.0` (tol 1e-12) AND `|P[i,i] − 2.5e-3| > 1e-6` — velocity marginal is pure velocity, not the bias block.

#### testPSymmetricAndPSD
- **Scenario**: `shapes(500L)`; `initialize()`.
- **Checks**: `assertSymmetric(P, 1e-12)` and `assertPositiveSemiDefinite(P)`.

#### testPriorConfidenceOrdering
- **Scenario**: `singlePair(6L, 8, 1, 7)`; `initialize()`.
- **Checks**: `pos = P[0,0]`, `vel = P[n,n]`, `bias = P[2n,2n]`; assert `pos < bias` and `bias < vel`. Ordering position < bias < velocity.

#### testPriorVariancesPositive
- **Scenario**: `singlePair(7L, 8, 1, 7)`; `initialize()`.
- **Checks**: every diagonal `P[i,i] > 0.0`.

### JointLevelKFPredictTest.java
- **Purpose**: Ported from `test_predict.py`. Locks in EKF time update `x⁻ = F·x`, `P⁻ = F·P·Fᵀ + Q` through the package-private `predict()` seam from a seeded prior.
- **Shared setup / constants**: `DT = 1e-3` (from fixture), `IMU_BIAS_VAR = 1e-4` (from fixture).
- **`seededPrior(f, seed)` reference**: builds mean `x` (dim×1): position segment `x[i] = i+1` (i=0..n−1); velocity segment `x[n+i] = i+1+100`; bias segment `x[2n+i] = i+1+1000` (i=0..3m−1); covariance `= spd(dim, seed)`; calls `filter.setStateForTest(x, P)`.

#### testShapesPreserved
- **Scenario**: `shapes(2000L)`; `seededPrior(f,0)`; `predict()`.
- **Checks**: state vector rows == dim; P is dim×dim.

#### testMeanPropagationExact
- **Scenario**: `shapes(2100L)`; `seededPrior(f,1)`; capture x, `predict()`, capture x'.
- **Checks**: position `x'[i] == x[i] + DT·x[n+i]` (tol `1e-9`) — closed-form constant-velocity integration; velocity segment unchanged (tol `1e-12`); bias segment unchanged (tol `1e-12`).

#### testCovarianceMatchesBuilders
- **Scenario**: `shapes(2200L)`; `seededPrior(f,2)`; read P, `F = getTransitionMatrix()`, `Q = getProcessNoise()` before predict; `predict()`; read actual P⁻.
- **Checks**: `expected = F·P·Fᵀ + Q` (built via EJML mult/multTransB/addEquals); `assertAllClose(actual, expected, 1.0e-6)`. Verifies covariance propagation matches the filter's own F and Q builders.

#### testCovarianceSymmetricPSD
- **Scenario**: `shapes(2300L)`; `seededPrior(f,3)`; `predict()`.
- **Checks**: `assertSymmetric(P, 1e-6)`, `assertPositiveSemiDefinite(P)`.

#### testBiasBlockGrowthDiagonal
- **Scenario**: `singlePair(2400L, 8, 1, 7)`; `seededPrior(f,4)`; extract bias block `[2n:2n+3m, 2n:2n+3m]` before and after `predict()`.
- **Checks**: `increment = after − before == scaledIdentity(3m, DT·IMU_BIAS_VAR)` i.e. `dt·1e-4·I` (tol `1e-9`). Bias marginal grows by exactly the bias process noise (F identity on bias block, no coupling).

#### testDeterministic
- **Scenario**: `singlePair(2500L, 8, 1, 7)`; `x0[i] = 0.01·(i+1)`, `p0 = spd(dim,5)`; `setStateForTest`, `predict()` → (xa, pa); reset same state, `predict()` again.
- **Checks**: results bit-identical, tol `0.0` for both x and P.

#### testDiffusePositionVarianceGrows
- **Scenario**: `singlePair(2600L, 8, 1, 7)`; `initialize()`; record position diagonals; run 5 `predict()` steps (no measurement).
- **Checks**: each step P stays PSD; every position variance `cur ≥ prev − 1e-9` (never shrinks under measurement-free predict).

### JointLevelKFUpdateTest.java
- **Purpose**: Ported from `test_update.py`. Locks in the Joseph-form measurement update `ν=z−Hx⁻`, `S=HP⁻Hᵀ+R`, `K=P⁻Hᵀ S⁻¹`, `x⁺=x⁻+Kν`, `P⁺=(I−KH)P⁻(I−KH)ᵀ+KRKᵀ` via package-private `josephUpdate(H,z,R)`, checked against an explicit-inverse reference KF.
- **Reference helpers**:
  - `genericH(k, dim, seed)`: `H[r,c] = sin(0.37·(r·dim + c + 1) + seed)` — deterministic non-geometric Jacobian. Replicate exactly.
  - `seededPrior(f, seed)`: mean `x[i] = 0.1·(i+1)`, cov `P = spd(dim, seed)`, `setStateForTest`; returns P.
  - `referenceUpdate(x,P,H,z,R)`: full explicit-inverse KF (transpose, `PHt=P·Hᵀ`, `S=H·PHt+R`, `Sinv=inv(S)`, `K=PHt·Sinv`, `ν=z−H·x`, `xNew=x+K·ν`, `Pnew=(I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ`). This is the reference to reproduce in NumPy.

#### testShapesAndReferenceKF
- **Scenario**: `shapes(3000L)`; `P=seededPrior(f,1)`, `x=getStateVector()`; `H=genericH(3, dim, 2)`; `z[i]=0.05·(i+1)` (3×1); `R=spd(3,7)`. Compute `ref=referenceUpdate(...)`; call `josephUpdate(H,z,R)`.
- **Checks**: state rows == dim; `x⁺` matches `ref[0]` (tol `1e-6`); `P⁺` matches `ref[1]` (tol `1e-6`).

#### testCovarianceSymmetricPSD
- **Scenario**: `shapes(3100L)`; `seededPrior(f,3)`; `H=genericH(3,dim,4)`; `z=0` (3×1); `R=spd(3,9)`; `josephUpdate`.
- **Checks**: `assertSymmetric(P⁺, 1e-6)`, `assertPositiveSemiDefinite(P⁺)`.

#### testCovarianceShrinks
- **Scenario**: `shapes(3200L)`; `prior=seededPrior(f,5)`; `H=genericH(3,dim,6)`; `z=0`; `R=spd(3,11)`; `josephUpdate`; `post=getCovariance()`.
- **Checks**: `diff = prior − post` is PSD; `trace(post) ≤ trace(prior) + 1e-6`. A measurement cannot increase uncertainty.

#### testEncoderPullTinyR
- **Scenario**: `singlePair(3300L, 8, 1, 7)`; `seededPrior(f,13)`; `target = 0.3 + x[0]`; `H` = 1×dim with `H[0,0]=1` (observe position state 0); `z[0]=target`; `R=[[1e-16]]`; `josephUpdate`.
- **Checks**: posterior `x[0] == target` (tol `1e-3`) — extremely confident encoder pulls posterior onto the reading.

#### testBiasObservability
- **Scenario**: `singlePair(3400L, 8, 1, 7)`; set IMU0 gyro `(0.05,−0.03,0.02)`, IMU1 gyro `(−0.04,0.06,−0.01)`; `initialize()`; `buildStackedMeasurementForTest()`; read `H=getStackedMeasurementJacobian()`, `z=getStackedMeasurementResidual()`, `R=getStackedMeasurementNoise()`. Single pair, no trusted feet ⇒ stacked measurement is exactly the pair's 3-row block. Read `parentBias = getPairParentBiasColumn(0)`, `childBias = getPairChildBiasColumn(0)`. `josephUpdate(H,z,R)`.
- **Checks**: sum of `|Δ|` over the 6 bias entries (3 parent + 3 child) `> 1e-9` — a nonzero IMU innovation must move the per-IMU bias estimate.

#### testDeterministic
- **Scenario**: `singlePair(3500L, 8, 1, 7)`; `x0[i]=0.02·(i+1)`, `p0=spd(dim,21)`, `H=genericH(3,dim,22)`, `z=0`, `R=spd(3,23)`; run `josephUpdate` twice from same state.
- **Checks**: identical x and P, tol `0.0`.

### JointLevelKFMeasurementTest.java
- **Purpose**: Ported from `test_measurement.py` (Rev. 2). Locks in the measurement model: encoder Jacobian `[I_n | 0]`, and relative-gyro measurement `z = ω_child − R·ω_parent` with Jacobian rows `[0 | scattered J | +R_child at child-bias | −R_parent at parent-bias]`. Uses the STACKED measurement seam: one stacked `z_g/H_g/R_g` over all pairs (rows `[3e, 3e+3)` for pair e) plus stance anchors; with no trusted feet anchors are inactive (K=0), so a single-pair stacked build == the old 3×dim H, 3×1 z, 3×3 R.
- **Reference helpers**:
  - `pairJacobian(f, pairIndex)`: `buildStackedMeasurementForTest()`, then `block(Hg, getStackedRowForPair(pairIndex), 0, 3, dim)`.
  - `pairResidual(f, pairIndex)`: same for the residual (3×1 slice).

#### testEncoderJacobianStructure
- **Scenario**: `shapes(4000L)`; `Henc = getEncoderJacobian()`.
- **Checks**: `Henc[0:n, 0:n] == I_n` (tol `1e-12`); `Henc[0:n, n:dim] == 0` (tol `1e-12`). Encoder rows observe q, independent of q̇ and bias.

#### testEncoderPredictsPosition
- **Scenario**: `singlePair(4050L, 8, 1, 7)`; `x[i]=0.1·(i+1)`; compute `hx = Henc·x`.
- **Checks**: `hx[i] == x[i]` for i=0..n−1 (tol `1e-12`) — predicted encoder measurement equals the position segment.

#### testPairJacobianStructure
- **Scenario**: `shapes(4100L)`; `H = pairJacobian(f, 0)` (3×dim).
- **Checks**: `H` is 3×dim; q-columns `H[:, 0:n] == 0` (tol `1e-12`); every velocity column in `[n, 2n)` NOT in `getPairVelocityColumns(0)` is exactly 0.0; every bias column in `[2n, dim)` outside `[parentBias, parentBias+3)` and `[childBias, childBias+3)` is exactly 0.0. Verifies sparsity structure.

#### testBiasColumnsOfHgAreExactlyL
- **Scenario**: `shapes(4150L)` (incl. two-pair shared-IMU); `buildStackedMeasurementForTest()`; `Hg=getStackedMeasurementJacobian()`, `L=getMixingOperator()`.
- **Checks**: `Hg.numRows == L.numRows`; bias-column block `Hg[:, 2n : 2n+3m] == L` bit-identically (tol `0.0`). Central Rev. 2 identity (SPEC §5.3): bias columns of H_g ARE the mixing operator L.

#### testChildBiasBlockIsIdentity
- **Scenario**: `singlePair(4200L, 8, 1, 7)`; `H = pairJacobian(f,0)`; extract `childBlock = block(H, 0, getPairChildBiasColumn(0), 3, 3)`.
- **Checks**: `childBlock == I₃` (tol `1e-9`) — child IMU frame is the Jacobian frame ⇒ `R_child = I` ⇒ `+I` in child-bias block.

#### testParentBiasBlockIsNegativeRotation
- **Scenario**: `singlePair(4300L, 8, 1, 7)`; `H = pairJacobian(f,0)`; `parentBlock = block(H, 0, getPairParentBiasColumn(0), 3, 3)`; `rotation = −parentBlock`.
- **Checks**: `rotationᵀ·rotation == I₃` (tol `1e-9`) — parent-bias block is `−R_parent`, a negated rotation (orthonormal).

#### testRelativeGyroParentZero
- **Scenario**: `singlePair(4400L, 8, 1, 7)`; IMU0 (parent) gyro `(0,0,0)`, IMU1 (child) gyro `(0.1,−0.2,0.3)`; `z = pairResidual(f,0)`.
- **Checks**: `z == (0.1, −0.2, 0.3)` (tol `1e-9` each) — `ω_parent=0 ⇒ z=ω_child` in Jacobian (child) frame.

#### testRelativeGyroChildZeroPreservesNorm
- **Scenario**: `singlePair(4500L, 8, 1, 7)`; IMU0 gyro `(0.1,−0.2,0.3)`, IMU1 gyro `(0,0,0)`; `z = pairResidual(f,0)`.
- **Checks**: `|z| == sqrt(0.1²+0.2²+0.3²)` (tol `1e-9`) — `ω_child=0 ⇒ z=−R·ω_parent`, rotation preserves norm.

#### testMeasurementNoiseSymmetricPSD
- **Scenario**: `shapes(4600L)`; `buildStackedMeasurementForTest()`; `R=getStackedMeasurementNoise()`.
- **Checks**: `R.numRows == 3·getNumberOfPairs()`; `assertSymmetric(R, 1e-12)`; `assertPositiveSemiDefinite(R)`.

#### testMeasurementNoiseUsesGyroMeasurementCovariance
- **Scenario**: `singlePair(4700L, 8, 1, 7)`; parent = IMU0, child = IMU1. Set ANISOTROPIC gyro MEASUREMENT covariances: parent `diag(4e-4, 1e-6, 2.5e-5)`, child `diag(9e-4, 1.6e-5, 4.9e-6)`; set tiny bias process covariances: both `diag(1e-9,1e-9,1e-9)`. `buildStackedMeasurementForTest()`; `R = getStackedMeasurementNoise()` (3×3, no anchors).
- **Checks**: Independently build `expected = R_c·Σ_c·R_cᵀ + R_p·Σ_p·R_pᵀ`, where Jacobian frame = child's body-fixed frame so `R_child = I` and `R_parent` = parent-measurement-frame→child-frame rotation (read from euclid `getTransformToDesiredFrame`, converted via `JointLevelKFPreFilter.set_matrix`). So `expected = rotParent·Σ_parent·rotParentᵀ + Σ_child`. Assert `R == expected` (tol `1e-12`). Also assert `trace(R) = R[0,0]+R[1,1]+R[2,2] > 1e-5` (guard: bias-built R would have trace ~6e-9). Regression: R built from gyro MEASUREMENT noise, not bias random-walk covariance.
- **Port notes**: Uses `JointLevelKFPreFilter.set_matrix(dmatrix, euclidRotation)` static helper and euclid `RigidBodyTransform.getTransformToDesiredFrame`.

#### testMeasurementNoiseIndependentOfBiasProcessCovariance
- **Scenario**: `singlePair(4800L, 8, 1, 7)`; `buildStackedMeasurementForTest()`, capture `rBefore`; scale every IMU's bias process covariance to `diag(0.1,0.1,0.1)` (1000×); rebuild, capture `rAfter`.
- **Checks**: `rAfter == rBefore` (tol `0.0`) — measurement noise R independent of bias process covariance.

---
#### Cross-cutting port notes (jointLevel unit tests)
- **Fixed seeds** (all `java.util.Random(seed)` / `spd(seed)` — deterministic): State test bases 100/200; singlePair seeds 1,2,3,4,6,7. Predict base 2000/2100/2200/2300 and singlePair 2400,2500,2600 with `spd` seeds 0–5. Update base 3000/3100/3200 and singlePair 3300,3400,3500; `genericH` seeds 2,4,6,22; `spd` seeds 1,3,5,7,9,11,13,21,23. Measurement bases 4000,4100,4150,4600 and singlePair 4050,4200,4300,4400,4500,4700,4800. Each `shapes(base)` uses sub-seeds `base+1..base+4` for the 4 chain geometries.
- **Deterministic matrix references to port bit-for-bit**: `spd(size,seed)` (`sin(i+1+seed)` fill, `m·mᵀ + size·I`); `genericH(k,dim,seed)` (`sin(0.37·(r·dim+c+1)+seed)`); the `seededPrior` mean patterns (predict: `i+1`, `i+1+100`, `i+1+1000`; update: `0.1·(i+1)`).
- **Filter test seams the Python port must expose**: `initialize()`, `predict()`, `josephUpdate(H,z,R)`, `setStateForTest(x,P)`, `getStateVector()`, `getCovariance()`, `getStateDimension()`, `getTransitionMatrix()`, `getProcessNoise()`, `getEncoderJacobian()`, `buildStackedMeasurementForTest()`, `getStackedMeasurementJacobian/Residual/Noise()`, `getStackedRowForPair(i)`, `getMixingOperator()`, `getPairParentBiasColumn(i)`, `getPairChildBiasColumn(i)`, `getPairVelocityColumns(i)`, `getNumberOfPairs()`, `getNumberOfFilteredJoints()`, `getNumberOfIMUs()`, `getFilteredJointsInStateOrder()`, static `set_matrix(...)`.
- **Hard dependency**: Mecano `RandomFloatingRevoluteJointChain` + `nextState(random, CONFIGURATION, VELOCITY)` and euclid frame math generate the actual link rotations that the gyro-Jacobian and R-rotation tests depend on. The Python port must either reproduce identical geometry from the Java `Random` sequence (unlikely feasible) or restructure these geometry-dependent tests around injected/known transforms while keeping the pure-linear-algebra tests (state, predict, Joseph update, spd/genericH references) exact.
- EJML → NumPy; `assertPositiveSemiDefinite` uses eigenvalues with tolerance `min_eig ≥ −1e-6·max(max_eig,1)`. YoRegistry → stub. Tests are plain JUnit 5 `@Test` iterating over `shapes(...)` lists (not `@ParameterizedTest`/`@RepeatedTest`), so Python can loop over the same shape list.

---

## jointLevel behavior / property tests

### JointLevelKFFilterTest.java
- **Purpose**: End-to-end whole-filter test driving the full predict→update orchestration (`computeJointState()` phase 1 + `computeImuBiases(feet)` phase 2) over trajectories, checking covariance stays symmetric PSD and bounded, the run is deterministic, encoder tracking holds, and the stance anchor makes the base bias converge to a constant gyro offset. Ported from `tests/jointKF/test_filter.py`.
- **Shared setup**: No `@BeforeEach`. Private helper `tick(f)` calls `f.filter.computeJointState()` then `f.filter.computeImuBiases(f.feet)`. Fixtures built per test via `JointLevelKFTestFixture.shapes(seed)` or `singlePair(seed, numChainJoints, imuAfterJoint, footAfterJoint)`. `f.dim` = full state dim, `f.n` = number of filtered joints, `f.m` = number of IMUs.

#### testTrajectoryFiniteAndShapes
- **Scenario**: For every fixture from `shapes(5000L)`: `initialize()`, then 50 ticks (predict+update, sensors left at fixture defaults).
- **Checks**: state vector `x` has `f.dim` rows; covariance `P` has `f.dim` rows; every element of `x` and every element of `P` is finite (`Double.isFinite`).

#### testCovariancePSDAlongTrajectory
- **Scenario**: For every fixture from `shapes(5100L)`: `initialize()`, 20 ticks; after each tick inspect `P`.
- **Checks**: `assertSymmetric(P, 1.0e-6)` and `assertPositiveSemiDefinite(P)` at each of the 20 ticks.

#### testDeterministic
- **Scenario**: Two fixtures `a`, `b` both `singlePair(5200L, 8, 1, 7)` (same seed ⇒ identical geometry). Both IMUs given fixed angular velocities: IMU0 `(0.02, -0.01, 0.03)`, IMU1 `(0.01, 0.02, -0.02)`; both `initialize()`. 30 ticks on each.
- **Checks**: `assertAllClose(a.x, b.x, 0.0)` and `assertAllClose(a.P, b.P, 0.0)` — exact bitwise equality (tolerance 0.0).
- **Port notes**: Requires the estimator be fully deterministic (no RNG in the filter itself). Port must ensure identical geometry from identical seed.

#### testEncoderTracking
- **Scenario**: `singlePair(5300L, 8, 1, 7)`. Set each filtered joint's encoder to `target[i] = 0.2*(i+1) - 0.5`. `initialize()`, 100 ticks.
- **Checks**: for each joint `i`, `assertEquals(target[i], x[i], 5.0e-3)` — position estimate tracks encoder within 5e-3 rad.

#### testStancePhaseBiasConvergence
- **Scenario**: `singlePair(5400L, 8, 1, 7)`. All IMUs given constant gyro offset `(ox,oy,oz)=(0.01,-0.02,0.03)`; all filtered joints' encoders set to 0. `initialize()`, **20,000** ticks.
- **Checks**: base-IMU residual bias `f.filter.getAngularVelocityBiasInIMUFrame(imus[0])` → offset within 2.0e-3 on each of X, Y, Z.
- **Port notes**: `getAngularVelocityBiasInIMUFrame` returns bias expressed in IMU frame. The 20k-tick convergence is the key numerical property.

#### testCovarianceBounded
- **Scenario**: `singlePair(5500L, 8, 1, 7)`. `initialize()`, record `initialTrace = trace(P)`, 2000 ticks, record `finalTrace`.
- **Checks**: `finalTrace` finite AND `finalTrace <= initialTrace*10.0 + 1.0`.

---

### JointLevelKFTrajectoryTest.java
- **Purpose**: Behavioral tests of phase-1 `computeJointState()` (predict + encoder + pair-gyro updates, NOT `computeImuBiases`) on an analytically-consistent per-joint sinusoid where encoder positions and IMU gyros are both derived from the same true q/q̇ on the live Mecano chain via `applyConsistentMotion`. Also regression-covers NaN hardening and covariance coupling structure.
- **Shared setup**: Constants: `AMP = 0.10` rad, `FREQ_HZ = 0.5`, `OMEGA = 2π·0.5` rad/s, `DT = 1e-3` s. Helper `trajectory(tick, n, q[], qd[])`: for joint `i`, `phase = OMEGA*tick*DT + i*π/n`; `q[i] = AMP*sin(phase)`, `qd[i] = AMP*OMEGA*cos(phase)`. Helper `assertAllFinite(matrix, msg)` checks every element finite. Fixtures via `singlePair(seed, 8, 1, 7)` and `singlePairPoisonBias(...)`.

#### testPositionTracksTrajectory
- **Scenario**: `singlePair(9100L, 8, 1, 7)`. 300 ticks: each tick compute trajectory, `applyConsistentMotion(q,qd)`, `computeJointState()`.
- **Checks**: at final tick (index 299), for each joint `assertEquals(q[i], x[i], 5.0e-3)`.

#### testVelocityConverges
- **Scenario**: `singlePair(9101L, 8, 1, 7)`. `warmup=500`, `total=3000` ticks; after warmup track per-joint max |estimated velocity| (state index `n+i`).
- **Checks**: at final tick, `assertEquals(qd[i], x[n+i], 3.0e-2)`; and `maxAbsEstimatedVelocity[i] > 0.5*peakTrueVelocity` where `peakTrueVelocity = AMP*OMEGA` (guards velocity is actually observed).

#### testBiasStaysSmallWithZeroTrueBias
- **Scenario**: `singlePair(9102L, 8, 1, 7)`. 1500 ticks of consistent motion (zero true bias).
- **Checks**: for each IMU, `getAngularVelocityBiasInIMUFrame(imu).norm() < 5.0e-3`.

#### testCovarianceStaysPsdAndBounded
- **Scenario**: `singlePair(9103L, 8, 1, 7)`. `warmup=200` ticks, record `traceWarmup`; then ticks 200..1999, and every 50th tick check `P`.
- **Checks**: `assertSymmetric(P, 1.0e-6)`, `assertPositiveSemiDefinite(P)` at each checkpoint; final `traceFinal <= 10.0*traceWarmup + 1.0`.

#### testTransientNonFiniteInputRecovers
- **Scenario**: `singlePair(9104L, 8, 1, 7)`. Ticks 0..99 clean warm-up. Ticks 100..104 "bad window": after `applyConsistentMotion`, set IMU0 gyro to `(NaN,NaN,NaN)` and joint0 encoder position to `NaN` via `f.sensorMap.setPosition`. Then ticks 105..1104 restore consistent input.
- **Checks**: during bad window each tick, `assertAllFinite(x)` and `assertAllFinite(P)` (updates skipped, no NaN latch). After recovery, at tick 1104: `assertEquals(q[i], x[i], 5.0e-3)` and `assertEquals(qd[i], x[n+i], 3.0e-2)`.
- **Port notes**: NaN-gating logic in updates must be replicated exactly (bad measurements skipped, not propagated).

#### testPoisonedBiasCovarianceDoesNotLatchNaN
- **Scenario**: `singlePairPoisonBias(9105L, 8, 1, 7, 0)` — IMU index 0's bias process-noise covariance is non-finite BEFORE construction. Assert Q and F finite up front, then 200 ticks of consistent motion.
- **Checks**: `assertAllFinite(getProcessNoise())`, `assertAllFinite(getTransitionMatrix())` at start; `assertAllFinite(x)`, `assertAllFinite(P)` after 200 ticks. Verifies construction-time guard skips the poisoned IMU covariance and per-tick R3 guard keeps the pair update out.

#### testSingularInnovationIsSkippedNotLatched
- **Scenario**: `singlePair(9106L, 8, 1, 7)`. Drive from rest: `trajectory(0,...)` positions with zero velocities, `applyConsistentMotion(q0, zeros)`, `initialize()`, one `predict()` (finite PD prior). Record `xBefore`, `pBefore`. Construct rank-deficient `H` (3×dim): row0 and row1 both set column `velocityIndex=n` (joint 0 velocity) to 1.0 (duplicate rows), row2 zero; `z` = zeros(3); `R` = zeros(3×3) (exactly zero, no regularization → `S = HPHᵀ` singular). Call `josephUpdate(H, z, R)`.
- **Checks**: `assertAllFinite(xAfter)`, `assertAllFinite(pAfter)`; `assertAllClose(xAfter, xBefore, 0.0)` and `assertAllClose(pAfter, pBefore, 0.0)` — singular update skipped, state/covariance exactly unchanged.
- **Port notes**: exercises the direct `predict()` and `josephUpdate(H,z,R)` internal API. Singular-innovation detection/skip must be replicated.

#### testCovarianceEmbedsKinematicCoupling
- **Scenario**: `singlePair(9107L, 8, 1, 7)`. Drive from rest as above, `initialize()`, one `predict()`. Then 200 ticks of consistent moving trajectory.
- **Checks**: After one predict, `P0`: within-joint q↔q̇ coupling present for each joint `i`: `|P0[i, n+i]| > 1.0e-4`; no cross-joint velocity coupling: for `i<j`, `|P0[n+i, n+j]| < 1.0e-10`. After 200 pair-gyro update ticks, `P1`: `assertSymmetric(1e-6)`, `assertPositiveSemiDefinite`; compute max cross-joint velocity correlation `|P1[n+i,n+j]| / sqrt(P1[n+i,n+i]*P1[n+j,n+j])` over `i<j`, require `maxCrossCorrelation > 0.02` (kinematic tree embeds via shared Jacobian J(q)); within-joint coupling persists `|P1[i,n+i]| > 1.0e-6`.
- **Port notes**: verifies CWNA accel noise + double-integrator F create within-joint coupling, and pair-gyro Jacobian creates cross-joint coupling.

---

### JointLevelKFStandingStabilityTest.java
- **Purpose**: Reconciliation/diagnostic harness for the Alex002 hardware finding that joint VELOCITY covariance blows up in one `predict()` while POSITION covariance stays sane, driven by Schur process noise `Qa = σ_τ² Λ⁻²`. Quantifies Λ conditioning, proves predict()/Qa (not the gyro update) is the inflation source, ranks candidate fixes, and provides the property test the fix must pass. Several tests print diagnostics via `System.out`.
- **Shared setup**: Constants: `DT = 1e-3`, `SIGMA_TAU = 5.0`, `QA_MAX = 900.0` (= (30 rad/s²)²), `ROTOR_DEFAULT = 0.005`, `TARGET_QDD_STD = 20.0` rad/s², `ONE_TICK_QDD_VARIANCE_BOUND = 1.0` (rad/s)². Fixtures via `shapesMassMatrix(seed)`. Local reference/helper methods:
  - `referenceSchur(f)` → `{Λ, M_ff}`: independent EJML/LU reference. Builds spanning joint set from each filtered joint up to `f.rootJoint`; constructs `CompositeRigidBodyMassMatrixCalculator` over `[rootJoint]+spanning`; extracts filtered columns (in filter state order) and nuisance columns (base 6-DoF + gap joints); `Λ = M_ff − M_Nf^T M_NN⁻¹ M_Nf` (LU invert `M_NN`).
  - `condSPD(a)` = maxEig/minEig via `symmetricEigenvalues`; `minEig(a)`; `maxDiag(a)`; `maxAbs(a)`.
  - `qaFromLambda(Λ, σ_τ)` = `σ_τ² · Λ⁻¹·Λ⁻¹` (LU invert then square).
  - `qaFromLambdaEff(f, Λ, σ_τ)`: `Lambda_eff = Λ + diag(reflectedRotorInertiaForNameOrDefault(jointName))`, then `qaFromLambda(Lambda_eff, σ_τ)`.
  - `floorSpectrum(a, floor)`: `A + max(0, floor - λ_min)·I`.
  - `accelStd(inv, σ[])`: per-joint `sqrt(Σ_j (inv[i,j]·σ[j])²)`.

#### testSchurConditioningAndPredictInflatesVelocityCovariance
- **Scenario**: For each fixture from `shapesMassMatrix(9100L)`: compute `Λ = referenceSchur(f)[0]`, `qa = qaFromLambdaEff(f, Λ, SIGMA_TAU)`, find worst joint (max `qa[i,i]`). Seed sane standing prior: `x=0`, `P = 1e-4·I(dim)`, via `setStateForTest`. One `predict()`.
- **Checks**: one-predict `P_qdqd` increment at worst joint = `dt·Qa[worst,worst]` (Van Loan) within `1e-4·max(1, expected)`. Proves predict/Qa is the inflation source. (Tolerance loosened to 1e-4 due to Gram vs LU numerics on ill-conditioned Λ_eff.)

#### testCandidateFixesBoundQa
- **Scenario**: For each fixture from `shapesMassMatrix(9200L)`: from `referenceSchur` get Λ and M_ff. Compute max-diag Qa for: current (`qaFromLambda(Λ,50)`), σ_τ=5, σ_τ=1, eigenvalue-floored Λ (`floorSpectrum(Λ,0.05)` at σ_τ=50), locked-base (`qaFromLambda(M_ff,50)`).
- **Checks**: `locked <= curr*(1+1e-6)` — locked-base Qa (`M_ff⁻²`) ≤ Schur Qa (`Λ⁻²`) by PSD ordering `Λ ⪯ M_ff`. (Others only printed.)

#### testAccelerationEqualizedSigmaTauFloorsAtTargetAndEqualizesDominantTerm
- **Scenario**: For each fixture from `shapesMassMatrix(9500L)`: `Lambda_eff = Λ + diag(rotor)`, `inv = Lambda_eff⁻¹`. Equalized `σEq[i] = TARGET_QDD_STD / |inv[i,i]|`; uniform `σUni = sqrt(ΣσEq²/n)`. `stdEq = accelStd(inv, σEq)`.
- **Checks**: (a) FLOOR: each `stdEq[i] >= TARGET_QDD_STD*(1-1e-6)`; (b) DOMINANT-TERM EQUALIZATION exact: `|inv[i,i]|·σEq[i]` == `TARGET_QDD_STD` within `1e-9·TARGET`; sanity: `uniOwnSpread = invMax/invMin > 1+1e-9` (fixture has nontrivial inertia spread).

#### testSinglePredictVelocityVarianceInjectionIsPhysicallyBounded
- **Scenario**: For each fixture from `shapesMassMatrix(9300L)`: seed sane prior `x=0`, `P=1e-4·I`, one `predict()`; measure worst per-joint velocity-variance injection `pAfter[n+i,n+i] - pBefore[n+i,n+i]`.
- **Checks**: `pAfter` PSD; `assertSymmetric(pAfter, 1e-9·maxAbs(pAfter)+1e-12)`; overall worst injection across all fixtures `<= ONE_TICK_QDD_VARIANCE_BOUND = 1.0` (rad/s)². This is the regression gate for the Alex002 mechanism.

#### testRotorInertiaFloorBoundsQaForNearSingularLambda
- **Scenario**: Hand-built 3×3 diagonal `Λ = diag(2.0, 0.02, 1.5)` (one near-singular mode), `σ_τ = 5.0`, `rotor = 0.05`. `qaUnfloored = qaFromLambda(Λ, σ_τ)`; `Lambda_eff = Λ + rotor·I`; `qaFloored = qaFromLambda(Lambda_eff, σ_τ)`. Weyl bound `= σ_τ²/(λ_min(Λ)+rotor)²`.
- **Checks**: `maxDiag(qaUnfloored) > QA_MAX*1.5` (blow-up); `maxDiag(qaFloored) <= weylBound*(1+1e-9)`; `maxDiag(qaFloored) < maxDiag(qaUnfloored)`.
- **Port notes**: Fully self-contained numeric test (no fixture) — highly portable.

---

### JointLevelKFMassMatrixNoiseTest.java
- **Purpose**: Locks in the Schur-complement process noise (SPEC §3.2): unmodeled joint torque `w_τ` through floating-base dynamics gives `δq̈ = Λ⁻¹ w_τ`, `Λ = M_jj − M_jb M_bb⁻¹ M_bj`; with per-joint `Σ_τ`, `Qa = Λ_eff⁻¹ diag(σ_τ²) Λ_eff⁻ᵀ` (Gram form), Van-Loan discretized (`dt³/3·Qa`, `dt²/2·Qa`, `dt·Qa`). Expected values computed independently of the filter (second `CompositeRigidBodyMassMatrixCalculator`, plain EJML LU).
- **Shared setup**: Constants: `DT = 1e-3`, `SIGMA_TAU = 5.0`, `QA_MAX = 900.0`, `SIGMA_ACCEL = 50.0` (fallback), `ROTOR_DEFAULT = 0.005`. `relTol(expected) = 3.0e-3·max(1e-30, elementMaxAbs(expected))` (loosened from 1e-8 because filter uses Cholesky/Gram, reference uses LU). Reference methods:
  - `referenceSchur(f)` → `{Λ, M_ff}`: identical construction to StandingStability's; asserts `baseCols.length==6` and `M_NN` inverts.
  - `referenceQa(f)`: `Lambda_eff = Λ + diag(reflectedRotorInertiaForNameOrDefault)`; `Y[i,j] = Lambda_eff⁻¹[i,j]·referenceSigmaTau(f,j)`; `Qa = Y Yᵀ` via `multTransB`.
  - `referenceSigmaTau(f, i)`: `alphaForName(name)·effortLimitUpper` if finite/positive else `SIGMA_TAU` (random chain → fallback).
  - Fixtures via `singlePairMassMatrix(...)`, `shapesMassMatrix(...)`, and scalar `singlePair(...)`.

#### testMassMatrixPathEnabledOnlyWithModel
- **Scenario**: `singlePairMassMatrix(6000L, 8, 1, 7)` vs `singlePair(6000L, 8, 1, 7)`.
- **Checks**: model-provided fixture `isUsingMassMatrixProcessNoise()==true`; no-model fixture `==false`.

#### testProcessNoiseEqualsVanLoanOfSchurComplementInverseSquared
- **Scenario**: For each `shapesMassMatrix(6100L)`: `qa = referenceQa(f)`, `Q = filter.getProcessNoise()`.
- **Checks**: `block(Q,0,0,n,n) ≈ dt³/3·qa`, `block(Q,0,n) ≈ dt²/2·qa`, `block(Q,n,0) ≈ dt²/2·qa`, `block(Q,n,n) ≈ dt·qa`, all within `relTol(expected)`.

#### testSchurComplementIsSymmetricPDAndDominatedByLockedInertia
- **Scenario**: For each `shapesMassMatrix(6800L)`: bent config `q[i] = 0.7·(±1)` (alternating sign), `qd=0`, `applyConsistentMotion`. Compute `{Λ, M_ff}`.
- **Checks**: `assertSymmetric(Λ, 1e-9·max(1,maxAbs(Λ)))`, `assertPositiveSemiDefinite(Λ)`; `M_ff − Λ ⪰ 0` (PSD, since `= M_jN M_NN⁻¹ M_Nj`).

#### testVanLoanBlocksAreExactlySymmetric
- **Scenario**: For each `shapesMassMatrix(6900L)`: `Q = getProcessNoise()`, extract qq, qdqd, qqd, qdq blocks.
- **Checks**: `qq[i,j]==qq[j,i]` (tol 0.0), `qdqd[i,j]==qdqd[j,i]` (0.0), `qqd[i,j]==qdq[j,i]` (0.0) — exact bit-symmetry (filter symmetrizes with 0.5(A+Aᵀ)).

#### testBiasBlockUntouchedByMassMatrixPath
- **Scenario**: For each `shapesMassMatrix(6200L)`, `mDof=3·m`.
- **Checks**: `block(Q, 2n, 2n, mDof, mDof) ≈ scaledIdentity(mDof, DT·IMU_BIAS_PROCESS_VAR)` (tol 1e-12); joint↔bias cross blocks are zero (tol 1e-12).

#### testProcessNoiseSymmetricPSDOnMassMatrixPath
- **Scenario**: For each `shapesMassMatrix(6300L)`.
- **Checks**: `assertSymmetric(Q, 1e-12)`, `assertPositiveSemiDefinite(Q)`.

#### testProcessNoiseCouplesJointsThroughInertia
- **Scenario**: `singlePairMassMatrix(6400L, 10, 1, 9)` (n=8). Extract q̇q̇ block.
- **Checks**: max off-diagonal `> 0.0` (dense M⁻² couples joints). Scalar path `singlePair(6400L, 10, 1, 9)`: all off-diagonals of q̇q̇ block exactly 0.0.

#### testProcessNoiseIsConfigurationDependent
- **Scenario**: `singlePairMassMatrix(6500L, 10, 1, 9)` (n=8). Record q̇q̇ block; drive to large alternating bend `q[i]=0.9·(±1)`, `applyConsistentMotion`; `updateProcessNoiseFromMassMatrixForTest()`.
- **Checks**: max element change `> 1e-6·elementMaxAbs(qBefore)` (Q changed); new q̇q̇ block ≈ `dt·referenceQa(f)` at new config within `relTol`.

#### testPredictRefreshesQAndKeepsCovarianceSymmetricPSD
- **Scenario**: `singlePairMassMatrix(6600L, 8, 1, 7)`. Seed `x0=0`, `P0=spd(dim, 42L)` via `setStateForTest`; config `q[i] = -0.8+0.2i`, `applyConsistentMotion`; `predict()`.
- **Checks**: q̇q̇ block ≈ `dt·referenceQa(f)` within `relTol` (predict itself refreshes Q); `assertSymmetric(P, 1e-9·max(1,maxAbs(P)))`, `assertPositiveSemiDefinite(P)`.

#### testMassMatrixAndScalarPathsDiffer
- **Scenario**: `singlePairMassMatrix(6700L, 8, 1, 7)` vs `singlePair(6700L, 8, 1, 7)`. `sa2dt = SIGMA_ACCEL²·DT`.
- **Checks**: scalar q̇q̇ `[0,0] == sa2dt` (tol 1e-12); max abs diff between mass and scalar q̇q̇ blocks `> 1e-3·sa2dt`.

---

### JointLevelKFTransitionNoiseTest.java
- **Purpose**: Locks in transition matrix F, the discrete process noise Q via CWNA/Van-Loan closed form (diagonal `σ_acc² I` FALLBACK path, no robot model), and the encoder measurement noise. Ported from `tests/jointKF/test_noise.py`.
- **Shared setup**: Constants: `DT = 1e-3`, `SIGMA_ACCEL = 50.0`, `ENCODER_VAR = 5.0e-5` (scalar fallback), `IMU_BIAS_VAR = JointLevelKFTestFixture.IMU_BIAS_PROCESS_VAR`.

#### testBuildFStructureAndExactness
- **Scenario**: For each `shapes(1000L)`.
- **Checks**: F has `dim` rows; `block(F,0,0,n,n)=I`, `block(F,n,n,n,n)=I`, `block(F,2n,2n,3m,3m)=I` (bias) (tol 1e-12); `block(F,0,n,n,n)=dt·I` (q→q̇); `block(F,n,0)=0`, bias↔joint couplings zero (tol 1e-12).

#### testBuildFEqualsIPlusADt
- **Scenario**: `singlePair(1001L, 6, 1, 4)` (n=3). Build expected `= I(dim)` with `expected[i, n+i]=DT`.
- **Checks**: `assertAllClose(F, expected, 1e-12)` — `F = I + A·dt`.

#### testProcessNoiseVanLoanJointBlocks
- **Scenario**: `sa2 = 50²=2500`; `posFactor=dt³/3`, `crossFactor=dt²/2`, `velFactor=dt`. For each `shapes(1100L)`.
- **Checks**: `block(Q,0,0)≈posFactor·sa2·I`, `block(Q,0,n)≈crossFactor·sa2·I`, `block(Q,n,0)≈crossFactor·sa2·I`, `block(Q,n,n)≈velFactor·sa2·I` (tol 1e-9); `Qa[0,0]>0`.

#### testProcessNoiseBiasBlockAndNoCrossCoupling
- **Scenario**: For each `shapes(1200L)`, `mDof=3m`.
- **Checks**: `block(Q,2n,2n,mDof,mDof) ≈ scaledIdentity(mDof, DT·IMU_BIAS_VAR)` (tol 1e-12); joint↔bias cross blocks zero.

#### testProcessNoiseSymmetricPSD
- **Scenario**: For each `shapes(1300L)`.
- **Checks**: `assertSymmetric(Q, 1e-12)`, `assertPositiveSemiDefinite(Q)`.

#### testEncoderMeasurementModel
- **Scenario**: For each `shapes(1400L)`. `Henc = getEncoderJacobian()`, `Renc = getEncoderNoise()`.
- **Checks**: `Henc` is `n×dim`; `block(Henc,0,0,n,n)=I`; `block(Henc,0,n,n,dim-n)=0` (observes only position); `Renc ≈ scaledIdentity(n, ENCODER_VAR=5e-5)` (tol 1e-12).

---

### JointLevelKFBiasObservabilityTest.java
- **Purpose**: Tests observability of the base-IMU gyro bias (root cause of pelvis pitch drift). The stacked gyro measurement `z = J_stack(q)·q̇ + L(q)·b + noise`; the IMU-pair rows see bias only as rotated differences, so its nullspace is the 3-D common-mode gauge `N = {δb : δb_i = R(i←W)·β}`. The stance-anchor row's `+I₃` on base bias is the only absolute bias observation. Also tests the Alex ankle regression (anchor with unfiltered chain joints).
- **Shared setup**: `SEED = 20260712L`. Fixtures via `shapes(SEED)`, `singlePairFootBeyondIMUs(SEED, 10, 1, 5, 9)`, `singlePair(SEED, 10, 1, 9)`. Uses filter test hooks: `setTrustedFeetForTest`, `buildStackedMeasurementForTest`, `getStackedMeasurementJacobian`, `getStackedMeasurementNoise`, `getActiveAnchorCountForTest`, `getBiasBlockColumn`. Reference helper `gaugeDirection(f, βWorld)`: builds δx=(0,0,δb) where for each IMU `δb_i = R(imu←world)·β` (via `worldToImu.inverseTransform`), placed at `getBiasBlockColumn(imu)`.

#### testCommonModeBiasIsUnobservableWithoutAnchors
- **Scenario**: For each `shapes(SEED)`: `setTrustedFeetForTest(empty)` (no foot ⇒ pairs only), `buildStackedMeasurementForTest`. `gauge = gaugeDirection(f, (0.013,-0.007,0.021))`. `hDelta = H·gauge`.
- **Checks**: `normF(hDelta) == 0.0` within 1e-9 (common-mode bias is nullspace direction of H).

#### testStanceAnchorFixesTheGauge
- **Scenario**: For each `shapes(SEED)`: `β=(0.013,-0.007,0.021)`, `gauge=gaugeDirection`; `setTrustedFeetForTest(all feet)` (anchor active), `buildStackedMeasurementForTest`. `hDelta = H·gauge`.
- **Checks**: `normF(hDelta) == β.norm()` within 1e-9 (anchor's +I₃ reads back exactly β).

#### testAnchorIsUsableWhenChainHasUnfilteredJoints
- **Scenario**: `singlePairFootBeyondIMUs(SEED, 10, 1, 5, 9)` — 10 joints, IMUs after joints 1 and 5 (joints 2..5 filtered), foot after joint 9 (joints 6..9 on base→foot chain but NOT states; Alex ankle case). Trust feet, build stacked measurement.
- **Checks**: `getActiveAnchorCountForTest() == 1` (anchor active despite unfiltered ankles). `β=(0.011,0.004,-0.017)`, `normF(H·gauge) == β.norm()` within 1e-9. Then untrust feet, rebuild: `normF(H0·gauge) == 0.0` within 1e-9 (gauge reopens — anchor-fixed not topology-fixed).

#### testAnchorCovarianceIncludesUnfilteredJointNoise
- **Scenario**: `singlePairFootBeyondIMUs(SEED, 10, 1, 5, 9)`, trust feet, `R = getStackedMeasurementNoise()`. Anchor block = trailing 3×3 (rows `[R.rows-3 .. R.rows-1]`). `anchorTrace` = sum of its 3 diagonals. Compare against all-filtered `singlePair(SEED, 10, 1, 9)`.
- **Checks**: `anchorTrace > 1.0e-2` (must include `J_U·diag(σ_qd²)·J_Uᵀ` congruence for 4 unfiltered ankle velocities at σ_qd=0.1 rad/s; Σ_ε alone = 3·ANCHOR_VAR = 1.2e-3). `cleanTrace < anchorTrace` (all-filtered anchor is tighter).
- **Port notes**: `R_anchor = Σ_ε + J_U·diag(σ_qd²)·J_Uᵀ`. Requires unfiltered-joint velocity-noise congruence in port.

---

### JointLevelKFEncoderNISConsistencyTest.java
- **Purpose**: Per-joint encoder measurement-noise wiring and chi-square NIS consistency. Property: with prior `(x̂, P)` (P diagonal) and encoder `z_i = q_true,i + e_i`, truth sampled from prior `q_true,i = x̂_i + ε_i, ε_i~N(0,P_ii)`, noise `e_i~N(0,R_ii)`, innovation `ν_i = ε_i + e_i ~ N(0, S_ii)`, per-joint `NIS_i = ν_i²/S_ii ~ χ²₁` (mean 1, var 2).
- **Shared setup**: `ENCODER_VAR_FALLBACK = 5.0e-5`, `SEED = 31_001L`, `NUM_CHAIN_JOINTS = 10` (→ n=8). `sigmaFor(name) = 1e-4·(1 + floorMod(name.hashCode(), 9))` → STD in [1e-4, 9e-4]. Fixtures via `singlePairWithEncoderNoise(SEED, 10, 1, 9, sigmaFor)`. Uses `getEncoderNoise`, `getEncoderJacobian`, `josephUpdate(H,z,R,"encoder")`, YoVariable lookups via `registry.findVariable`.
  - Reference helper `runNISTrials(noiseScale, trials)`: builds fixture; prior `x=0`, diagonal P with `P[i,i]=0.25·R[i,i]` (i<n), `1e-2` (velocities), `1e-6` (bias); `Random(SEED+7)`. Each trial: `setStateForTest(xPrior,pPrior)`; for each joint sample `truth = xPrior_i + sqrt(P_ii)·gaussian`, `noise = noiseScale·sqrt(R_ii)·gaussian`, `z_i = truth+noise`; `josephUpdate(Henc,z,Renc,"encoder")`; read YoDouble `jointKF_encNIS_<name>`. Returns per-joint mean.

#### perJointEncoderVarianceIsWiredIntoREncAndPublished
- **Scenario**: `singlePairWithEncoderNoise(SEED, 10, 1, 9, sigmaFor)`. `Renc = getEncoderNoise()`.
- **Checks**: `Renc` has n rows; `Renc[i,i] == sigmaFor(name)²` within 1e-15; off-diagonals exactly 0.0; YoDouble `jointKF_encR_<name>` published and `== expectedVar` within 1e-15.

#### unmatchedJointFallsBackToScalarEncoderVar
- **Scenario**: Probe fixture to get name of joint 0; rebuild with `name -> equals(unmatched)? NaN : sigmaFor(name)`.
- **Checks**: `Renc[i,i] == ENCODER_VAR_FALLBACK (5e-5)` for the unmatched joint, else `sigmaFor²`, within 1e-15.

#### encoderNISIsChiSquareConsistentAtTheWiredNoise
- **Scenario**: `runNISTrials(1.0, 4000)`. `envelope = 4·sqrt(2/4000) ≈ 0.089`.
- **Checks**: each `|meanNIS[i] - 1.0| < envelope` (4-sigma on χ²₁ sample mean).

#### encoderNISCatchesUnderstatedR
- **Scenario**: `runNISTrials(3.0, 4000)` (noise at 3× wired σ, 9× variance; E[NIS]=(P+9R)/(P+R)=7.4 for P=R/4).
- **Checks**: each `meanNIS[i] > 4.0`.
- **Also inside runNISTrials**: per-trial asserts NIS finite and ≥0; on trial 0 asserts signed innovation `jointKF_encInnov_<name> == z_i - xPrior_i` within 1e-15.
- **Port notes**: Fixed Java RNG (`Random(SEED+7)`, `nextGaussian()`) — only the sample mean over 4000 trials is asserted, so exact draw-matching is not required. YoVariable NIS/innovation publishing must exist in the port's diagnostic layer. `String.hashCode()` for `sigmaFor` — replicate Java hashCode or substitute an equivalent deterministic per-joint map.

---

### JointLevelKFDirectVelocityMeasurementTest.java
- **Purpose**: Direct joint-velocity measurement channel: wiring, adaptive lag inflation, chi-square NIS consistency. Lag-inflation property: measured q̇ is output of first-order low-pass with corner ω_eff; `R_ii(t) = σ_i² + (d̂_i/ω_eff,i)²`, d̂ = 5 Hz-smoothed finite difference of the measurement.
- **Shared setup**: `SIGMA_QD_FALLBACK = 0.1` rad/s, `DT = 1e-3`, `SEED = 47_001L`, `NUM_CHAIN_JOINTS = 10` (n=8). `posSigmaFor(name)=1e-4·(1+floorMod(hashCode,9))`; `velSigmaFor(name)=5e-3·(1+floorMod(hashCode,7))` → STD in [5e-3, 3.5e-2]. Fixtures via `singlePairWithDirectVelocity(SEED, 10, 1, 9, posSigmaFor, velSigmaFor, cornerFn)`. Uses `getVelocityMeasurementJacobian`, `getVelocityMeasurementNoise`, `setDirectVelocityMeasurementForTest`, `refreshDirectVelocityNoise`, `josephUpdate(H,z,R,"encoderVelocity")`.
  - Reference helper `runNISTrials(noiseScale, trials)`: cornerFn=null (static R). Prior `x=0`, `P[i,i]=1e-4` (positions), `P[n+i,n+i]=0.25·R[i,i]` (velocities), `1e-6` (bias). `Random(SEED+13)`. Each trial: sample `truth = xPrior[n+i] + sqrt(P[n+i,n+i])·gaussian`, `noise = noiseScale·sqrt(R[i,i])·gaussian`; `josephUpdate(Hqd,z,Rqd,"encoderVelocity")`; read `jointKF_qdNIS_<name>`.

#### velocityMeasurementModelIsWired
- **Scenario**: cornerFn=null. `Hqd = getVelocityMeasurementJacobian()` (n×dim), `Rqd = getVelocityMeasurementNoise()`.
- **Checks**: `Hqd[i,j] == (j==n+i ? 1 : 0)` (exactly `[0 | I_n | 0]`); `Rqd[i,i] == velSigmaFor(name)²` within 1e-15; YoDouble `jointKF_qdR_<name>` published `== expectedVar`.

#### unmatchedJointFallsBackToSigmaQdUnfiltered
- **Scenario**: Probe joint 0's name; rebuild velSigma with `name -> equals(unmatched)? NaN : velSigmaFor`.
- **Checks**: `Rqd[i,i] == SIGMA_QD_FALLBACK² (0.01)` for unmatched, else `velSigmaFor²`, within 1e-15.

#### lagInflationTracksMeasuredSlewExactly
- **Scenario**: `cornerHz=10.0`, `invOmega=1/(2π·10)`, cornerFn returns 10.0. Phase 1: 100 ticks of constant z=0 (`setDirectVelocityMeasurementForTest`+`refreshDirectVelocityNoise`). Phase 2: 300 ticks of ramp `z[i]=slope·t·DT`, `slope=2.0` rad/s per s. Phase 3: 2000 ticks with z frozen at last ramp value.
- **Checks**: Phase 1 `Rquiet[i,i] == velSigmaFor²` within 1e-12 (floor). Phase 2 `Rswing[i,i] == velSigmaFor² + (slope·invOmega)²` within `1e-3·expected`; published `jointKF_qdR_<name> == Rswing[i,i]` exactly (tol 0.0). Phase 3 `Rsettled[i,i] < velSigmaFor²·1.01` (inflation decays, no latch-up).
- **Port notes**: smoother α = exp(−2π·5·dt); 300 ticks leaves relative error <1e-4. Deterministic — highly portable.

#### directVelocityNISIsChiSquareConsistentAtTheWiredNoise
- **Scenario**: `runNISTrials(1.0, 4000)`, `envelope = 4·sqrt(2/4000)`.
- **Checks**: each `|meanNIS[i] - 1.0| < envelope`.

#### directVelocityNISCatchesUnderstatedR
- **Scenario**: `runNISTrials(3.0, 4000)`.
- **Checks**: each `meanNIS[i] > 4.0`.
- **Also inside runNISTrials**: per-trial NIS finite and ≥0; on trial 0 signed innovation `jointKF_qdInnov_<name> == z_i - xPrior_{n+i}` within 1e-15; cross-talk guard: after "encoderVelocity" updates, `jointKF_encNIS_<name>` stays NaN (velocity channel must not publish into position NIS).
- **Port notes**: dispatch keyed on measurement label ("encoder" vs "encoderVelocity") — exact-match dispatch required.

---

### JointLevelKFStackedReferenceTest.java
- **Purpose**: THE decisive reference for the Rev.2 stacked gyro measurement (SPEC §9): the stacked Joseph update must equal a reference KF that measures RAW per-IMU gyros with block-diagonal (independent) noise plus a near-zero absolute-rate constraint per trusted foot, over a state augmented with a nuisance base angular velocity ω_base, then marginalizes ω_base out (information form, improper ω_base prior = γ→∞ limit).
- **Shared setup**: `ANCHOR_VAR = 4.0e-4` (must match filter's `ANCHOR_VAR`/Σ_ε). Fixtures via `twoPairs(seed, 10, 1, 5, 9)` and `singlePairMassMatrix(seed, 8, 1, 6)`. Uses `spd(dim, seed)`, filter hooks `setStateForTest`, `setTrustedFeetForTest`, `buildStackedMeasurementForTest`, `getStackedMeasurementJacobian/Residual/Noise`, `josephUpdate(H,z,R)`, `getBaseIMU`, `getBiasBlockColumn`, `getJointStateIndex`, `JointLevelKFPreFilter.set_matrix`.
  - **Reference `referenceMarginalized(f, muX, Pxx, activeFeet)`** → `{muXPost, PxxPost}`: augments state `x=[q;q̇;b]` with nuisance ω_base (D=dim+3). Builds `H (M×D)`, `z`, `R` where `M = 3·numIMUs + 3·numActiveFeet`. For each IMU: `z = raw gyro`; ω_base columns = `R(base measurement frame → IMU measurement frame)` (via `getTransformToDesiredFrame` + `set_matrix`); bias columns = `+I₃` at `getBiasBlockColumn(imu)`; q̇ columns = absolute angular Jacobian base→IMU link expressed in IMU frame (0 for base IMU) via `GeometricJacobianCalculator` (setKinematicChain(baseLink, imuLink), setJacobianFrame(imuFrame)); noise block = `imu.getAngularVelocityNoiseCovariance` (independent Σ_i). For each active foot: q̇ columns = `J_leg` base→foot in base frame; `+I₃` on ω_base; `z=0`; `R = ANCHOR_VAR·I₃`. Then information-form: `Λ = blkdiag(Pxx⁻¹, 0₃) + Hᵀ R⁻¹ H`, `η = [Pxx⁻¹·muX; 0] + Hᵀ R⁻¹ z`, `Σ = Λ⁻¹`, `mu = Σ·η`; return x-block of mu and Σ.

#### testStackedUpdateMatchesNuisanceMarginalizedReference
- **Scenario**: `Random(90000L)`, 12 trials. Each: `twoPairs(90100L+trial, 10, 1, 5, 9)` (two pairs sharing middle IMU, chain a-b-c, base=a; one stance foot on far link). Random raw gyros per IMU `0.4·(rand-0.5)`. `muX[i] = 0.05·(i+1) - 0.1`, `Pxx = spd(dim, 700L+trial)`. All feet trusted. Compute reference; run filter stacked update from same prior.
- **Checks**: `assertAllClose(filter.x⁺, ref[0], 1e-5)`, `assertAllClose(filter.P⁺, ref[1], 1e-5)`.

#### testPairsOnlyMatchesReference
- **Scenario**: `Random(91000L)`, 8 trials. Each: `singlePairMassMatrix(91100L+trial, 8, 1, 6)`. Random gyros `0.3·(rand-0.5)`. `muX[i]=0.02·(i+1)`, `Pxx=spd(dim, 800L+trial)`. No trusted feet (K=0, pairs-only; bias common mode unobservable but improper prior handles it).
- **Checks**: `assertAllClose(x⁺, ref[0], 1e-5)`, `assertAllClose(P⁺, ref[1], 1e-5)`.
- **Port notes**: Depends on Mecano `GeometricJacobianCalculator`, `getTransformToDesiredFrame`, `set_matrix`. Python port needs its own independent forward-kinematics/Jacobian reference. `spd(dim, seed)` deterministic SPD generator.

---

### JointLevelKFRotorAndGramTest.java
- **Purpose**: Machine-precision reference for the two algebraic halves of the Part B process-noise fix on a well-conditioned synthetic Λ (item 1: reflected rotor inertia + Weyl floor; item 3/6: per-joint σ_τ Gram form). Mirrors `updateProcessNoiseFromMassMatrix`'s exact arithmetic. Fully self-contained (hand-built matrices, no fixture except the static `reflectedRotorInertiaForNameOrDefault` lookup).
- **Shared setup**: helper `minEig(a)` via EJML `EigenDecomposition_F64`.

#### testReflectedRotorInertiaAddAndWeylFloor
- **Scenario**: `Λ = [[1.20,0.10,0.02],[0.10,0.03,0.01],[0.02,0.01,0.90]]` (light mode at index 1), `rotor=[0.062,0.167,0.070]`. `Lambda_eff = Λ + diag(rotor)`.
- **Checks**: diagonal add exact (`Λ[i,i]+rotor[i]`, tol 0.0); off-diagonals unchanged (tol 0.0); Weyl: `λ_min(Lambda_eff) >= λ_min(Λ) + min(rotor) - 1e-12`; `λ_min(Lambda_eff) >= min(rotor) - 1e-12`.

#### testGramFormQaEqualsDenseReferenceAndIsSymmetricPSD
- **Scenario**: `Lambda_eff = [[1.262,0.10,0.02],[0.10,0.197,0.01],[0.02,0.01,0.970]]`, `sigmaTau = [0.15·160.7, 0.15·217.2, 0.15·193.6]`. Filter path: `Y[i,j] = Lambda_eff⁻¹[i,j]·σ_τ[j]`, `Qa_gram = Y Yᵀ` (`multTransB`). Dense ref: `Lambda_eff⁻¹ diag(σ²) Lambda_eff⁻ᵀ`.
- **Checks**: `Qa_gram[i,j] == Qa_ref[i,j]` within `1e-10·max(1,maxAbs(Qa_ref))`; exact symmetry `Qa_gram[i,j]==Qa_gram[j,i]` (tol 0.0); PSD `minEig(Qa_gram) >= -1e-12`.

#### testRotorInertiaTableLookup
- **Scenario**: Direct calls to `JointKFParameters.reflectedRotorInertiaForNameOrDefault(name)`.
- **Checks** (all tol 0.0): `LEFT_HIP_X`→0.062; `RIGHT_HIP_Y`→0.167; `left_knee_y`→0.167 (case-insensitive, KNEE); `LEFT_ANKLE_Y`→0.070; `LEFT_ANKLE_X`→0.050; `SPINE_Z`→0.062; `SOME_UNKNOWN_JOINT`→0.005 (default floor).
- **Port notes**: Rotor-inertia name→value table is a filter constant the port must replicate exactly (substring, case-insensitive matching).

---

### JointLevelKFSingularInnovationDiagnosticTest.java
- **Purpose**: Locks in the near-singular innovation-covariance diagnostic (`describeSingularInnovation`): when S = HPHᵀ + R goes near-singular and the update is skipped, the diagnostic message must name the physical measurement (IMU pair / anchor / encoder joint) carrying the degenerate direction.
- **Shared setup**: None shared; each test builds its own fixture. Uses `setStateForTest`, `identity`, `describeSingularInnovation(label, reason, H, R)`, `getBaseIMU`.

#### testDiagnosticNamesTheDegenerateGyroPair
- **Scenario**: `singlePair(1234L, 10, 1, 9)` (n=8, m=2, one pair). Set `x=0`, `P=identity(dim)`. Build `H (3×dim)`: rows 0 and 1 identical (`sin(0.31(c+1))`), row 2 = `cos(0.17(c+2))`; `R = 1e-6·I(3)`. Call `describeSingularInnovation("stackedGyroUpdate", "test-induced rank deficiency", H, R)`.
- **Checks**: message contains "near-singular", "cond(S)", "gyro pair 0", and the base-IMU sensor name (`getBaseIMU().getSensorName()`).

#### testDiagnosticNamesTheDegenerateEncoderJoint
- **Scenario**: `singlePair(4321L, 6, 1, 5)` (n=4). `x=0`, `P=identity(dim)`. `H (2×dim)`: rows 0 and 1 both set column 0 = 1.0 (both observe joint state index 0); `R = diag(1e-6,1e-6)`. Call `describeSingularInnovation("encoder", ...)`.
- **Checks**: message contains "near-singular" and `"encoder q of joint " + filteredJoints[0].getName()`.
- **Port notes**: Diagnostic-string content is implementation-specific; port must produce equivalent human-readable attribution. Tests the mapping from near-null eigenvector of S back to physical measurement rows.

---

### JointLevelKFPreFilterAllocationTest.java
- **Purpose**: Guards the `JointLevelKFPreFilter` hot path against per-tick heap allocation on the real-time estimator thread (root cause of hardware EtherCAT errors: `CommonOps_DDRM.invert` newing an LU solver when n>5, and a redundant `updateFrame()`). **JVM-specific — the two allocation tests are NOT portable to Python** (rely on `com.sun.management.ThreadMXBean` allocation counters). `testHotPathStaysFinite` is the only behaviorally portable one.
- **Shared setup**: Constants: `DT = 1e-3`, `MAX_BYTES_PER_TICK = 32.0`, `WARMUP_TICKS = 20_000`, `MEASURED_TICKS = 60_000`. This test file defines its OWN self-contained fixtures (not `JointLevelKFTestFixture`):
  - **`Fixture(Random, useMassMatrixProcessNoise)`**: 10-joint `RandomFloatingRevoluteJointChain` with `jointAxes[i]` cycling X/Y/Z by `i%3`; `nextState(CONFIGURATION, VELOCITY)`, `updateFramesRecursively()`. IMUs `parentIMU` on `joints.get(1).getSuccessor()`, `childIMU` on `joints.get(9).getSuccessor()` (chain spans 8 joints → n=8>5). One pair `("testPair", true, "parentIMU", "childIMU", 0.0, 0.0)`. `feet = [childLink]`. Filter built with elevator (mass-matrix path) or without (scalar path).
  - **`TestIMU`**: fixed angular velocity `(0.01,-0.02,0.03)`, linear accel `(0,0,9.81)`, identity orientation, all noise covariances `= diag(1e-4)`.
  - **`TestSensorMap`**: returns the IMUs and cached `dummyOneDoFJointState` per joint (no per-lookup allocation).

#### testPhase1AndPhase2HotPathIsAllocationFree
- **Scenario**: `assumeTrue(threadMXBean.isThreadAllocatedMemorySupported())`. `Fixture(Random(9001L), false)` (scalar path). `initialize()`, 20,000 warmup ticks (each `computeJointState()`+`computeImuBiases(feet)`), then measure allocated bytes over 60,000 ticks.
- **Checks**: `bytesPerTick < 32.0`.
- **Port notes**: NOT portable — JVM allocation counter. Skip in Python suite.

#### testSchurProcessNoiseHotPathIsAllocationFree
- **Scenario**: `Fixture(Random(9003L), true)` (mass-matrix/Schur path active). Assert `isUsingMassMatrixProcessNoise()`. Same warmup/measure loop.
- **Checks**: `bytesPerTick < 32.0` (per-tick Schur rebuild reuses pre-warmed solvers/scratch).
- **Port notes**: NOT portable — JVM-specific.

#### testHotPathStaysFinite
- **Scenario**: `Fixture(Random(9002L), false)`. `initialize()`, 1000 ticks.
- **Checks**: each filtered joint's `getEstimatedJointPosition` and `getEstimatedJointVelocity` finite; base IMU bias `getAngularVelocityBiasInIMUFrame(baseIMU)` X/Y/Z finite.
- **Port notes**: This one IS portable (finiteness/sanity smoke test).

---

#### Cross-cutting port notes (jointLevel behavior tests)
- **Filter constants the tests mirror** (must match the Python estimator): `SIGMA_ACCEL=50.0`, `SIGMA_TAU=5.0`, `QA_MAX=900.0`, `ENCODER_VAR=5.0e-5`, `SIGMA_QD_UNFILTERED=0.1`, `ANCHOR_VAR=4.0e-4`, `ROTOR_INERTIA_DEFAULT=0.005`, `TARGET_QDD_STD=20.0`, rotor-inertia table (HIP_X 0.062, HIP_Y/KNEE 0.167, ANKLE_Y 0.070, ANKLE_X 0.050, SPINE 0.062), `ALPHA` fraction 0.15, direct-velocity smoother 5 Hz, IMU-noise `diag(1e-4)`.
- **EJML/Mecano dependencies** to replace with NumPy/SciPy + an independent kinematics reference: `CompositeRigidBodyMassMatrixCalculator`, `GeometricJacobianCalculator`, `RigidBodyTransform`, `RotationMatrix`, LU/Cholesky invert. Note the deliberate LU-vs-Cholesky tolerance loosening (`relTol = 3e-3·maxAbs` in MassMatrixNoise, `1e-4` in StandingStability) — a NumPy port using a single inversion path can likely tighten these.
- **Determinism/RNG**: NIS tests use fixed `java.util.Random(SEED+7)` / `(SEED+13)` and `nextGaussian()`; only sample means over 4000 trials are asserted (envelope 0.089), so exact draw-matching is not required, but the pass margins assume ~4000 independent standard-normal draws. `sigmaFor`/`velSigmaFor`/`posSigmaFor` use Java `String.hashCode()` — replicate Java hashCode or substitute an equivalent deterministic map.
- **Not portable**: `JointLevelKFPreFilterAllocationTest`'s two allocation tests (JVM `ThreadMXBean`); keep only `testHotPathStaysFinite` as a smoke test. Bit-exact (`tol=0.0`) assertions (`testDeterministic`, exact-symmetry, exact innovation) depend on identical floating-point operation ordering — in Python assert to machine epsilon rather than literal bit-equality where operation order differs.

---

## invariant_estimator core tests (Lie group, state, propagator, updater, EKF, reseed)

### SEK3UtilsTest.java

- **Purpose**: Tests `SEK3Utils`, the SE_k(3) matrix-Lie-group exp/log/adjoint utilities: exp/log round-trip for k=1,2,3, agreement with trusted SE(3) tooling at k=1, and adjoint algebraic identities (homomorphism, conjugation).
- **Shared setup**: No `@BeforeEach`. Constants: `EPSILON = 1.0e-10`, `ITERATIONS = 1000`. Each test builds its own `Random` with a fixed seed. The algebra vector layout is ξ = [φ (3, rotation part); then k blocks of 3 (translation/velocity/contact parts)], length `3 + 3k`. The group element X is an `(3+k)×(3+k)` matrix (top-left 3×3 rotation R; k columns of length 3 to the right of it; bottom rows form the SE_k(3) structured identity). Adjoint size is `(3+3k)×(3+3k)`. Ordering convention is [φ; ρ] (rotation-first).

#### testExpLogRoundTrip
- **Scenario**: seed `1234L`. Loop k = 1..3. For each k, allocate ξ length `3+3k`, X of size `(3+k)×(3+k)`. Run 1000 iterations; each iteration fills ξ via `randomAlgebraVector`, computes `SEK3_Utils.exp(xi, X)` then `SEK3_Utils.log(X, recovered)`.
- **Checks**: for every component j, `assertEquals(xi[j], recovered[j], 1.0e-10)`. Verifies log∘exp = identity on the algebra (exp/log round-trip) across all three group dimensions.
- **Port notes**: 1000 trials × 3 k-values with one shared RNG stream seeded 1234 (order matters if reproducing exact draws, but tolerance-based so any valid RNG works if the round-trip is exact). φ drawn as a bounded rotation vector (‖φ‖ ≤ π) — see helper.

#### testExpMatchesSE3ForK1
- **Scenario**: seed `5768L`. ξ length 6, X is 4×4, plus a Euclid `RigidBodyTransform`. 1000 iterations, k=1 random ξ.
- **Checks**: `SEK3_Utils.exp(xi, X)` vs `SE3LieGroupTools.exp(xi, transform)`. Compare top-left 3×3 of X to `transform.getRotation().getElement(r,c)` (all 9, tol 1e-10); compare X(0,3),X(1,3),X(2,3) to transform translation x,y,z (tol 1e-10). Verifies the k=1 SE_k(3) exp matches the trusted SE(3) exponential block-for-block (rotation block and translation column 3).
- **Port notes**: Python port needs its own reference SE(3) exp (e.g. Sophus/manif or a hand-rolled Rodrigues + left-Jacobian V matrix) to stand in for `SE3LieGroupTools.exp`. Translation is `V(φ)·v` where V is the SO(3) left Jacobian.

#### testLogRejectsWrongSizedOutput
- **Scenario**: X = 5×5 (that is k=2, expecting ξ length 9). Pass a length-6 output array.
- **Checks**: `assertThrows(IllegalArgumentException.class, () -> SEK3_Utils.log(X, wrongLength))`. Validates size-consistency guard between matrix dimension `n = 3+k` and output length `3+3k`.
- **Port notes**: Python should raise `ValueError` when output length ≠ `3 + 3*(n-3)`.

#### testAdjointMatchesSE3ForK1
- **Scenario**: seed `7777L`. ξ length 6, X 4×4, `adj` and `adjSE3` both 6×6. 1000 iterations.
- **Checks**: `SEK3_Utils.exp(xi,X)` then `SEK3_Utils.adjoint(X,adj)`; compare against `SE3LieGroupTools.adjoint(transform, adjSE3)` where transform = SE3 exp of same ξ. All 36 entries `assertEquals(adjSE3(r,c), adj(r,c), 1e-10)`. Verifies the 6×6 adjoint at k=1 equals the trusted SE(3) adjoint with the same [φ; ρ] ordering.
- **Port notes**: adjoint block form for SE(3): [[R, 0],[hat(t)·R, R]] under this ordering (rotation-first). Confirm ordering when porting.

#### testAdjointHomomorphism
- **Scenario**: seed `8888L`. Loop k=1..3, `n=3+k`, `m=3+3k`. 1000 iterations each: draw ξ_A, ξ_B via `randomAlgebraVector`; `X_A=exp(ξ_A)`, `X_B=exp(ξ_B)`; `X_AB = X_A·X_B` (`CommonOps_DDRM.mult`); form adj_A, adj_B, adj_AB, and adjProduct = adj_A·adj_B.
- **Checks**: for all m×m entries `assertEquals(adjProduct(r,c), adjAB(r,c), 1.0e-9)`. Verifies the adjoint homomorphism Ad_{X_A·X_B} = Ad_{X_A}·Ad_{X_B}.
- **Port notes**: tolerance loosened to 1e-9 here (vs 1e-10). Two independent random ξ draws per iteration from same RNG.

#### testAdjointConjugationIdentity
- **Scenario**: seed `9999L`. Loop k=1..3, `n=3+k`, `m=3+3k`. 1000 iterations: draw η and ξ; `X=exp(η)`, `Xinv = inv(X)`; `expXi = exp(ξ)`; `conj = X·expXi·Xinv`; `lhsXi = log(conj)`; `adj = adjoint(X)`; `adXi = adj·ξ` (via local `multiply`).
- **Checks**: for all m components `assertEquals(adXi[j], lhsXi[j], 1.0e-8)`. Verifies the defining adjoint identity log(X·exp(ξ)·X⁻¹) = Ad_X·ξ, tying adjoint to exp/log through conjugation.
- **Port notes**: tolerance 1e-8. Uses matrix inverse of X. Two random draws (η then ξ) per iteration.

**Helper/reference methods**:
- `multiply(DMatrixRMaj A, double[] x, double[] result)`: plain matrix-vector product result = A·x (rows×cols · cols).
- `randomAlgebraVector(Random random, int k, double[] xiToPack)`: fills length `3+3k`. Slots 0..2 = `EuclidCoreRandomTools.nextRotationVector(random)` (a random rotation vector with magnitude bounded by π, so exp/log stays in the injectivity radius). Then k blocks each = `EuclidCoreRandomTools.nextVector3D(random)` (components ~ U(-1,1) by default in Euclid). Port must emulate the bounded-π rotation vector to keep log well-defined; the translation vectors need only be modest-magnitude random.

---

### InvariantStateTest.java

- **Purpose**: Tests the `InvariantState` container: identity construction and dimension formulas, set/get round-trips for each named component, column independence (no aliasing), contact index bounds, tangent-index layout, and `setToIdentity`.
- **Shared setup**: No `@BeforeEach`. Constants `EPSILON = 1.0e-12`, `ITERATIONS = 500`. `InvariantState(numberOfContacts N)` holds a group element X of size `(5+N)×(5+N)` and covariance P of size `(9+3N)×(9+3N)`. Group size n = 5+N, tangent size m = 9+3N.

#### testConstructorIdentityAndSizes
- **Scenario**: `contactCounts = {0,1,2,4}`. Construct a fresh `InvariantState` for each.
- **Checks**: `getNumberOfContacts()==N`; `getGroupSize()==5+N`; `getTangentSize()==9+3N`; `MatrixFeatures_DDRM.isIdentity(getGroupElement(), 1e-12)` true; `MatrixFeatures_DDRM.isZeros(getCovariance(), 1e-12)` true. Verifies fresh state = group identity, covariance = 0, and correct sizes.
- **Port notes**: n = 5+N group matrix; the "5" = 3 (rotation) + 2 structural rows for base velocity + base position columns. Covariance initialized to zeros (not identity).

#### testRotationRoundTrip
- **Scenario**: seed `1234L`, `InvariantState(2)`, 500 iterations. Each: `expected = EuclidCoreRandomTools.nextRotationMatrix(random)`, `setRotation(expected)`, `getRotation(actual)`.
- **Checks**: `EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, 1e-12)`. Rotation block round-trips exactly.
- **Port notes**: rotation stored as top-left 3×3 of X.

#### testBaseVelocityAndPositionRoundTrip
- **Scenario**: seed `2345L`, `InvariantState(2)`, 500 iterations. Each draws random velocity and position via `nextVector3D`; sets both, reads both.
- **Checks**: `assertVector3DGeometricallyEquals` for velocity and position, tol 1e-12.
- **Port notes**: base velocity and base position are the two structured columns of X (columns 3 and 4 for N contacts). Confirm which column holds velocity vs position.

#### testNamedComponentsAreIndependent
- **Scenario**: seed `3456L`, N=3. Set rotation, velocity, position, and 3 distinct random contacts, then read all back.
- **Checks**: rotation (`assertMatrix3DEquals`), velocity, position, and each contact (`assertVector3DGeometricallyEquals`), all tol 1e-12. Verifies no column aliasing — each named slot independent.
- **Port notes**: contact i occupies column 5+i of X.

#### testContactIndexOutOfBounds
- **Scenario**: `InvariantState(2)` (valid indices 0,1).
- **Checks**: `assertThrows(IndexOutOfBoundsException)` for `getContactPosition(-1,out)`, `getContactPosition(2,out)`, `setContactPosition(2,in)`, and `contactTangentIndex(2)`.
- **Port notes**: Python should raise `IndexError` for out-of-range contact index on get/set/tangentIndex.

#### testTangentIndices
- **Scenario**: `InvariantState(3)`.
- **Checks**: `rotationTangentIndex()==0`, `baseVelocityTangentIndex()==3`, `basePositionTangentIndex()==6`, `contactTangentIndex(0)==9`, `(1)==12`, `(2)==15`. Verifies tangent block layout: rotation 0, velocity 3, position 6, contact i at 9+3i.
- **Port notes**: This ordering (rotation, velocity, position, contacts) is load-bearing for all Jacobian/covariance indexing in the port.

#### testSetToIdentity
- **Scenario**: seed `4567L`, `InvariantState(2)`. Mutate rotation, velocity, position, contacts 0 and 1 randomly, then call `setToIdentity()`.
- **Checks**: `MatrixFeatures_DDRM.isIdentity(getGroupElement(), 1e-12)`. Verifies reset restores X to identity (does not assert anything about P).
- **Port notes**: `setToIdentity` only touches X, not covariance.

---

### InvariantPropagatorTest.java

- **Purpose**: Tests `InvariantPropagator.predict`. Mean/kinematic behavior validated against exact closed-form integrals (free fall, gravity-compensated rest, rotation composition, static contacts); covariance validated structurally (symmetry, growth from zero, zero-noise invariance) and via the exact log-linear error-propagation property of the invariant EKF.
- **Shared setup**: No `@BeforeEach`. Constants `EPSILON = 1.0e-10`, `GRAVITY = -9.81` (z-component of gravity; matches the propagator's internal gravity). `InvariantPropagator(numberOfContacts, gyroNoise, accelNoise, contactNoise)` constructor. `predict(state, omega, accel, dt)` mutates the state's X and P in place. `accel` is the IMU specific force (proper acceleration); gravity is added internally. Note sign convention: at rest, specific force accel = -g = (0,0,+9.81).

#### testFreeFall
- **Scenario**: dt = 1e-3, 1000 steps. `InvariantPropagator(0, 0.0, 0.0, 0.0)` (no contacts, no noise), `InvariantState(0)`. omega = (0,0,0), accel = (0,0,0). Elapsed T = 1.0 s.
- **Checks**: `velocity.x==0` and `.y==0` (tol 1e-10); `velocity.z == GRAVITY*T` (tol 1e-9); `position.z == 0.5*GRAVITY*T²` (tol 1e-9); rotation == identity (`assertMatrix3DEquals(new RotationMatrix(), rotation, 1e-10)`). Verifies exact constant-acceleration integration: v_z=gT, p_z=½gT².
- **Port notes**: the constant-accel integrator is claimed exact, so the accumulation of 1000 discrete steps must equal the closed form to ~1e-9. Python integrator must match (typical mid-point/exponential integrator for SE_k(3)).

#### testStationaryWithGravityCompensation
- **Scenario**: dt=1e-3, 1000 steps. `InvariantPropagator(0,0,0,0)`, `InvariantState(0)`. omega=(0,0,0), accel=(0,0,-GRAVITY)=(0,0,+9.81).
- **Checks**: final velocity ≈ 0 and position ≈ 0, both `assertVector3DGeometricallyEquals(new Vector3D(), ·, 1e-9)`. Verifies specific-force gravity compensation keeps the base at rest.
- **Port notes**: confirms internal gravity is (0,0,-9.81) and is added to accel.

#### testConstantAngularVelocityComposesRotation
- **Scenario**: dt=1e-3, 500 steps. `InvariantPropagator(0,0,0,0)`, `InvariantState(0)`. omega=(0.3,-0.2,0.5), accel=(0,0,0). T=0.5 s.
- **Checks**: `expected = SO3LieGroupTools.exp(T·omega)`; compare to state rotation, `assertMatrix3DEquals(expected, actual, 1e-9)`. Verifies equal-axis increment composition R_N = exp(ω·T).
- **Port notes**: needs an SO(3) exp reference. Because axis is constant, discrete increments exp(ω·dt) compose exactly to exp(ω·T).

#### testContactsRemainStatic
- **Scenario**: dt=1e-2, one predict. `InvariantPropagator(2, 1.0e-4, 1.0e-2, 1.0e-6)` (nonzero noise), `InvariantState(2)`. contact0=(1,2,3), contact1=(-1,0,2). omega=(0.1,0.2,-0.1), accel=(0,0,-GRAVITY).
- **Checks**: both contacts unchanged after predict, `assertVector3DGeometricallyEquals(·, ·, 1e-10)`. Verifies contact columns are world-static under prediction.
- **Port notes**: contacts must be exactly invariant under `predict` regardless of noise.

#### testCovarianceStaysSymmetric
- **Scenario**: seed `12345L`, dt=1e-2. `InvariantPropagator(2, 1.0e-3, 1.0e-2, 1.0e-4)`, `InvariantState(2)`. Random rotation/velocity/position/2 contacts; `CommonOps_DDRM.setIdentity(P)` to start from symmetric PSD P=I. 50 steps, each with random omega, accel from `nextVector3D`.
- **Checks**: after every step `assertSymmetric(P, 1e-9)` (checks P(r,c)==P(c,r) for all upper-triangle pairs). Verifies covariance symmetry preserved across steps.
- **Port notes**: P starts at identity (m=9+3·2=15). Random omega/accel each step from the shared RNG.

#### testCovarianceGrowsFromZero
- **Scenario**: dt=1e-2. `InvariantPropagator(1, 1.0e-3, 1.0e-2, 1.0e-4)`, `InvariantState(1)` (P starts zero). omega=0, accel=(0,0,-GRAVITY). One predict.
- **Checks**: `assertSymmetric(P, 1e-12)` and `trace(P) > 0`. Verifies P = Q_d after one step from zero: symmetric, positive trace.
- **Port notes**: single-step discrete process noise Q_d has positive trace and is symmetric.

#### testZeroNoiseKeepsCovarianceZero
- **Scenario**: dt=1e-2. `InvariantPropagator(1, 0.0, 0.0, 0.0)`, `InvariantState(1)` (P=0). omega=0, accel=(0,0,-GRAVITY). 20 predicts.
- **Checks**: `MatrixFeatures_DDRM.isZeros(P, 1e-10)`. Verifies with zero process noise and P₀=0, Φ·0·Φᵀ+0 stays exactly zero.
- **Port notes**: covariance update is Φ P Φᵀ + Q_d; with Q_d=0 and P=0 it remains 0.

#### testLogLinearErrorPropagation
- **Scenario**: seed `2024L`. N=2, n=5+2=7, m=9+6=15, dt=1e-3, 200 steps (T=0.2 s). Build a random true SE_k(3) state (random rotation/velocity/position/2 contacts). Build a LARGE initial error ξ₀ (length 15), each entry ~ U(-0.5,0.5) (`0.5*(2*rand-1)`). Form η₀ = exp_G(ξ₀), estimate X̂₀ = exp_G(ξ₀)·X₀. Copy X̂₀ into `estimateState.getGroupElement()`. Propagator `InvariantPropagator(2,0,0,0)` (no noise). omega=(0.4,-0.3,0.6), accel=(0.5,0.2,-GRAVITY). Propagate BOTH true and estimate 200 steps with identical inputs.
- **Checks**: measured final error ξ_N = log_G(X̂_N · X_N⁻¹) (via invert X_N, mult, log). Predicted ξ_N = exp(A·T)·ξ₀ using `buildErrorTransition(T, ...)`. For all 15 components `assertEquals(xiPredicted, xiMeasured, 1.0e-10)`. Verifies the exact log-linear property of the right-invariant error: η = X̂·X⁻¹ evolves exactly linearly (not merely first-order) even for large ξ₀.
- **Port notes**: This is the deepest correctness test. Python needs SE_k(3) exp/log, group inverse, and the exact `buildErrorTransition`. Right-invariant error is X̂·X⁻¹ (left multiplication convention: X̂ = exp(ξ)·X).

**Helper/reference methods**:
- `buildErrorTransition(double time, DMatrixRMaj transition m×m)`: Constructs Φ(T) = exp(A·T) analytically. Start with identity. Let gravity = (0,0,-9.81), `hatGravity = hat(gravity)` (3×3 skew). Then for the 3×3 blocks: block (velocity-rows 3..5, rotation-cols 0..2) = `time · hatGravity`; block (position-rows 6..8, rotation-cols 0..2) = `0.5·time²·hatGravity`; block (position-rows 6..8, velocity-cols 3..5) += `time·I`. All contact blocks stay identity. Math: ξ̇ = A ξ with A = [[0,0,0,...],[hat(g),0,0,...],[0,I,0,...],[0...]], so exp(A·T) = I + A·T + ½A²T² gives exactly these blocks (the ½T² term is A² producing position←rotation = hat(g)·½T²).
- `assertSymmetric(matrix, epsilon)`: for all r, c>r, `assertEquals(M(r,c), M(c,r), epsilon)`.

---

### InvariantUpdaterTest.java

- **Purpose**: Tests `InvariantUpdater.update` with a synthetic single-contact measurement model H = [+I at base-position block, −I at contact block], residual r = R̂·y − (d̂ − p̂). Validates residual reduction (sign/H correctness), covariance shrink+symmetry, and the NIS (normalized innovation squared) against an independent quadratic form and its χ²(3) statistics.
- **Shared setup**: No `@BeforeEach`. Constants `CONTACTS=1`, `GROUP_SIZE=6`, `TANGENT_SIZE=12`, `POSITION_BLOCK=6`, `CONTACT_BLOCK=9`. Updater constructed per-test as `new InvariantUpdater(TANGENT_SIZE=12)`. Measurement dim z=3. Measurement model: y = Rᵀ(d−p) (body-frame forward kinematics). Residual computed in world frame as r = R̂·y − (d̂−p̂). Jacobian H (3×12): +I on columns 6..8 (position), −I on columns 9..11 (contact).

#### testUpdateDrivesResidualToZero
- **Scenario**: seed `101L`. truth = random state; measurement = exact FK from truth. estimate = exp(ξ₀)·truth with ξ₀ = `randomError(random, 1.0e-4)` (each entry `1e-4·(2·rand-1)`). P = identity. Jc = contact Jacobian. R (measurement cov) = `scaledIdentity(3, 1e-10)`. One `updater.update(estimate, Jc, residual, R)`.
- **Checks**: `residualAfter < 1e-6` and `residualAfter < residualBefore`. Verifies one Kalman update with tiny error and tiny measurement noise drives residual to ~0 (confirms correct H sign and residual definition).
- **Port notes**: `update(state, H, residual, R)` is the low-level 4-arg form (state, Jacobian, precomputed residual, measurement covariance). It applies a left-invariant correction to X and updates P.

#### testUpdateReducesResidualForLargerError
- **Scenario**: seed `202L`. ξ₀ = `randomError(random, 0.05)`. P=I. R = `scaledIdentity(3, 1e-8)`. One update.
- **Checks**: `residualAfter < 0.1·residualBefore`. Verifies large residual reduction (≥10×) for moderate error — rules out a sign flip.
- **Port notes**: moderate perturbation 0.05.

#### testCovarianceShrinksAndStaysSymmetric
- **Scenario**: seed `303L`. ξ₀=`randomError(random,0.02)`. P=I, record `traceBefore=trace(P)`. R=`scaledIdentity(3,1e-4)`. Compute residual, one update.
- **Checks**: `assertSymmetric(P, 1e-9)` and `trace(P) < traceBefore`. Verifies information gain: covariance stays symmetric and trace decreases (Joseph-form-style update).
- **Port notes**: trace comparison uses the mutated P vs the pre-update identity trace (=12).

#### testNormalizedInnovationSquaredMatchesQuadraticForm
- **Scenario**: seed `404L`. ξ₀=`randomError(random,0.03)`. Prior P set to a non-trivial SPD diagonal: `P(i,i)=0.05+0.02*i` for i=0..11 (rest zero). Snapshot P before update (`priorCovarianceSnapshot`). R=`scaledIdentity(3,1e-3)`. Compute residual, snapshot it (`residualSnapshot`). One update.
- **Checks**: `expected = quadraticFormNIS(Jc, priorCovarianceSnapshot, R, residualSnapshot)`; `assertEquals(expected, updater.getNormalizedInnovationSquared(), 1e-9)`. Verifies NIS = rᵀ(H P Hᵀ + R)⁻¹ r computed on the PRIOR P.
- **Port notes**: crucial that NIS uses the prior covariance (before the update mutates P) and the prior residual. Diagonal prior chosen so the quadratic form is non-degenerate.

#### testNormalizedInnovationSquaredIsNaNBeforeAnyUpdate
- **Scenario**: fresh `InvariantUpdater(12)`, no update run.
- **Checks**: `Double.isNaN(updater.getNormalizedInnovationSquared())`. Verifies NIS initializes to NaN so a never-updated value can't read as "in-band."
- **Port notes**: Python should init NIS to `float('nan')`.

#### testNormalizedInnovationSquaredAveragesToMeasurementDegreesOfFreedom
- **Scenario**: seed `505L`, `samples=4000`, `measurementVariance=1e-2`. truth = random state, Jc = contact Jacobian, R = `scaledIdentity(3, 1e-2)`. Innovation std = `sqrt(2 + 1e-2)` (because with P=I and H=[+I,−I], S = H P Hᵀ + R = (2+σ²)I). For each of 4000 samples: fresh `estimate = perturb(truth, new double[12])` (exp(0)=identity, so estimate==truth), P=I; residual drawn per-axis `innovationStd · random.nextGaussian()`; run update; accumulate NIS.
- **Checks**: `meanNIS = sumNIS/4000`; `assertEquals(3.0, meanNIS, 0.25)`. Verifies NIS consistency: when innovation ~ N(0,S), NIS ~ χ²(3) with mean = 3 DOF.
- **Port notes**: 4000 Gaussian draws — Python should use a seeded Gaussian RNG; result is statistical (tol 0.25, std of mean ≈ sqrt(6/4000)≈0.039). Uses a single updater instance reused across samples. S = (2+σ²)I derivation depends on H = [+I at position, −I at contact], P = I.

**Helper/reference methods**:
- `randomState(random)`: `InvariantState(1)` with random rotation/velocity/position/1 contact.
- `forwardKinematicsFromTruth(truth)`: y = Rᵀ(d − p) — subtract position from contact, then `rotation.inverseTransform` (Rᵀ). This is the body-frame contact measurement.
- `randomError(random, scale)`: length-12 array, each `scale·(2·rand−1)` ~ U(−scale, scale).
- `perturb(truth, xi)`: `expXi = exp(xi)` (6×6), estimate.X = expXi · truth.X (left multiplication). Covariance not set here (caller sets it).
- `buildContactJacobian()`: 3×12; H(r, 6+r)=+1, H(r, 9+r)=−1.
- `quadraticFormNIS(H, P, R, r)`: computes hp = H·P; s = hp·Hᵀ (`multTransB`); s += R; sInv = inv(s); returns rᵀ·sInv·r (`dot`). Independent reference for NIS = rᵀ(HPHᵀ+R)⁻¹r.
- `scaledIdentity(size, scale)`: identity·scale.
- `computeResidual(estimate, y, residualToPack)`: r = R̂·y − (d̂ − p̂). Steps: rotatedMeasurement = R̂·y (`transform`); residual = (d̂−p̂) then negate then add rotatedMeasurement → R̂·y − (d̂−p̂); packs 3×1 and returns Euclidean norm.
- `assertSymmetric(matrix, epsilon)`: as before.

---

### InvariantEKFTest.java

- **Purpose**: Tests the `InvariantEKF` orchestrator (pure wiring). Delegation tests confirm `predict`/`update` reproduce standalone `InvariantPropagator`/`InvariantUpdater` (+`ContactUpdater`) bit-for-bit on identical states; plus create/initialize wiring, validation, and an end-to-end predict-then-update error-reduction sanity check.
- **Shared setup**: No `@BeforeEach`. Constants `CONTACTS=1`, `GROUP_SIZE=6`, `TANGENT_SIZE=12`; noise `GYRO_VARIANCE=1e-4`, `ACCEL_VARIANCE=1e-3`, `CONTACT_VARIANCE=1e-6`. Filter built via `InvariantEKF.create(numberOfContacts, gyroVar, accelVar, contactVar)`. `initialize(rotation, velocity, position, Tuple3DReadOnly[] contacts, DMatrixRMaj covariance)` sets X and P. `update(contactIndex, Vector3D measurement, Matrix3D bodyCovariance)` is the high-level 3-arg contact-update form (body-frame measurement + body-frame 3×3 covariance).

#### testCreateWiresConsistentSizes
- **Scenario**: `create(2, ...)`. Then `initialize(identity rotation, zero velocity, zero position, [zero, zero] contacts, scaledIdentity(15, 1.0))`. Then `update(0, (0.1,0.0,-0.5), bodyScaledIdentity(1e-6))`.
- **Checks**: `getNumberOfContacts()==2`; `getState().getGroupSize()==7`; `getTangentSize()==15`. The `update` call must NOT throw `IllegalStateException` (confirms the `ContactUpdater` is wired). No numeric assertion on the update result.
- **Port notes**: For N=2, m=15 covariance. The contract: calling update before wiring a ContactUpdater would throw IllegalStateException — Python must have the contact updater installed by `create`.

#### testInitializeSetsEstimate
- **Scenario**: seed `11L`. `create(1,...)`. Random rotation/velocity/position/contact; covariance = `scaledIdentity(12, 2.5)`. `initialize(...)`. Read back all four components and covariance.
- **Checks**: `rotation.epsilonEquals(out, 1e-12)`, same for velocity, position, contact; `assertMatricesEqual(covariance, getState().getCovariance(), 1e-12)`. Verifies initialize populates X and P and getters read them back.
- **Port notes**: covariance stored directly (scale 2.5·I).

#### testInitializeRejectsWrongContactCount
- **Scenario**: `create(1,...)`. `initialize(... , new Tuple3DReadOnly[0], scaledIdentity(12,1.0))` — zero contacts for a 1-contact filter.
- **Checks**: `assertThrows(IllegalArgumentException)`.
- **Port notes**: contacts array length must equal numberOfContacts.

#### testInitializeRejectsWrongCovarianceSize
- **Scenario**: `create(1,...)`. `initialize(..., [one contact], scaledIdentity(3,1.0))` — covariance 3×3 instead of 12×12.
- **Checks**: `assertThrows(IllegalArgumentException)`.
- **Port notes**: covariance must be m×m = (9+3N)².

#### testPredictDelegatesToPropagator
- **Scenario**: seed `22L`. `create(1,...)`, `initializeRandom`. `reference = copyOf(ekf.getState())`. Standalone `InvariantPropagator(1, gyroVar, accelVar, contactVar)`. Random angularVelocity, linearAcceleration; dt=1e-3. `ekf.predict(av, la, dt)` and `propagator.predict(reference, av, la, dt)`.
- **Checks**: `assertStatesEqual(reference, ekf.getState(), 1e-12)` — compares X and P element-wise. Verifies EKF.predict delegates identically to the standalone propagator.
- **Port notes**: bit-for-bit equivalence (tol 1e-12). Same noise params passed to both.

#### testUpdateDelegatesToUpdater
- **Scenario**: seed `33L`. `create(1,...)`, `initializeRandom`. `reference = copyOf(state)`. Standalone `InvariantUpdater(12)` with `setContactUpdater(new ContactUpdater(1))`. Random measurement; `bodyCovariance = bodyScaledIdentity(1e-6)`. `ekf.update(0, measurement, bodyCovariance)` vs `updater.update(reference, 0, measurement, bodyCovariance, false)`.
- **Checks**: `assertStatesEqual(reference, ekf.getState(), 1e-12)`. Verifies EKF.update delegates identically to standalone updater + contact updater.
- **Port notes**: reveals the 5-arg low-level updater signature `update(state, contactIndex, measurement, Matrix3D bodyCovariance, boolean learnedFlag)` — flag false here. The EKF's 3-arg update maps to this with the flag false.

#### testPredictThenUpdateReducesError
- **Scenario**: seed `44L`. truth = random state; estimate = `perturb(truth, randomError(random, 1e-3))`. `create(1,...)`, `initializeFromState(ekf, estimate, scaledIdentity(12,1.0))`. Random angularVelocity, linearAcceleration; dt=1e-3. Propagate truth with a fresh `InvariantPropagator(1,...)` and ekf with `ekf.predict`. measurement = `forwardKinematicsFromTruth(truth)` (after truth moved). `residualBefore = worldResidualNorm(ekf.getState(), 0, measurement)`. `ekf.update(0, measurement, bodyScaledIdentity(1e-10))`. `residualAfter = worldResidualNorm(...)`.
- **Checks**: `residualAfter < residualBefore`. Verifies the assembled predict→update loop reduces estimation error.
- **Port notes**: measurement computed from truth AFTER the truth predict, so the update targets the propagated truth.

**Helper/reference methods**:
- `initializeRandom(ekf, random)`: random rotation/velocity/position + CONTACTS random contacts, covariance = `scaledIdentity(12, 1.0)`.
- `initializeFromState(ekf, source, cov)`: extracts rotation/velocity/position/contacts from an existing `InvariantState` and initializes the ekf with them.
- `randomState(random)` / `forwardKinematicsFromTruth(truth)` / `randomError(random, scale)` / `perturb(truth, xi)` / `worldResidualNorm` / `copyOf` / `assertStatesEqual` / `assertMatricesEqual` / `scaledIdentity` / `bodyScaledIdentity(scale)` (Euclid `Matrix3D` identity·scale, 3×3 body-frame covariance) — as in InvariantUpdaterTest.

---

### InvariantEKFReseedTest.java

- **Purpose**: Property tests for `InvariantEKF.reseedContact` (touchdown re-seed, "H4 Phase 2", derivation note 2026-07-16): (1) PSD preservation of covariance, (2) covariance consistency P_dd = P_pp + R·N·Rᵀ and P_θd = P_θp, and (3) zero-release — a contact update with the same measurement immediately after re-seed yields ~0 residual/NIS/correction.
- **Shared setup**: No `@BeforeEach`. Constants `N_CONTACTS=2`, `M = 9+3·2 = 15`, `EPS=1e-10`. Filter built directly via constructor `new InvariantEKF(2, 1.0e-4, 1.0e-3, 1.0e-6)` (gyro, accel, contact variances). `reseedContact(contactIndex, Vector3D bodyMeasurement, Matrix3D fkCovariance)` returns a `double` (the pre-reseed residual norm). Covariance tangent indices via `basePositionTangentIndex()` (=6) and `contactTangentIndex(0)` (=9); rotation block starts at 0.

#### testReseedPreservesPositiveSemiDefiniteness
- **Scenario**: seed `4242`. 50 trials. Each: `ekf = randomlyInitializedEKF(random, randomPsd(random, 0.5))`. fkCovariance = identity·`(1e-6·(1+rand))`. newFoothold = `Point3D(0.05·rand, 0.05·rand, 0.01·rand)` (cm-scale displacement). `ekf.reseedContact(0, consistentMeasurement(ekf, newFoothold), fkCovariance)`.
- **Checks**: covariance symmetric (all upper-triangle pairs `assertEquals(P(r,c),P(c,r),1e-10)`); `minEigenvalue(covariance) > -1e-8`. Verifies re-seed (congruence P_{d·}←P_{p·}, P_dd←P_pp+R·N·Rᵀ) keeps P symmetric PSD for any PSD input P.
- **Port notes**: 50 random trials. `randomPsd` builds A·Aᵀ + 1e-9·I. `minEigenvalue` via EJML symmetric eig. Python: numpy eigvalsh, threshold −1e-8.

#### testReseedCovarianceConsistency
- **Scenario**: seed `99`. Single `ekf = randomlyInitializedEKF(random, randomPsd(random, 0.5))`. `before = P.copy()`. pIdx=6, dIdx=9. Read rotation R. fkCovariance = identity·`3.0e-6`. `reseedContact(0, consistentMeasurement(ekf, Point3D(0.1,0.05,0.0)), fkCovariance)`. Compute `rotatedN = R·N·Rᵀ` (Euclid `rotation.transform(Matrix3D)`).
- **Checks**: for a,b in 0..2: `after.get(dIdx+a, dIdx+b) == before.get(pIdx+a, pIdx+b) + rotatedN(a,b)` (tol 1e-10) — i.e. P_dd = P_pp + R·N·Rᵀ; and `after.get(a, pIdx+b) == after.get(a, dIdx+b)` (tol 1e-10) — i.e. P_θd = P_θp (rotation-to-contact cross equals rotation-to-base cross, the K_θ=0 condition).
- **Port notes**: R·N·Rᵀ uses the state rotation. The θ (rotation) block is rows/cols 0..2. Both consistency equations must hold exactly.

#### testZeroReleaseAfterReseed
- **Scenario**: seed `7`. `ekf = randomlyInitializedEKF(random, randomPsd(random, 0.5))`. fkCovariance = identity·`1e-4`. measurement = `consistentMeasurement(ekf, Point3D(0.08,-0.03,0.005))`. `preResidual = ekf.reseedContact(0, measurement, fkCovariance)`. Then read rotationBefore/positionBefore, call `ekf.update(0, measurement, fkCovariance)` (same measurement), read rotationAfter/positionAfter.
- **Checks**: `preResidual > 1e-3` (real geometric discrepancy); after the update: `wasLastUpdateApplied()` true (passes the conditioning gate); `getLastNormalizedInnovationSquared() == 0.0` (tol 1e-9); `getLastCorrectionRotationNorm() == 0.0` (tol 1e-9); rotation unchanged (`epsilonEquals`, 1e-9); position unchanged (1e-9). Verifies zero-release: after re-seeding the contact to be exactly consistent with the measurement, re-applying the same measurement produces zero residual → zero NIS → zero correction (rotation rows of gain K_θ vanish).
- **Port notes**: exercises the EKF's introspection API — `wasLastUpdateApplied()`, `getLastNormalizedInnovationSquared()`, `getLastCorrectionRotationNorm()`, plus a conditioning/NIS gate on `update`. Python port must expose these. Reseed must make the immediately-following identical measurement produce exactly zero innovation.

**Helper/reference methods**:
- `randomlyInitializedEKF(random, cov)`: `new InvariantEKF(2, 1e-4, 1e-3, 1e-6)`. Rotation via `setYawPitchRoll(yaw=rand−0.5, pitch=0.4·(rand−0.5), roll=0.4·(rand−0.5))`. basePosition = `Point3D(rand, rand, 0.9+0.1·rand)`. Contacts (2): each `Point3D(baseX + 0.3·(rand−0.5), baseY + (i==0? +0.1 : −0.1), 0.0)` (feet near ground, offset ±0.1 in y). Zero base velocity. `initialize(rotation, zeroVelocity, basePosition, contacts, cov)`.
- `randomPsd(random, scale)`: A is M×M with entries `scale·(rand−0.5)`; psd = A·Aᵀ; add 1e-9 to each diagonal.
- `minEigenvalue(symmetric)`: EJML symmetric `EigenDecomposition_F64`, min real eigenvalue.
- `consistentMeasurement(ekf, worldFoothold)`: y = Rᵀ(worldFoothold − basePosition). Generates a body-frame FK measurement exactly consistent with the chosen world foothold.

#### Cross-file port notes (shared conventions the Python suite must replicate)
- **Group/tangent layout**: group X is (5+N)×(5+N); tangent m = 9+3N; tangent block order rotation(0), base velocity(3), base position(6), contact i (9+3i). Group columns: rotation 3×3, velocity col, position col, then contact cols.
- **Right-invariant error**: η = X̂·X⁻¹, with perturbation applied as X̂ = exp_G(ξ)·X (left multiplication).
- **Gravity**: (0,0,−9.81); IMU `accel` is specific force (add gravity internally); rest → accel = (0,0,+9.81).
- **Measurement model**: body-frame FK y = Rᵀ(d − p); world residual r = R̂·y − (d̂ − p̂); Jacobian H = [+I at position block, −I at contact block]; innovation covariance S = H P Hᵀ + R; NIS = rᵀ S⁻¹ r; contact/measurement covariance supplied body-frame and rotated by R.
- **RNG/seeds**: fixed `Random(seed)` per test (seeds: 1234/5768/7777/8888/9999; 1234/2345/3456/4567; 12345/2024; 101/202/303/404/505; 11/22/33/44; 4242/99/7). All assertions are tolerance-based, so an equivalent RNG with the same distributions and the same trial counts suffices. Trial counts to preserve: 1000 (SEK3Utils), 500/50/20/200 (State/Propagator), 4000 (Updater NIS mean), 50 (Reseed PSD).
- **Tolerances** (exact): 1e-10 default several places, 1e-9 (homomorphism, several kinematic), 1e-8 (conjugation), 1e-12 (state round-trips, EKF delegation), 1e-6 (residual-to-zero), statistical 0.25 (mean NIS), 1e-8 eigenvalue floor.
- **Introspection API on InvariantEKF**: `wasLastUpdateApplied()`, `getLastNormalizedInnovationSquared()`, `getLastCorrectionRotationNorm()`, conditioning gate on `update`; `reseedContact` returns the pre-reseed residual norm.
- **Two update signatures**: low-level `InvariantUpdater.update(state, H, residual, R)` (generic linear) and `update(state, contactIndex, measurement, Matrix3D bodyCov, boolean)` (contact-specialized, needs an installed `ContactUpdater(N)`); high-level `InvariantEKF.update(contactIndex, measurement, bodyCov)`.

---

## invariant_estimator contact / support tests

### ContactUpdaterTest.java
- **Purpose**: Tests the contact forward-kinematics measurement subpiece `ContactUpdater` and its orchestration through `InvariantUpdater.update(state, contactIndex, measurement, bodyCovariance, learnedFlag)`. Measurement is body-frame contact position y = R̂ᵀ(d − p); right-invariant world residual is r = R̂·y − (d̂ − p̂).
- **Shared setup** (class constants): `CONTACTS = 1`, `GROUP_SIZE = 6`, `TANGENT_SIZE = 12`, `POSITION_BLOCK = 6`, `CONTACT_BLOCK = 9`. No `@BeforeEach`; each test builds its own `Random` with a fixed seed. `randomState`: random rotation matrix, random base velocity, base position, contact position (all from `EuclidCoreRandomTools`).

#### testContactUpdateDrivesResidualToZero
- **Scenario**: seed `101L`. `truth = randomState`. `measurement = forwardKinematicsFromTruth(truth)` (exact, noise-free y = Rᵀ(d−p)). `estimate = perturb(truth, randomError(scale=1.0e-4))` — a left-multiplicative exp(ξ)·truth perturbation with each of 12 ξ components uniform in ±1.0e-4. Covariance = I (12×12). Body measurement covariance = identity·1.0e-10.
- **Checks**: `worldResidualNorm` before and after one `updater.update(estimate, 0, measurement, bodyCov, false)`. Assert `residualAfter < 1.0e-6` and `residualAfter < residualBefore`.

#### testContactUpdateReducesResidualForLargerError
- **Scenario**: seed `202L`. Same construction, but error scale `0.05` (moderate) and body covariance identity·1.0e-8.
- **Checks**: `residualAfter < 0.1 * residualBefore` — one update reduces residual by >10× (rules out sign flip).

#### testCovarianceShrinksAndStaysSymmetric
- **Scenario**: seed `303L`. Error scale `0.02`, covariance identity, `traceBefore = trace(P)`. Body covariance identity·1.0e-4. One update.
- **Checks**: `assertSymmetric(P, 1.0e-9)`; `trace(P) < traceBefore` (information gained).

#### testJacobianStructureAndStateIndependence
- **Scenario**: seed `404L`. Two `computeJacobian(randomState, 0, H)` calls into separate 3×12 matrices H1, H2 with different random states.
- **Checks**: (1) State independence — every element H1(r,c) == H2(r,c) exactly (tol 0.0). (2) Structure — for r,c in 0..2: expected = 1.0 if `c == POSITION_BLOCK + r` (c=6+r), −1.0 if `c == CONTACT_BLOCK + r` (c=9+r), else 0.0, matched exactly (tol 0.0). So H = [0(3×6) | +I(3×3) | −I(3×3)].

#### testResidualFormula
- **Scenario**: seed `505L`. `state = randomState`. `measurement = nextVector3D` (arbitrary, not FK-consistent). `computeResidual(state, 0, measurement, residual)` into 3×1.
- **Checks**: expected = R̂·measurement − (contactPos − basePos), computed independently with Euclid `rotation.transform`. Each of 3 components matched within `1.0e-12`.

#### testMeasurementCovarianceIsRotatedToWorld
- **Scenario**: seed `606L`. `state = randomState`. `bodyCovariance = symmetricMatrix3D(random)` (½(A+Aᵀ) of a random 3×3 in ±1). `computeMeasurementCovariance(state, bodyCov, actual)` into 3×3.
- **Checks**: expected = R̂·N·R̂ᵀ computed via EJML mult + multTransB. Element-wise tol `1.0e-12`.

#### testMapEncoderNoise
- **Scenario**: seed `707L`. `nJoints = 6`. `contactJacobian` random 3×6 (±1). `jointCovariance` symmetric 6×6. Static call `ContactUpdater.mapEncoderNoise(J, Σ, actual)` (actual is Matrix3D).
- **Checks**: expected = J·Σ·Jᵀ. Element-wise tol `1.0e-12`.
- **Port notes**: N = J·Σ·Jᵀ. Static/free function.

#### testUpdateWithoutContactUpdaterThrows
- **Scenario**: seed `808L`. `InvariantUpdater(TANGENT_SIZE)` with NO `setContactUpdater` call. Body covariance identity·1.0e-6.
- **Checks**: `update(state, 0, measurement, bodyCov, false)` throws `IllegalStateException`.

#### testLearnedModuleNotImplementedThrows
- **Scenario**: seed `909L`. ContactUpdater installed. Call `update(..., true)` (learnedFlag = true).
- **Checks**: throws `NotImplementedException` (Apache commons-lang3).
- **Port notes**: The `learned` measurement branch is unimplemented; Python raises `NotImplementedError`.

**Helper/reference methods (math)**:
- `randomState(random)`: InvariantState(1) with random rotation/velocity/position/contact.
- `forwardKinematicsFromTruth(truth)`: y = Rᵀ(d − p).
- `randomError(random, scale)`: 12-vector, each = scale·(2·rand−1), uniform ±scale.
- `perturb(truth, xi)`: expXi = SEK3_Utils.exp(xi) (6×6); estimate.groupElement = expXi · truth.groupElement.
- `worldResidualNorm(estimate, i, meas)`: ‖R̂·y − (d̂ − p̂)‖.
- `symmetricMatrix(size, random)`: A random ±1; return ½(A + Aᵀ). `symmetricMatrix3D`: same wrapped as Matrix3D.
- **Port-wide notes**: Fixed seeds 101/202/303/404/505/606/707/808/909 — Java `Random` + `EuclidCoreRandomTools` sampling will NOT reproduce identical numbers in Python, so port must use tolerance-based/property assertions; the FK/residual/covariance references are all self-consistent (recomputed from the same random state), so they port cleanly regardless of RNG.

---

### FootSwitchContactProbabilityProviderTest.java
- **Purpose**: Property tests for `FootSwitchContactProbabilityProvider`'s debounced Schmitt-trigger contact trust and its EMA-smoothed probability, plus trust-mode switching (SCHMITT/LEGACY/NONE). Anchored to hardware log 20260717_112516 impact-blip signature.
- **Shared setup** (instance fields reinitialized per test method by JUnit):
  - `DT = 1.0e-3` (1 kHz).
  - `switches`: `SideDependentList<ScriptedFootSwitch>` with a distinct ScriptedFootSwitch for LEFT and RIGHT. Each exposes 3 scriptable signals: `filtered` (hasFootHitGroundFiltered), `sensitive` (hasFootHitGroundSensitive), `load` (getFootLoadPercentage). Other FootSwitchInterface methods inert (CoP distance NaN, CoP null, wrench null, measurement frame = world).
  - `provider = new FootSwitchContactProbabilityProvider(switches, DT, registry)` (default-parameter constructor).
  - Inferred default Schmitt/dwell parameters: ENTER (upper) load threshold ≈ 0.35, STAY (lower) load threshold ≈ 0.25, dwell ≈ 40 ms (40 ticks), plus an EMA smoothing time constant. A "sensitive-only" state yields probability approaching 0.5 from below. Trusted state → probability EMA-rises to 1.0.
  - `tick(ticks, filtered, sensitive, load)` sets ONLY the LEFT switch's signals and calls `provider.update()` that many times (RIGHT stays at its init default).
  - `settleUntrusted()`: `tick(500, false,false,0.0)`; asserts LEFT `isTrusted == false` and `getContactProbability(LEFT) == 0.0` (tol 1e-6).

#### startsTrustedWithFeetPlanted
- **Scenario**: Fresh provider, no ticks. Then `tick(100, true,true,0.5)`.
- **Checks**: Immediately `isTrusted(LEFT) == true`, `probability(LEFT) == 1.0` (tol 1e-12) — anchored from first tick, no init transient. After 100 loaded ticks still trusted, probability 1.0 (tol 1e-6).

#### impactBlipIsRejectedAndRealLoadingTrustsOnce
- **Scenario**: `settleUntrusted()` first. Three phases, counting trust rising edges (`isTrusted && !previousTrusted`):
  - Phase 1 (170 ticks): filtered=true, sensitive=true; load = 0.38 for ticks i∈[50,80) (30 ms above enter threshold ~0.35), else 0.15. Track `maxProbabilityDuringBlip`.
  - Phase 2 (600 ticks): filtered=false, sensitive=false, load=0.0 (force dropout).
  - Phase 3 (300 ticks): filtered=true, sensitive=true, load=0.8 (genuine load).
- **Checks**: After phase 1, `trustRises == 0` (30 ms over-threshold < 40 ms dwell → never enters trust) AND `maxProbabilityDuringBlip < 0.5`. After phase 2, cumulative `trustRises == 0`. After phase 3, cumulative `trustRises == 1` (exactly one rise), `isTrusted(LEFT)==true`, `probability(LEFT)==1.0` (tol 1e-6).
- **Port notes**: Dwell = 40 ticks; enter threshold between 0.35 and 0.38; sensitive-only asymptote < 0.5. Property: one physical strike → exactly one trust rise.

#### loadChatterInsideSchmittBandCannotToggleTrust
- **Scenario**: `tick(100, true,true,0.5)` → trusted. Then 50 cycles: each `tick(10,true,true,0.28)` then `tick(10,true,true,0.45)` — oscillating load across ENTER (0.35) but staying above STAY (0.25). Finally `tick(100, true,true,0.20)` (below stay for the dwell).
- **Checks**: During every cycle `isTrusted==true` and after the 0.28 leg `probability==1.0` (tol 1e-6) — hysteresis holds trust continuously. After dropping to 0.20 for 100 ticks, `isTrusted==false`.
- **Port notes**: STAY threshold ≈ 0.25 (0.28 stays, 0.20 exits); Schmitt band [0.25, 0.35).

#### subDwellSwitchChatterNeverTrusts
- **Scenario**: `settleUntrusted()`. 100 cycles of `tick(20,true,true,1.0)` (fully loaded 20 ms) then `tick(20,false,false,0.0)` (off 20 ms) — each burst shorter than the 40 ms dwell.
- **Checks**: After every loaded burst `isTrusted(LEFT)==false`.

#### nanLoadDegradesToSwitchOnlyTrust
- **Scenario**: `settleUntrusted()`. `tick(100, true,true, NaN)` then `tick(100, false,false, NaN)`.
- **Checks**: After loaded+NaN, `isTrusted==true` (NaN load → load gate passes; trust follows dwelled filtered boolean alone). After unloaded+NaN, `isTrusted==false`.
- **Port notes**: NaN load must not permanently lock a foot out.

#### probabilityStaysInUnitIntervalUnderRandomChatter
- **Scenario**: seed `24601L`. 20000 iterations: each tick random filtered/sensitive booleans; load = NaN with 1/10 probability else `1.5*rand` (can exceed 1.0). Both sides checked.
- **Checks**: `probability(side) ∈ [0,1]` for every tick, both sides.
- **Port notes**: Property-only assertion, so RNG differences don't matter.

#### invertedSchmittBandIsRejected
- **Scenario**: Full-parameter constructor `new FootSwitchContactProbabilityProvider(switches, DT, new YoRegistry("bad"), 0.8, 0.25, 0.35, 0.04)` — Schmitt band arguments inverted (stay/enter swapped relative to valid ordering); trailing 0.04 s = 40 ms dwell.
- **Checks**: constructor throws `IllegalArgumentException`.
- **Port notes**: Constructor validates enter > stay; dwell 0.04 s confirms the 40-tick dwell.

#### legacyModeReproducesUndebouncedBehavior
- **Scenario**: `setTrustMode(TrustMode.LEGACY)`; `settleUntrusted()`. `tick(170, true,true,0.15)` (blip: filtered high, load far below enter). Then `tick(600, false,false,0.0)`.
- **Checks**: After blip, `probability(LEFT) > 0.99` (LEGACY fully trusts raw filtered boolean via EMA). After dropout, `probability == 0.0` (tol 1e-6). But `isTrusted(LEFT)==false` — the SCHMITT state machine kept advancing underneath and never entered trust.

#### noneModePinsFullTrust
- **Scenario**: `setTrustMode(TrustMode.NONE)`; seed `603L`. 5000 ticks random filtered/sensitive, load `1.5*rand`.
- **Checks**: every tick `probability(LEFT)==1.0` and `probability(RIGHT)==1.0` (tol 1e-12, no lag).

#### liveModeSwitchResumesFromCurrentDebounceState
- **Scenario**: `settleUntrusted()` (SCHMITT). `setTrustMode(NONE)`; `tick(50,false,false,0.0)`. Then `setTrustMode(SCHMITT)`; `tick(100,false,false,0.0)`.
- **Checks**: Under NONE, `probability==1.0` (tol 1e-12) despite unloaded. After switching back to SCHMITT with no load, `isTrusted==false` and `probability==0.0` (tol 1e-6) — state machine kept advancing under NONE; only the output selection changes.

**Port-wide notes**: Depends on IHMC `FootSwitchInterface` (implement a scriptable stub exposing filtered/sensitive/load), `SideDependentList`/`RobotSide`, `YoRegistry` (no-op stub). Fixed seeds 24601/603 feed property-only assertions. All timing in ticks at DT=1e-3. This whole suite IS portable.

---

### GravityLevelingUpdaterTest.java
- **Purpose**: Tests the accelerometer gravity-leveling (roll/pitch) measurement — the attitude observation the contact update lacks. Verifies analytic residual/Jacobian, anisotropic measurement covariance R (loose pitch, tight roll), the quasi-static/horizontal/rotation gates, the complementary (gyro-fused) gravity reference that rejects lateral-accel artifacts while passing true tilt, filter-level leveling that preserves unobservable yaw, and P symmetric-PSD invariants.
- **Shared setup** (class constants):
  - `G = 9.81`; `UP = (0,0,1)`.
  - `ROLL_VAR = 2.5e-3` (≈(2.9°)²); `PITCH_VAR = 1.9e-1` (≈(25°)²); `DT = 1.0e-3`.
  - `BALANCE_OMEGA = 2π·0.49` rad/s (0.49 Hz hardware balance mode).
  - `PREDICTED_ARTIFACT_GAIN = 1/√(1+(BALANCE_OMEGA·5.0)²)` ≈ 0.065 (τ = 5 s low-pass at ω_b, ≈15× rejection).
  - `GravityLevelingUpdater` constructors: `(tangentSize, isotropicVar, G)` and `(tangentSize, rollVar, pitchVar, G)`.
  - `InvariantEKF(numContacts, gyroVar, accelVar, contactVar)` used with `(0, 1.0e-7, 1.0e-7, 1.0e-12)`.
  - No @BeforeEach. All tests deterministic (no RNG).

#### testUprightGivesZeroResidualAndTilt
- **Scenario**: `InvariantState(0)`, rotation identity. Updater `(tangentSize, 2.5e-3, G)`. `assemble(state, (0,0,G))`.
- **Checks**: residual x,y,z all 0.0 (tol 1e-12); `getTiltErrorAngle()==0.0` (tol 1e-12).

#### testPitchTiltDiagnosticAndResidual
- **Scenario**: θ=0.20. State rotation identity (estimate upright). True specific force for body pitched +θ about y = g·[−sinθ, 0, cosθ]. Updater `(tangentSize, 2.5e-3, G)`, `assemble`.
- **Checks**: `getTiltErrorAngle()==|θ|` (tol 1e-9); `getTiltErrorPitch()==−sinθ` (tol 1e-9); `getTiltErrorRoll()==0` (tol 1e-9). Residual = [−sinθ, 0, cosθ−1] (tol 1e-9 each). Jacobian H (3×tangentSize): its δφ_z column (index 2) is exactly 0.0 for all 3 rows (yaw unobservable, tol 0.0).

#### testGravityUpdateLevelsTiltAndPreservesYaw
- **Scenario**: trueGravity=(0,0,G). (1) `InvariantEKF(0, 1e-7,1e-7,1e-12)`, m=9 (N=0). Init rotation = yawPitchRoll(0, 0.20, 0). Velocity/position zero, no contacts, P = I(9×9). Loop 200×: `assembleGravityLeveling(trueGravity)`, `applyGravityLeveling()`. (2) Second EKF init rotation yawPitchRoll(0.7, 0, 0), loop 50×.
- **Checks**: (1) Each iter: `wasLastUpdateApplied()==true`; `getLastConditionProxy()` finite; `assertSymmetricPSD(P)`. After loop: `tiltAngle < 1e-3`; final yaw==0 (tol 1e-9), roll==0 (tol 1e-9). (2) After 50 iters: yaw==0.7 preserved (tol 1e-9); `tiltAngle < 1e-9`.

#### testAnisotropicMeasurementCovarianceStructure
- **Scenario**: State identity. Updater `(tangentSize, ROLL_VAR, PITCH_VAR, G)`, `assemble(state,(0,0,G))`. `getMeasurementCovariance()` 3×3.
- **Checks**: R(0,0)==PITCH_VAR (pitch-informing x), R(1,1)==ROLL_VAR (roll-informing y), R(2,2)==ROLL_VAR (gravity-null z) — all tol 1e-12. Off-diagonals == 0 (tol 1e-12), symmetric (tol 1e-15).

#### testPitchCorrectionAuthorityBelowRoll
- **Scenario**: θ=0.10, p0=1.0e-2 (converged covariance). `rollReduction = θ − oneStepTiltAfterCorrection(0,0,θ, p0)`; `pitchReduction = θ − oneStepTiltAfterCorrection(0,θ,0, p0)`.
- **Checks**: `rollReduction > 5.0*pitchReduction`; `pitchReduction > 0`.
- **Port notes**: per-tick Kalman gain P/(P+σ²); pitch:roll ratio ≈ (P+σ_roll²)/(P+σ_pitch²).

#### testRollStillLevelsUnderAnisotropy
- **Scenario**: EKF `(0,1e-7,1e-7,1e-12)`, init rotation yawPitchRoll(0,0,0.20), P=I. Loop 200× assemble+apply with trueGravity=(0,0,G).
- **Checks**: `tiltAngle < 1e-3`.

#### testPitchGateFreezesPitchButNotRoll
- **Scenario**: State identity. Updater(ROLL_VAR, PITCH_VAR, G). `setPitchObservable(false)`, `assemble(state,(0,0,G))`.
- **Checks**: `R(0,0) > 1e3` (pitch variance jumps to disabled value); `R(1,1)==ROLL_VAR` (tol 1e-12).

#### testHorizontalAccelGateRejectsForeAftButPassesGravity
- **Scenario**: State identity. Updater. zeroOmega. `settleGravityReference(updater,(0,0,G))` (3000 ticks of updateGravityReference). Then foreAft = (3.0, 0, √(G²−9)) ≈ (3,0,9.34), ‖foreAft‖ within ±5% of g. One `updateGravityReference(foreAft, zeroOmega, DT)`; `assemble(state, foreAft)`.
- **Checks**: `isQuasiStatic(gravity, zeroOmega, 0.05, 0.15, 0.5)==true` (pure gravity passes). `|‖foreAft‖−G| ≤ 0.05·G` (norm gate alone would pass). `isQuasiStatic(foreAft, zeroOmega, 0.05, 0.15, 0.5)==false` (horizontal-accel component rejects).
- **Port notes**: `isQuasiStatic(specificForce, omega, normTol=0.05, rotTol=0.15, horizTol=0.5)`.

#### testQuasiStaticGateIsIndependentOfEstimatorAttitude (regression, FINDINGS §F.3)
- **Scenario**: Updater, zeroOmega, trueGravity=(0,0,G) (robot static). `settleGravityReference`. Sweep degrees 0..15: state rotation `setToPitchOrientation(radians(deg))`, `assemble(state, trueGravity)`.
- **Checks**: For every tilt 0..15°, `isQuasiStatic(trueGravity, zeroOmega, 0.05, 0.15, 0.5)==true`. (Old gate resolved horizontal force against ĝ=R̂ᵀe_z giving g·sinθ, locking out at θ>2.92°.)
- **Port notes**: Gate must depend on the sensor-driven reference, not the estimate's attitude.

#### testRotationGateUsesRawGyroNotBiasCorrupted
- **Scenario**: State identity. Updater. gravity=(0,0,G). settleGravityReference, assemble.
- **Checks**: `isQuasiStatic(gravity, (0,0,0), ...)==true`; `isQuasiStatic(gravity, (0,0.3,0), ...)==false` (0.3 rad/s > 0.15 gate). Rotation gate uses raw gyro ω.

#### testPitchDistrustAxisIsBodyYAtNonZeroYaw
- **Scenario**: State rotation = `setToYawOrientation(radians(90))`. Updater(ROLL_VAR,PITCH_VAR). gravityBody = yawed⁻¹·(0,0,G). `assemble(state, gravityBody)`. R = `getMeasurementCovariance()`. ĝ = normalized gravityBody. pitchResidualDir = normalize(e_y × ĝ); rollResidualDir = normalize(e_x × ĝ).
- **Checks**: `quadraticForm(R, pitchResidualDir)==PITCH_VAR` (tol 1e-9); `quadraticForm(R, rollResidualDir)==ROLL_VAR` (tol 1e-9) — pitch distrust follows BODY axis even at 90° yaw.

#### testLateralAccelArtifactIsRejectedAtTheBalanceFrequency (roll-sway (1))
- **Scenario**: State identity, never rotates (true tilt=0). Updater(ROLL_VAR,PITCH_VAR). lateralAccelAmplitude=0.21 m/s²; rawArtifactAmplitude=0.21/G. `settleGravityReference((0,0,G))`. Run 12000 ticks (12 s); measure after 8000. Each tick t: specificForce=(0, 0.21·sin(ω_b t), G). `updateGravityReference(sf, zeroOmega, DT)`; `assemble(state, sf)`; track max|residual.y| after 8000.
- **Checks**: `achievedGain = maxResidualY/rawArtifactAmplitude < 0.1` (>10× rejection). Also `achievedGain == PREDICTED_ARTIFACT_GAIN` (tol 0.03) — matches 1/√(1+(ωτ)²) with τ=5 s.

#### testTrueTiltStillPassesAtUnityGainAtTheBalanceFrequency (roll-sway (2))
- **Scenario**: Updater(ROLL_VAR,PITCH_VAR). rollAmplitude=0.03 rad. settleGravityReference((0,0,G)). 12000 ticks, measure after 8000. Each tick: roll=0.03·sin(ω_b t), rollRate=0.03·ω_b·cos(ω_b t). trueRotation=yawPitchRoll(0,0,roll). specificForce = trueRotation⁻¹·(0,0,G). omega=(rollRate,0,0). `updateGravityReference(sf, omega, DT)`. After 8000: error = getGravityReference() − trueRotation⁻¹·UP; track max‖error‖.
- **Checks**: `maxTrackingError < 0.1·rollAmplitude` — reference tracks true tilt at unity gain (gyro carries it). A naive low-pass would fail.

#### testStaticTiltStillProducesFullResidual (roll-sway (3), DC authority)
- **Scenario**: roll=0.05 rad. State rotation identity. Updater(ROLL_VAR,PITCH_VAR). trueRotation=yawPitchRoll(0,0,0.05); specificForce = trueRotation⁻¹·(0,0,G). `settleGravityReference(updater, specificForce)`; `assemble(state, specificForce)`.
- **Checks**: residual.x==0 (tol 1e-6); residual.y==sin(0.05) (tol 1e-4); residual.z==cos(0.05)−1 (tol 1e-4). Full undiminished DC tilt.

**Helpers/references**:
- `oneStepTiltAfterCorrection(yaw,pitch,roll,p0)`: EKF(0,1e-7,1e-7,1e-12); init rotation yawPitchRoll, P=p0·I; one assembleGravityLeveling((0,0,G))+apply; assert symmetric-PSD; return tiltAngle.
- `quadraticForm(R,u)`: uᵀRu.
- `settleGravityReference(updater, sf)`: 3000× updateGravityReference(sf, zeroOmega, DT).
- `tiltAngle(ekf)`: acos(clamp((R̂ᵀe_z)·e_z)).
- `assertSymmetricPSD(P)`: symmetric within 1e-9; Cholesky of P + 1e-12·I succeeds.

**Port-wide notes**: No RNG — all deterministic analytic/time-domain; fully portable. Port must replicate: complementary gravity reference (τ=5 s) fusing gyro+accel, anisotropic R mapped to residual directions, quasi-static gate (norm 0.05, rotation 0.15 rad/s raw gyro, horizontal 0.5), pitch-observable gate, condition gating with `wasLastUpdateApplied`/`getLastConditionProxy`. EKF API: `assembleGravityLeveling`, `applyGravityLeveling`, `getRotation`.

---

### TouchdownReseedLatchTest.java
- **Purpose**: Property tests for `TouchdownReseedLatch` — a state machine ensuring the touchdown re-seed fires at most once per sustained-low (swing) episode, immune to the mid-strike p:1→0→1 double pulse (hardware log 20260717_112516).
- **Shared setup** (class constants): `TRIGGER = 0.5` (rising-crossing fire threshold), `REARM = 0.1` (low threshold that, sustained, re-arms), `DWELL_TICKS = 100` (100 ms @ 1 kHz consecutive-below-REARM ticks needed to re-arm). Constructor: `TouchdownReseedLatch(trigger, rearm, dwellTicks, initialArmedFlag)`; tests pass initial armed = false. `drive(latch, probability, ticks)`: calls `latch.advance(probability)` `ticks` times, returns count of `true` returns (fires). API: `advance(p)` → boolean, `isArmed()`.

#### firesOnceOnCleanTouchdownAndNotAgainWhileHigh
- **Scenario**: latch(0.5,0.1,100,false). `drive(0.0, 100)` (swing arms). Then `drive(1.0, 500)`.
- **Checks**: swing drive returns 0 fires; `isArmed()==true` after swing; high drive returns exactly 1 fire (rising crossing); `isArmed()==false` after firing.

#### midStrikeDropoutCannotDoubleFire
- **Scenario**: `drive(0.0,500)`; `drive(1.0,90)` → asserts 1 fire. `drive(0.0, 99)` (dropout = DWELL_TICKS−1, shorter than re-arm dwell). Then `drive(1.0,500)`.
- **Checks**: 99-tick dropout returns 0 fires; `isArmed()==false`; subsequent high drive returns 0 fires (same physical strike, no re-seed).

#### sustainedSwingRearmsForTheNextStrike
- **Scenario**: `drive(0.0,500)`; `drive(1.0,200)` → 1 (strike N); `drive(0.0,300)` → 0 (genuine swing ≥ dwell re-arms); `drive(1.0,200)`.
- **Checks**: strike N fires 1; swing returns 0 and `isArmed()==true`; strike N+1 returns 1.

#### singleTickDipsNeverRearm
- **Scenario**: `drive(0.0,500)`; `drive(1.0,90)` → 1. Then 1000 iterations of `drive(0.05,1)` (1-tick dip below rearm) + `drive(0.9,5)` (high), summing fires.
- **Checks**: total fires == 0 (dwell counter resets on each high tick → never re-arms).

#### midBandProbabilityNeitherArmsNorFires
- **Scenario**: fresh latch(...,false). `drive(0.3, 1000)` (p between REARM and TRIGGER). Then `drive(1.0, 100)`.
- **Checks**: mid-band drive returns 0 fires; `isArmed()==false`; subsequent high drive returns 0 (unarmed latch never fires).

#### atMostOneFirePerSustainedLowEpisodeUnderRandomChatter
- **Scenario**: seed `1868L`. latch(0.5,0.1,100,false). 200,000 iterations of `latch.advance(rand)`. Reference model in the test: `consecutiveLow` increments while p<REARM (reset otherwise); reaching DWELL_TICKS sets `episodeCredit=true`.
- **Checks**: whenever latch fires, assert `episodeCredit==true`, then clear credit. Property: never fires without a preceding ≥100-tick sub-REARM episode; at most one fire per episode.
- **Port notes**: Property-only; port with its own RNG and the same reference-model check.

#### constructorRejectsDegenerateConfigurations
- **Scenario/Checks**: `TouchdownReseedLatch(0.5, 0.5, 100, false)` throws `IllegalArgumentException` (trigger == rearm, no hysteresis band); `TouchdownReseedLatch(0.5, 0.1, 0, false)` throws (dwellTicks == 0).

**Port-wide notes**: No IHMC/Euclid/EJML dependency — pure scalar state machine, fully portable. State machine: armed only after ≥DWELL_TICKS consecutive p<REARM; fires exactly once on the rising crossing p≥TRIGGER while armed; disarms on fire; brief dips (< dwell) do not re-arm.

---

### InvariantMainStateEstimatorTest.java
- **Purpose**: "Full filter" integration tests of `InvariantMainStateEstimator` — the InEKF floating-base estimator wrapping a `JointLevelKFPreFilter`, driven by a synthetic `RandomFullHumanoidRobotModel` + hand-fed `SettableTestSensorMap` (no SCS simulation). Locks in assembled-pipeline behavior and NaN hardening.
- **Shared setup**:
  - `DT = 1.0e-3`; `WORLD = ReferenceFrame.getWorldFrame()`.
  - `Rig` holds model, joints, sensorMap, pelvisIMU, imus list, preFilter, main estimator, ekf. `doControl()` → `main.doControl()`.
  - `buildRig(seed, baseOrientation, contactProbability, poisonLegImu)`:
    - `Random(seed)`; `RandomFullHumanoidRobotModel(random)`; `MultiBodySystemRandomTools.nextState(random, CONFIGURATION, joints)`; all Qd=0; root joint config zero; update frames.
    - IMUs: pelvisIMU on pelvis, left/rightFootIMU on feet (`SettableTestIMU`). If `poisonLegImu`, `leftFootIMU.setBiasProcessNoiseCovarianceNonFinite()` BEFORE pre-filter build.
    - `SettableTestSensorMap(imuOutputs, joints)`; each joint's sensor position = model q.
    - Pre-filter pairs: `("leftLeg"/"rightLeg", true, "pelvisIMU", "leftFootIMU"/"rightFootIMU", 0.0, 0.0)`. feet = [LEFT foot, RIGHT foot]. `preFilter = JointLevelKFTestSupport.newPreFilter(sensorMap, pairParams, feet, DT, YoRegistry("preFilter"))`.
    - `InvariantMainStateEstimator(model, sensorMap, "pelvisIMU", null, DT, gyroVariance=1.0e-4, accelVariance=1.0e-3, contactVariance=1.0e-6, contactMeasurementVariance=1.0e-6, initialCovariance=1.0, enableYawSeeding=false, preFilter)`.
    - `setContactProbabilityProvider(constantContact(contactProbability))`; `initializeEstimator(baseTransform with rotation=baseOrientation, empty TObjectDoubleHashMap)`.

#### testStaticEquilibriumStaysPutAndFinite
- **Scenario**: `buildRig(4001L, identity, 1.0, false)` (feet in contact). `setGravityConsistentAccel` (each IMU specific force = world (0,0,9.81) expressed in IMU frame). All gyros 0. 1000 `doControl()` ticks.
- **Checks**: `ekf.getRotation` distance to identity < 1e-3; `getBaseVelocity().norm() < 1e-2`; for each pre-filter joint, `preFilter.getEstimatedJointPosition(joint) == encoder` (tol 5e-3); `assertPipelineFinite`.

#### testNoContactRotationIntegrates
- **Scenario**: `buildRig(4002L, identity, 0.0, false)` (no contact → hold mode). 1000 ticks; each tick `setRigidRotationGyros(0, 0.2, 0)` (0.2 rad/s about world Y) then doControl.
- **Checks**: swept angle = `rotation.distance(identity)`; expected = 0.2 rad; `assertEquals(expected, swept, 2.0e-2)`; `assertPipelineFinite`.

#### testHangingUnderGravityIsCompensated
- **Scenario**: tilt = `setToRollOrientation(radians(20))`. `buildRig(4003L, tilt, 0.0, false)` (hold mode). setGravityConsistentAccel at 20° tilt. gyros 0. 1000 ticks.
- **Checks**: rotation distance to tilt < 1e-3; velocity.norm < 1e-3; position drift.norm < 1e-3; `assertPipelineFinite`.

#### testFreeFallDoesNotRunAwayOrFail
- **Scenario**: `buildRig(4004L, identity, 0.0, false)`. All IMUs gyro 0, linear accel 0 (weightless). 2000 ticks, checking velocity each tick.
- **Checks**: every tick `getBaseVelocity().norm() < 1e-6` (no velocity ramp — hold mode); final position drift.norm < 2e-2; `assertPipelineFinite`.

#### testPoisonedLegImuDoesNotFailPipeline
- **Scenario**: `buildRig(4005L, identity, 1.0, true)` — left-foot IMU bias process-noise covariance non-finite before pre-filter build. setGravityConsistentAccel, gyros 0. 500 ticks (must not throw).
- **Checks**: `rotation.distance(identity)` finite; `assertPipelineFinite`.

**Helpers**:
- `setGravityConsistentAccel`: each IMU specific force = FrameVector3D(WORLD,0,0,9.81) → changeFrame(IMU frame) → setLinearAcceleration.
- `setRigidRotationGyros(wx,wy,wz)`: common world angular velocity projected to each IMU body frame.
- `assertPipelineFinite`: rotation/velocity/position finite; every covariance element finite; covariance symmetric (|P(r,c)−P(c,r)| < 1e-6).
- `constantContact(p)`: ContactProbabilityProvider with constant probability p.

**Port-wide notes**: Heaviest IHMC coupling (`RandomFullHumanoidRobotModel`, Mecano, Euclid frames, Trove). A Python port needs an equivalent kinematic robot model + IMU/joint sensor scaffolding and the ported JointLevelKF, or these become end-to-end scenario tests against the ported pipeline. Fixed seeds 4001–4005 feed geometry only; assertions are physical-property (tolerances 1e-3…2e-2), so RNG differences are acceptable. Constructor exposes the full InEKF parameter set: gyroVar 1e-4, accelVar 1e-3, contactVar 1e-6, contactMeasVar 1e-6, initialCov 1.0, enableYawSeeding false.

---

### InvariantEstimatorAllocationTest.java
- **Purpose**: JVM allocation regression guard — the InEKF predict/update hot path and the SE_k(3)/SO(3) Lie-group scratch overloads must allocate zero bytes per tick on the estimator thread. **NOT portable to Python.**
- **Shared setup**: `NUMBER_OF_CONTACTS = 2`; `DT = 0.001`; `MAX_BYTES_PER_TICK = 16.0`; `WARMUP_TICKS = 50_000`; `MEASURED_TICKS = 200_000`. Uses `com.sun.management.ThreadMXBean.getThreadAllocatedBytes`.

#### testPredictUpdateHotPathIsAllocationFree
- **Scenario**: `InvariantEKF(2, 1e-4, 1e-3, 1e-6)`. Init: identity rotation, zero velocity/position, contacts at (0.1,0.1,0) and (0.1,−0.1,0); initialCovariance = I(15×15). Per tick: `setContactSlipVariance(0,1e-6)`, `(1,1e-6)`, `predict((0.01,−0.02,0.03), (0,0,9.81), DT)`, `update(0, (0,0.12,−0.90), diag(1e-4))`, `update(1, same)`. Warm 50k ticks, measure 200k.
- **Checks**: bytes/tick < 16.0.

#### testLieGroupScratchOverloadsAreAllocationFree
- **Scenario**: xi = 9-vector {0.03,−0.02,0.05,0.10,−0.20,0.30,−0.40,0.50,0.05}; per tick: `SEK3_Utils.exp(xi, groupElement, scratch...)`, `SEK3_Utils.adjoint(...)`, `SEK3_Utils.log(...)`, `SO3LieGroupTools.exp(omega=(0.01,0.02,−0.03), rotationIncrement)` — all with pre-allocated scratch.
- **Checks**: bytes/tick < 16.0.

**Port notes — NOT PORTABLE**: JVM-specific (per-thread allocation counter, JIT warm-up). OMIT from the Python port; the scenario values are only useful as a functional smoke test of the same calls if desired.

---

## Porting guide (consolidated)

### What ports bit-for-bit

- **Deterministic matrix references** — replicate exactly, they seed most priors and Jacobians:
  - `spd(size, seed)`: fill row-major `m[i] = sin(i + 1 + seed)`, `A = m·mᵀ + size·I`.
  - `genericH(k, dim, seed)`: `H[r,c] = sin(0.37·(r·dim + c + 1) + seed)`.
  - Seeded prior mean patterns (predict: `i+1` / `i+1+100` / `i+1+1000`; update: `0.1·(i+1)`; stacked reference: `0.05·(i+1)−0.1`).
- **Reference-KF cross-checks** (independent of the implementation under test): explicit-inverse Joseph
  update (`JointLevelKFUpdateTest.referenceUpdate`), information-form nuisance-marginalized stacked
  reference (`JointLevelKFStackedReferenceTest.referenceMarginalized`), LU-based Schur complement
  (`referenceSchur`), Gram-vs-dense Qa (`JointLevelKFRotorAndGramTest`), quadratic-form NIS
  (`quadraticFormNIS`), analytic error transition `exp(A·T)` (`InvariantPropagatorTest.buildErrorTransition`).
- **Assertion helpers**: `assertAllClose` (elementwise `|a−e| ≤ tol`), `assertSymmetric`
  (upper-triangle pairwise), `assertPositiveSemiDefinite` (`λ_min ≥ −1e-6·max(λ_max, 1)`; use
  `numpy.linalg.eigvalsh`).

### RNG strategy

Java `java.util.Random(seed)` / `EuclidCoreRandomTools` streams cannot be reproduced in NumPy.
This is fine: every randomized test is either (a) property-based (the reference is recomputed from
the same random draw, so any seeded RNG works), or (b) statistical with wide envelopes
(χ² NIS means over 4000 trials, envelope `4·sqrt(2/N)`). Preserve the **trial counts**
(1000 for Lie-group identities, 500 round-trips, 4000 NIS samples, 50 reseed-PSD trials,
200k latch-chatter ticks, 12/8 stacked-reference trials) and use a fixed Python seed per test for
run-to-run determinism. `sigmaFor`-style maps use Java `String.hashCode()` — either reimplement
Java's hashCode (`h = 31·h + c`) or substitute any deterministic per-joint map.

### The one hard dependency: kinematics

Tests that only exercise linear algebra (state/predict/update/noise/Lie group/latch/Schmitt/
gravity leveling) port directly. Tests that read Jacobians, frame rotations, or mass matrices off
a Mecano `RandomFloatingRevoluteJointChain` or `RandomFullHumanoidRobotModel` need a Python
kinematics stand-in. Two viable routes:

1. **Reimplement the fixture**: a serial revolute chain with axes cycling X/Y/Z (`i%3`), random
   configuration/velocity from the port's own RNG, plus FK, geometric (angular) Jacobians, and a
   composite-rigid-body mass matrix (e.g. Pinocchio, or a small hand-rolled serial-chain library).
   The tests never depend on the *specific* random geometry — only on self-consistency between
   the fixture's kinematics and the filter's kinematics — so exact Mecano reproduction is NOT needed.
2. **Inject known transforms**: restructure geometry-dependent tests around fixed, hand-chosen
   link transforms. More work per test but removes the kinematics library dependency.

`applyConsistentMotion` (zero base twist; gyros = link angular velocity from FK of the commanded
q/q̇) is the key fixture reference to keep: it guarantees encoder and gyro measurements are consistent
with the same truth, which is what the trajectory/tracking tolerances assume.

### Not portable / adapt

- **Skip**: `JointLevelKFPreFilterAllocationTest` (2 of 3 tests) and `InvariantEstimatorAllocationTest`
  (both tests) — JVM `ThreadMXBean` per-thread allocation counters; no meaningful Python equivalent.
  Keep `testHotPathStaysFinite` as a smoke test.
- **Adapt**: bit-exact `tol=0.0` assertions (determinism, exact symmetry, exact innovation) assume
  identical floating-point operation ordering; in Python assert determinism against the Python
  implementation's own repeat run, and symmetry to ~1e-15 relative where the operation order differs.
  Exception: exact-symmetry of Van-Loan Q blocks holds if the port also symmetrizes with `0.5·(A+Aᵀ)`.
- **Adapt**: YoVariable-published diagnostics (`jointKF_encNIS_*`, `jointKF_qdR_*`, singular-innovation
  message strings) — port the *observable* (per-joint NIS value, per-joint R value, degenerate-row
  attribution), not the YoRegistry/string plumbing.

### Filter constants the tests lock in (must match the Python estimator)

| Constant | Value | Where asserted |
|---|---|---|
| `DT` | 1e-3 s | everywhere |
| Init prior vars (q / q̇ / b) | 1e-6 / 1.0 / 2.5e-3 | StateTest |
| IMU bias process var | 1e-4 (per axis) | PredictTest, fixture |
| `SIGMA_ACCEL` (scalar-path CWNA) | 50.0 rad/s² | TransitionNoiseTest |
| `ENCODER_VAR` fallback | 5.0e-5 | TransitionNoiseTest, NIS tests |
| `SIGMA_TAU` fallback | 5.0 N·m | MassMatrixNoise, StandingStability |
| σ_τ per-joint | `0.15 · effortLimit` | MassMatrixNoise |
| Rotor-inertia table | HIP_X 0.062, HIP_Y/KNEE 0.167, ANKLE_Y 0.070, ANKLE_X 0.050, SPINE 0.062, default 0.005 | RotorAndGramTest |
| `ANCHOR_VAR` (stance anchor Σ_ε) | 4.0e-4 | StackedReference, BiasObservability |
| `SIGMA_QD_UNFILTERED` | 0.1 rad/s | BiasObservability, DirectVelocity |
| Direct-velocity slew smoother | 5 Hz first-order | DirectVelocityMeasurementTest |
| InEKF gravity | (0, 0, −9.81) | PropagatorTest |
| InEKF tangent layout | rot 0, vel 3, pos 6, contact i at 9+3i | InvariantStateTest |
| Gravity-leveling R | roll 2.5e-3, pitch 1.9e-1 | GravityLevelingUpdaterTest |
| Quasi-static gates | norm 0.05·g, rotation 0.15 rad/s, horizontal 0.5 m/s² | GravityLevelingUpdaterTest |
| Gravity-reference complementary τ | 5 s | GravityLevelingUpdaterTest |
| Contact-trust Schmitt band / dwell | stay 0.25, enter 0.35, 40 ms | FootSwitchContactProbabilityProviderTest |
| Reseed latch | trigger 0.5, rearm 0.1, dwell 100 ticks | TouchdownReseedLatchTest |

### Test seams the Python estimator must expose

- **JointLevelKF**: `initialize`, `predict`, `josephUpdate(H, z, R[, label])`, `computeJointState`,
  `computeImuBiases(feet)`, `setStateForTest(x, P)`, `getStateVector`, `getCovariance`,
  `getTransitionMatrix`, `getProcessNoise`, `getEncoderJacobian`, `getEncoderNoise`,
  `getVelocityMeasurementJacobian/Noise`, `buildStackedMeasurementForTest`,
  `getStackedMeasurementJacobian/Residual/Noise`, `getStackedRowForPair`, `getMixingOperator`,
  `getPairParentBiasColumn/ChildBiasColumn/VelocityColumns`, `getBiasBlockColumn`,
  `setTrustedFeetForTest`, `getActiveAnchorCountForTest`, `getAngularVelocityBiasInIMUFrame`,
  `getEstimatedJointPosition/Velocity`, `isUsingMassMatrixProcessNoise`,
  `updateProcessNoiseFromMassMatrixForTest`, `reflectedRotorInertiaForNameOrDefault`,
  `describeSingularInnovation`, `setDirectVelocityMeasurementForTest`, `refreshDirectVelocityNoise`.
- **InEKF stack**: `SEK3_Utils.exp/log/adjoint` (+ size validation), `InvariantState`
  getters/setters + tangent indices + `setToIdentity`, `InvariantPropagator.predict`,
  `InvariantUpdater.update` (generic 4-arg and contact 5-arg forms) + `getNormalizedInnovationSquared`
  (NaN before first update) + `setContactUpdater`, `ContactUpdater.computeJacobian/computeResidual/`
  `computeMeasurementCovariance/mapEncoderNoise`, `InvariantEKF.create/initialize/predict/update/`
  `reseedContact` (returns pre-reseed residual) / `assembleGravityLeveling/applyGravityLeveling/`
  `wasLastUpdateApplied/getLastNormalizedInnovationSquared/getLastCorrectionRotationNorm/`
  `getLastConditionProxy/setContactSlipVariance`, `GravityLevelingUpdater.assemble/`
  `updateGravityReference/isQuasiStatic/setPitchObservable` + tilt diagnostics,
  `FootSwitchContactProbabilityProvider` (+ `TrustMode` enum), `TouchdownReseedLatch.advance/isArmed`.

### Suggested porting order

1. `SEK3UtilsTest` → `InvariantStateTest` → `InvariantPropagatorTest` (pure math, no fixtures).
2. `TouchdownReseedLatchTest`, `FootSwitchContactProbabilityProviderTest`, `GravityLevelingUpdaterTest`
   (deterministic, no kinematics).
3. `ContactUpdaterTest` → `InvariantUpdaterTest` → `InvariantEKFTest` → `InvariantEKFReseedTest`.
4. JointLevel linear-algebra core: `StateTest`, `PredictTest`, `UpdateTest`, `TransitionNoiseTest`,
   `RotorAndGramTest`.
5. Build the Python chain fixture (route 1 above), then: `MeasurementTest`, `FilterTest`,
   `TrajectoryTest`, `MassMatrixNoiseTest`, `StandingStabilityTest`, `BiasObservabilityTest`,
   `StackedReferenceTest`, NIS tests, `SingularInnovationDiagnosticTest`.
6. Optional: an `InvariantMainStateEstimatorTest` analogue as end-to-end scenario tests once a full
   robot-model stand-in exists.
