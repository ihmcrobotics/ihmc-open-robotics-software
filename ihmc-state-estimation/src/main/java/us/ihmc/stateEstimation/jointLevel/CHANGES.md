# JointLevelKFPreFilter — Rev. 1 → Rev. 2, Change 1: Schur-complement process noise

Scope of this change: **Change 1 only** (SPEC `jointKF_derivation.md` §3.2). The measurement model
(Change 2, stacked gyro update) and the phase merge (Change 3) are **not** in this change — the per-pair
sequential gyro updates and the phase-2 stance anchor are untouched. Everything else in the class (state
layout, `ProprioceptivePreFilter` API, encoder update, init/on-ground gating, NaN-guard/rollback, YoVariable
publishing) is unchanged.

## What moved

Rev. 1 process noise for the joint blocks was the **locked-base** map `Qa = σ_τ² M_jj(q)⁻²`, with the mass
matrix computed over the filtered joints only. Rev. 2 replaces it with the **floating-base Schur complement**
(SPEC §3.2):

```
Qa = σ_τ² Λ(q)⁻²,   Λ = M_jj − M_jb M_bb⁻¹ M_bj   (Schur complement of the 6-DoF base block)
```

Because the free base recoils, the joints accelerate more per unit torque: `Λ ⪯ M_jj`, so `Λ⁻² ⪰ M_jj⁻²`
and the effective process noise **grows** at fixed `σ_τ`, worst for proximal joints.

### Code changes (`JointLevelKFPreFilter.java`)

- **Mass-matrix system now includes the floating base.** The `CompositeRigidBodyMassMatrixCalculator` is
  built over `{6-DoF floating base} ∪ {joints spanning base→filtered}` instead of the joints-only system.
  The base joint is found as the 6-DoF child of the tree root (`findFloatingBaseJoint`); if there is none
  (fixed-base model) the filter degrades to the scalar-CWNA fallback with a warning.
- **Topology generalization (nuisance block).** The composite-rigid-body calculator prunes the entire
  subtree below any *ignored* joint, so an unfiltered joint sitting *above* a filtered joint cannot simply be
  locked — it would zero the filtered joints' inertia. Such "gap" joints are therefore **included** in the
  considered set and **marginalized** (treated as free) alongside the base. The marginalized "nuisance" block
  `N` is `{base 6-DoF} + {gap joints}`, and `Λ = M_ff − M_jN M_NN⁻¹ M_Nj`. In the **real gapless topology**
  (base IMU on the base link, every path joint filtered) there are no gap joints and this is *exactly* the
  SPEC's `Λ = M_jj − M_jb M_bb⁻¹ M_bj`. Genuinely off-path joints (e.g. arms when only legs are filtered)
  stay ignored and their inertia is composited into the adjacent link, as before. `collectSpanningJoints`
  walks base→filtered to build the span.
- **Per-tick Schur algebra** (`updateProcessNoiseFromMassMatrix`): extract `M_NN`, `M_Nf`, `M_ff` by resolved
  column index (never by assumed ordering); Cholesky-solve `M_NN X = M_Nf`; form `Λ = M_ff − M_Nf^T X`;
  symmetrize `Λ`; Cholesky-invert; square; fill the Van Loan blocks (`dt³/3`, `dt²/2`, `dt` pattern —
  unchanged) with the **symmetrized read** `0.5·(A(i,j)+A(j,i))` of `Λ⁻²`.
- **Pre-warming / allocation-free.** Two Cholesky solvers (`N×N` for `M_NN`, `n×n` for `Λ`) plus all scratch
  matrices are sized and warmed in `allocate()`. The per-tick path only `.set()`/`reshape()`s within warmed
  capacity — verified by `testSchurProcessNoiseHotPathIsAllocationFree` (< 32 B/tick).
- **Failure handling unchanged in spirit.** Non-finite `M`, Cholesky rejection of a non-PD `M_NN` or `Λ`, or
  a non-finite intermediate ⇒ keep the previous `Q`, `warnMassMatrixFailureOnce`, continue. The scalar-CWNA
  no-model fallback path is untouched.
- **`SIGMA_TAU` not retuned** (see open items).

## Tests — which test covers which trap

All in `us.ihmc.stateEstimation.jointLevel` (test source set), all green.

| Test | What it locks in |
|---|---|
| `JointLevelKFMassMatrixNoiseTest.testProcessNoiseEqualsVanLoanOfSchurComplementInverseSquared` | **Decisive oracle for the Schur algebra.** Filter `Q` joint blocks == Van Loan of `σ_τ² Λ⁻²`, with `Λ` recomputed by a fully independent reference (second calculator, plain-EJML LU invert, independent block extraction). Localizes any block-extraction or sign error in `M_NN`/`M_Nf`/`M_ff`. |
| `…testSchurComplementIsSymmetricPDAndDominatedByLockedInertia` | `Λ` symmetric PD; PSD ordering `Λ ⪯ M_ff` (`M_ff − Λ ⪰ 0`). At a strongly-bent configuration so the coupling `M_jN` is non-trivial. |
| `…testVanLoanBlocksAreExactlySymmetric` | The symmetrized read makes every joint block of `Q` bit-exactly symmetric and the two `q–q̇` cross blocks bit-exactly equal (0.0 tolerance) — the Joseph update depends on it. |
| `…testProcessNoiseIsConfigurationDependent` / `…testPredictRefreshesQAndKeepsCovarianceSymmetricPSD` | `Q` tracks `M(q)` across configuration changes and `predict()` (not just the test hook) does the refresh; `P` stays symmetric PSD through propagation. |
| `…testMassMatrixAndScalarPathsDiffer`, `…testProcessNoiseCouplesJointsThroughInertia`, `…testBiasBlockUntouchedByMassMatrixPath`, `…testProcessNoiseSymmetricPSDOnMassMatrixPath`, `…testMassMatrixPathEnabledOnlyWithModel` | Schur path differs from scalar; dense `Λ⁻²` couples joints; the bias random-walk block and zero joint/bias coupling are untouched; `Q` symmetric PSD; model ⇒ Schur path, no model ⇒ scalar fallback. |
| `JointLevelKFPreFilterAllocationTest.testSchurProcessNoiseHotPathIsAllocationFree` | The per-tick Schur rebuild (block extraction + `M_NN` solve + `Λ` invert) allocates < 32 B/tick on the estimator thread. |

The independent reference in the test (`referenceSchur`) replicates the *model definition* (spanning subtree
+ nuisance partition) but computes the linear algebra by a separate path (LU, its own calculator instance),
so agreement to round-off validates the filter's own extraction/solve/subtract, not a shared implementation.

## Convention-bound lines — review by hand (do not self-approve)

Change 1 has cheap decisive oracles (symmetry, PSD-ness, independent finite reference), so it is largely
self-checking. The two judgment calls that a human should confirm:

1. **Base-joint identification** (`findFloatingBaseJoint`): assumes the 6-DoF child of the tree root is the
   free-flyer. Correct for IHMC humanoid models (elevator → SixDoFJoint → pelvis). Confirm for the Alex model.
2. **Gap-joint marginalization vs. locking.** SPEC §3.2 says non-filtered joints are *locked*; this change
   *marginalizes* any unfiltered joint that sits between the base and a filtered joint (a "gap" joint). This
   is deliberate — the CRB calculator cannot lock a joint while keeping its descendants — and is moot for the
   real robot (no gap joints; base IMU on the base link, all path joints filtered). Confirm the Alex pair/anchor
   topology has **no** gap joints; if it does, decide whether marginalize (free, current) or lock is intended.

## Open items for a human — `TODO(retune)`

- **`SIGMA_TAU` retune (SPEC §8).** `Λ⁻² ⪰ M_jj⁻²`, so the effective `Qa` is larger than Rev. 1 at the same
  `σ_τ = 50 N·m`. The old value is **not** carried over as calibrated — retune against quiet-standing and
  walking NIS on the gyro block. A `// TODO(retune)` citing §8 is on `updateProcessNoiseFromMassMatrix`.
- **Base-wrench extension (SPEC §3.2, deferred).** If quiet-standing NIS runs cold after the `Λ` switch, the
  next term is the base-wrench uncertainty mapped through `−Λ⁻¹ M_jb M_bb⁻¹`, *not* a `σ_τ` retune.

## Not done in this change (future work)

- ~~Change 2 — stacked gyro update with `R_g = L Σ Lᵀ + Σ_ε` block (SPEC §5).~~ **Done — see below.**
- ~~Change 3 — phase merge of the stance anchor into the phase-1 stacked update (SPEC §6).~~ **Done — see below.**

---

# JointLevelKFPreFilter — Rev. 2, Changes 2 & 3: stacked gyro measurement + phase merge

Scope of this follow-up: **Change 2** (stacked gyro measurement, SPEC §5) and **Change 3** (phase merge,
SPEC §6). Builds on the Change-1 Schur process noise (already committed) and does **not** touch it, the state
layout, the `ProprioceptivePreFilter` API, the encoder update, init/on-ground gating, the NaN-guard/rollback
philosophy, or YoVariable publishing. `SIGMA_TAU` retune remains open (Change 1's item).

## What moved

Rev. 1 applied the gyros as **one Joseph update per IMU pair** (block-diagonal per-pair `R`) plus a
**separate phase-2** stance anchor. Both are replaced by **one stacked update per tick** over
`z_g ∈ R^{3(E+K)}` (E = pairs, K = active stance anchors, 0–2), with

```
H_g = [ 0 | J_stack(q̂) | L(q̂) ],   R_g = L Σ Lᵀ + blkdiag(0_pairs, Σ_ε per anchor)
```

`L` is the rotational edge-incidence operator; **the bias columns of `H_g` ARE `L`** (built once per tick,
used in both places). `Σ = blkdiag` of each IMU's angular-velocity **measurement**-noise covariance (not the
bias process noise). Because biases and gyro white noise enter through the same `L`, a block-diagonal per-pair
`R` is inconsistent — it double-counts shared-IMU samples on a star topology (Alex's layout) and both anchors'
shared base sample in double support; `R_g` carries the exact cross-covariances (SPEC §5.3).

### Code changes (`JointLevelKFPreFilter.java`)

- **`buildStackedMeasurement()`** (new) assembles `H_g`, `z_g`, `R_g`, and `L` (`Lmix`): pair `e` at rows
  `[3e,3e+3)` with residual `R(child→J_e)ω_child − R(parent→J_e)ω_parent`, q̇-columns `+J_e`, `L`-blocks
  `+R(child→J_e)`/`−R(parent→J_e)`; each active anchor with raw base gyro, q̇-columns `−J_leg`, `L`-block
  `+I3` on the base IMU. `R_g = LΣLᵀ` via a pre-sized dense congruence, then `+Σ_ε` (`anchorR`) on each anchor
  diagonal, then `symmetrize` (reuses the Change-1 helper). **Raw `getAngularVelocityMeasurement()` only** —
  no filtered/Mahony signal (SPEC §4.2).
- **`computeJointState()`** replaces the per-pair loop with one guarded `josephUpdate(H_g, z_g, R_g,
  "stackedGyroUpdate")`. If ANY entry of `z_g`/`H_g`/`R_g` is non-finite the **whole** block is skipped (never
  individual rows — a partial stack silently changes the bias-gauge structure, SPEC §6 step 3).
- **`computeImuBiases(trustedFeet)`** reduced to caching the trusted-feet set (`trustedFeetFromLastTick`); the
  phase-1 stacked update reads the **previous tick's** set (SPEC §6 phase note). First tick after init: empty
  set ⇒ K = 0 ⇒ pairs-only. No measurement update runs in phase 2.
- **Removed:** `pairGyroUpdate`, `buildPairMeasurement`, `stanceAnchorUpdate`, `congruenceAdd` (+ the `H`,
  `zMeas`, `zAnchor`, `R3`, `tmp3a`, `tmp3b` scratch). **Added:** `Hg/zg/Rg/Lmix/Sigma/LSigma`, `identity3`,
  `imusByOrdinal` (index loop avoids a per-tick map-iterator allocation), `E`/`maxStackRows`.
- **Allocation-free.** `allocate()` sizes the stacked scratch at `3(E+K_max)` and warms the innovation LU
  solver at `max(n, 3(E+K_max))`; per tick only `.reshape()`/`.set()` within capacity. Verified by the
  existing allocation test (< 32 B/tick over `computeJointState`+`computeImuBiases`).
- **Test seams:** the per-pair seams (`buildPairMeasurementForTest`, `getMeasurement{Jacobian,Residual,Noise}`)
  are replaced by stacked seams (`buildStackedMeasurementForTest`, `getStackedMeasurement{Jacobian,Residual,
  Noise}`, `getMixingOperator`, `setTrustedFeetForTest`, `getStackedRowForPair`, `getBiasBlockColumn`,
  `getImuOrdinal`, `getBaseIMU`). The pair column getters (`getPairParentBiasColumn`, …) are kept.

## Tests — which test covers which trap

`ihmc-state-estimation` (`us.ihmc.stateEstimation.jointLevel`):

| Test | What it locks in |
|---|---|
| `JointLevelKFStackedOracleTest` | **THE decisive oracle (SPEC §9).** The stacked posterior == a fully independent reference KF that measures the RAW per-IMU gyros (block-diagonal noise) + a foot-rate≈0 constraint per trusted foot, over a state augmented with a nuisance base rate ω_base, marginalized. Built from a different decomposition (absolute base→IMU Jacobians/rotations, independent noise) so a sign/frame/block/`R_g`-correlation error disagrees. Matches to ~1e-6 over 20 randomized ticks, double-support and pairs-only. |
| `JointLevelKFMeasurementTest` (migrated) | Encoder `[I|0]`; pair block `z = ω_child − R ω_parent`, `+R_child`/`−R_parent` bias blocks; **`testBiasColumnsOfHgAreExactlyL`** (bias columns of `H_g` bit-identical to `L`); `R_g` symmetric PSD; `R_g` built from the **measurement** covariance, not the bias process covariance. |
| `JointLevelKFUpdateTest` (migrated) | Joseph update vs explicit-inverse reference KF; bias observability now driven through the stacked seam. |
| existing `Filter`/`Trajectory`/`State`/`Predict`/`TransitionNoise`/`MassMatrixNoise`/`Allocation` | Unchanged and green — state layout, F/Q, predict, full-tick orchestration, stance-phase bias convergence, and < 32 B/tick allocation on the new per-tick path. |

`alex` (`us.ihmc.alex.estimation.AlexJointLevelKFStackedEquivalenceTest`): end-to-end on the **real Alex model**
(REAL_ROBOT, pelvis-star IMU topology). A moving feet-untrusted phase (velocity converges to the true q̇ through
Alex's real pair Jacobians; positions track encoders; biases ≈ 0) and a static double-support phase (both
anchors active — the anchor×anchor shared-base cross-block — grounds the biases; joint covariance blocks stay
finite/symmetric/bounded). The decisive **reference-KF** equivalence is the synthetic `JointLevelKFStackedOracleTest`
because the filter's package-private prior/covariance seams are not reachable from the `alex` module (and
`ihmc-state-estimation` cannot depend on `alex`); the Alex test validates the same assembly against an exact
ground-truth reference through the public API.

## Convention-bound lines — review by hand (do not self-approve)

`JointLevelKFStackedOracleTest` pins all of these numerically, but per SPEC §9 confirm each by eye:

1. **`L` sign table** (`buildStackedMeasurement`): pair rows `+R(child→J_e)` in the child IMU's bias column,
   `−R(parent→J_e)` in the parent's — consistent with `setKinematicChain(parent, child)` and the residual
   `fvA(child) − fvB(parent)`.
2. **Anchor signs:** q̇-columns `−J_leg`, `L`-block `+I3` on the base IMU (J frame = base measurement frame, so
   the anchor's own rotation is exactly identity — not generalized away).
3. **Frames:** every rotation packed into `L` goes IMU-measurement-frame → Jacobian frame via
   `packRotationToJacFrame` (reused, not reimplemented).
4. **`Σ` source:** `getAngularVelocityNoiseCovariance` (measurement), **not**
   `getAngularVelocityBiasProcessNoiseCovariance`.

## Open items for a human — `TODO(retune)`

- The Change-1 `SIGMA_TAU` retune still stands. **Additionally**, re-check the **gyro-block NIS** on hardware
  after the stacked switch: if pair NIS runs hot specifically during swing, the designated knob is a
  rate-dependent `R_g` inflation `κ·diag(‖q̇_path‖²)·δq²` (SPEC §5.4), not a global one. Validate `ANCHOR_VAR`
  (`Σ_ε`) against the anchor-row marginal NIS (SPEC §6/§8).
