# Contact detection for the invariant EKF — theory, history, and the 2026-07-17 trust debounce

Companion note to `ContactProbabilityProvider`, `FootSwitchContactProbabilityProvider`,
`KinematicContactDetector`, `TouchdownReseedLatch`, and the contact sections of
`InvariantEKFStateEstimator`. Investigation evidence: hardware log
`20260717_112516_Alex002UnifiedControlProcess`, walking bout $t \in [2290, 2360]$ s, and
`.claude-reports/2026-07-17-contact-detection-old-vs-new.md`.

---

## 1. The seam: sensing vs. trust

The invariant filter never consumes a contact *boolean*. The seam is
`ContactProbabilityProvider`: once per tick, per foot, a continuous

$$p_i \in [0, 1], \qquad 1 = \text{fully trusted contact}, \quad 0 = \text{fully muted},$$

and everything downstream is a *soft* function of $p_i$. This is deliberate: the InEKF's contact
foot-position states want graded evidence, not a switch, and the seam is where a learned contact
module (ContactNet) will eventually plug in.

### 1.1 How $p$ enters the filter (`InvariantEKFStateEstimator`)

Two multiplicative covariance knobs, both of the form $c^{\,1-p}$ with $c = 90$:

**Knob 1 — measurement noise inflation.** The contact forward-kinematics update
$y_i = R^\top(d_i - x)$ (foot position in body frame) is *always applied* for both feet, but with

$$R_i(p) \;=\; \underbrace{c^{\,1-p}}_{\text{inflation}} \cdot R_i^{\text{FK}},
\qquad c^{\,1-p} \in [1, 90].$$

At $p = 1$ the foot constrains the base at full FK confidence; at $p = 0$ its influence is
attenuated $90\times$ (soft mute, not a skip). The Kalman gain scales roughly as
$K \sim P H^\top (HPH^\top + R)^{-1}$, so the inflation continuously dials how hard the foot
"drags" the base velocity.

**Knob 2 — contact-position process (slip) noise.** Before the prediction consumes $Q$:

$$\sigma_{c,i}^2(p) \;=\; c^{\,1-p} \cdot \sigma_{c}^2\big|_{\text{contact}},$$

so a swing foot's world anchor *forgets* its stale position (large random-walk noise) and
re-anchors softly at touchdown instead of releasing accumulated error as a base kick.

**Hard gates keyed on $p$** (bare 0.5 thresholds — this matters in §4):

- *No-contact hold*: both feet $p < 0.5$ → the whole contact update is skipped, base velocity
  zeroed (the hanging-robot path).
- *Gravity-leveling double-support gate*: a foot "counts as planted" iff $p \ge 0.5$.
- *Touchdown re-seed* (`InvariantEKF.reseedContact`, H4 Phase 2): fires when $p$ rises through
  0.5 while armed; re-arms at $p < 0.1$. Clones the base-position covariance into the contact
  block so the fresh anchor releases zero residual.
- *Yaw anchor* (`FootReferencedYawCorrector`): anchors at $p \ge 0.8$, releases at $p \le 0.3$.

Note the asymmetry that will bite in §4: the *derived events* (re-seed, yaw anchor) have
hysteresis; the R-inflation and the 0.5 gates track $p$ **instantaneously in both directions**.

---

## 2. The pre-existing providers

### 2.1 `KinematicContactDetector` — the FK-only fallback

Inputs: sole height above a flat ground plane $h_i$, and its finite difference
$\dot h_i = (h_i - h_i^{\text{prev}})/\Delta t$. No force, no switch. Two logistic gates,
multiplied:

$$p_{\text{raw}} \;=\;
\underbrace{\sigma\!\left(\frac{h_0 - h}{w_h}\right)}_{\text{"low enough"}}
\cdot
\underbrace{\sigma\!\left(\frac{v_0 - |\dot h|}{w_v}\right)}_{\text{"slow enough"}},
\qquad \sigma(x) = \frac{1}{1+e^{-x}},$$

with flat-ground defaults $h_0 = 4$ cm, $w_h = 8$ mm, $v_0 = 0.5$ m/s, $w_v = 0.1$ m/s, then a
one-pole smoother $p \leftarrow \alpha p + (1-\alpha)p_{\text{raw}}$, $\alpha = 0.6$.
Probabilities start at **0**.

Reading it as a probability model: the height gate is a sigmoid observation model for "sole at
ground level" with an 8 mm transition band; the speed gate *suppresses* the transient phases
(a foot sweeping past $h_0$ during swing is fast, so $|\dot h| > v_0$ kills the product). It is
**memoryless** — both gates are functions of the current instant only, and the EMA low-passes
but never latches.

Known failure mode (why it is only the fallback): it reads sole heights **in world through the
estimator's own model**. Run as the *main* estimator's detector, that is circular — unobservable
world-Z drift lowers/raises the perceived sole height and can mute a planted foot, which removes
the very measurement that would have arrested the drift. It also assumes flat ground at a known
height. In sim it doubles as a convenient ground-truth source when driven by the ground-truth
model.

### 2.2 `FootSwitchContactProbabilityProvider` — production (pre-2026-07-17 form)

Wraps the robot's production `FootSwitchInterface`s — on Alex the `JointTorqueBasedFootSwitch`:
ankle-pitch (ANKLE_Y) torque mapped through the leg Jacobian transpose to an estimated vertical
foot force, dual force thresholds (low 60 N "sensitive" + CoP validity, high 150 N ignoring CoP),
and a 25-tick glitch-filter window. Fixes the kinematic detector's two failure modes in main
mode: joint-torque detection is independent of the base estimate (no circularity), and
probabilities start at **1** (feet are planted when `initializeEstimator` runs, so the filter is
anchored from tick 0).

The original mapping was a three-level quantizer plus the same EMA:

$$p_{\text{raw}} = \begin{cases}
1 & \texttt{hasFootHitGroundFiltered()} \\
0.5 & \text{else } \texttt{hasFootHitGroundSensitive()} \\
0 & \text{otherwise,}
\end{cases}
\qquad p \leftarrow \alpha p + (1-\alpha) p_{\text{raw}}, \quad \alpha = 0.8 .$$

$\alpha = 0.8$ at 1 kHz is a $\tau = -\Delta t / \ln \alpha \approx 4.5$ ms time constant — the
EMA exists purely so the exponentiated inflation laws $c^{1-p}$ see a continuous ramp instead of
a step. **It is smoothing, not debouncing**: it delays both edges by ~5 ms and prevents nothing.
All the temporal structure of $p$ was the switch's, verbatim.

---

## 3. What the old (DRC) estimator did instead — `isFootTrusted`

The baseline `PelvisLinearStateUpdater` consumed the *same* foot switches but wrapped them in
three layers of armor before anything touched the state estimate (binary per-foot trust,
recomputed each tick):

1. **Extra dwell** (`delayTimeBeforeTrustingFoot`, Alex 20 ms): a `GlitchFilteredYoBoolean` on
   top of the switch's own 25-tick window — the filtered contact boolean must hold for the full
   window before a foot can change trust state. A minimum time-in-contact.
2. **Load-percentage Schmitt trigger** (when both feet pass layer 1): foot load
   $\ell_i = F_{z,i}/\sum_j F_{z,j}$ must exceed **0.35** to *become* trusted, and trust holds
   until $\ell_i$ drops below **0.25** (`forceInPercentOfWeightThresholdToTrustFoot` /
   `...ToNotTrustFoot`). A genuine hysteresis band: a foot breathing around one threshold cannot
   chatter, and an impact spike that never carries real weight never enters.
3. **Degenerate-case latches**: if nothing qualifies, either keep last tick's trusted set or
   trust the single most-loaded/slowest foot; a slipping foot (planar angular velocity
   > 2.5 rad/s) is untrusted.

Crucially, trust only selected **which feet feed the kinematic update** — flipping it swapped an
anchor with no covariance modulation, no correction impulse, no re-seed.

---

## 4. The pathology (hardware log 20260717_112516)

The joint-torque switch has a structural blind spot: the estimated foot force comes from
ankle-pitch torque through a lever arm, so **when the CoP passes under the ankle the estimated
force collapses even under full load**. On the log, one left-foot touchdown at $t \approx 2308$
reads:

| $t$ (s) | event |
|---|---|
| 2308.09 | impact: estimated force spikes 40 → 277 N, switch fires |
| 2308.09–2308.18 | $p_L: 0 \to 1$ in ~90 ms; re-seed fires; $R$-inflation 90 → 1 |
| 2308.14–2308.20 | estimated base $v_z$ jumps $-0.02 \to +0.09$ m/s in ~60 ms |
| 2308.18–2308.31 | force collapses to 20–60 N *during weight transfer onto this foot* |
| 2308.31 | switch drops → $p_L: 1 \to 0$ in ~30 ms, $R$ re-inflates $90\times$ mid-transfer |
| 2308.31–2308.94 | support foot, yet $p_L = 0$ for 600 ms (anchored on the other foot alone) |
| 2308.94 | ankle torque builds → $p_L: 0 \to 1$, **second re-seed for one physical touchdown** |

Bout statistics (70 s, ~11 left steps): $p_L$ crossed 0.5 **45 times** (~2× per physical
contact), left re-seed count **+23** (~2 per step), 13 periodic ~40 ms "loaded > 20 % of weight
but $p < 0.5$" events. The old pipeline, fed the identical switch signal, was structurally
immune: the impact blip peaked at 32 % load — below the 0.35 enter threshold — and layers 1–3
turned each strike into at most one debounced binary trust transition with no covariance
transient.

Mechanism of harm in the new stack: each strike hard-anchors a still-bouncing foot
($R \to 1\times$ within 90 ms of impact), yanks the base-velocity estimate by ~0.1 m/s, re-seeds,
de-anchors mid-transfer, and re-seeds again — a touchdown-synchronous impulse train into any
controller consuming the estimate (and a fresh per-step kick for the separately-identified 12 Hz
delayed-velocity-feedback loop; see `.claude-reports/2026-07-16-anchor-oscillation.md`).

---

## 5. The 2026-07-17 fix: a debounced Schmitt trigger in the provider

The fix ports layers 1–2 of the old armor into `FootSwitchContactProbabilityProvider`, as a
per-foot two-state machine ahead of the (unchanged) quantizer + EMA. Let $s$ = filtered switch
boolean, $\ell$ = `getFootLoadPercentage()` (fraction of robot weight):

$$\text{enter} \;=\; s \wedge (\ell \ge 0.35), \qquad
\text{stay} \;=\; s \wedge (\ell \ge 0.25),$$

$$\text{trusted} \leftarrow \begin{cases}
\text{true} & \text{if } \neg\text{trusted} \wedge \text{enter held for } T_d \text{ consecutive ticks} \\
\text{false} & \text{if } \text{trusted} \wedge \neg\text{stay held for } T_d \text{ consecutive ticks} \\
\text{unchanged} & \text{otherwise,}
\end{cases}$$

with dwell $T_d = 40$ ms. Output: trusted → $p_{\text{raw}} = 1$; else sensitive → 0.5; else 0;
EMA as before. Trust starts **true** (init anchoring preserved). NaN load (a switch with no force
estimate) passes both load tests — graceful degradation to switch-only trust.

Parameter provenance — nothing new was invented:

| parameter | value | source |
|---|---|---|
| enter threshold | 0.35 | old `forceInPercentOfWeightThresholdToTrustFoot` (Alex) |
| stay threshold | 0.25 | old `...ToNotTrustFoot` (Alex) |
| dwell $T_d$ | 40 ms | old `delayTimeBeforeTrustingFoot` (20 ms), doubled because the measured impact blips hold the enter condition for ≤ 30 ms while genuine load acceptance holds it for the rest of stance; still ≪ swing duration, so the cost is ≤ 40 ms of delayed anchoring against soft-inflation R anyway |

Two structural consequences:

- The **sensitive-only level approaches 0.5 from below** ($p \to 0.5(1 - \alpha^n) < 0.5$), so an
  untrusted-but-touching foot leans on the estimator's soft R-inflation
  ($c^{0.5} \approx 9.5\times$) without ever tripping the 0.5 hard gates. The quantizer levels
  and hard-gate thresholds now compose correctly: only *debounced trust* crosses them.
- Per physical strike there is now **at most one rising 0.5-crossing** (at genuine load
  acceptance), so R-inflation makes one monotone ramp per step and the re-seed can fire at most
  once per trust rise.

### 5.1 Re-seed re-arm dwell (`TouchdownReseedLatch`)

Defense in depth for the second re-seed: re-arming now requires $p < 0.1$ **sustained for
100 ms** (a genuine swing), not a single tick. A mid-strike dropout shorter than the dwell can no
longer re-arm the latch, so one physical touchdown fires at most one re-seed even if a future
provider re-admits fast trust drops. Extracted into a plain state-machine class precisely so the
property is unit-testable.

### 5.2 Live A/B: the `invariantContactTrustMode` YoEnum

The provider carries a live-switchable mode (no rebuild, settable from SCS2 or per-replay):

| mode | behavior |
|---|---|
| `SCHMITT` (default) | the debounced Schmitt trigger of §5 |
| `LEGACY` | the pre-2026-07-17 mapping — raw filtered boolean through the quantizer + EMA |
| `NONE` | detection off: both feet pinned $p = 1$ exactly (no EMA lag) — DRC-style permanent foothold trust, the control arm |

The Schmitt state machine advances every tick in *all* modes, so switching mid-run resumes from
current debounce state, never stale trust. `AlexEstimatorLogReplay` exposes it as
`--contact-trust schmitt|legacy|none` — baseline replays no longer require stashing the fix.

### 5.3 What the fix does *not* solve — and the honest caveat

The JacobianT force estimate still says "no contact" through much of true stance (the 600 ms
mid-transfer dropout survives any provider-level debounce longer than reasonable). During those
windows the filter anchors on the other foot at $9.5\times$–$90\times$ inflation on the dropped
one — the same graceful degradation the old estimator had (it untrusted the foot outright), minus
the chatter. The *sensor-level* fix is normalizing per-foot load by the **total** measured
vertical force $\sum_j F_{z,j}$ (as the old load percentage did) and/or fusing the CoP position
into the force estimate so the ankle-torque null is filled; that is future work at the
foot-switch level, not the provider.

---

## 6. Property tests (where each claim is pinned)

| property | test |
|---|---|
| impact blip (enter-condition < dwell) never trusts; sensitive-only $p < 0.5$ strictly | `FootSwitchContactProbabilityProviderTest.impactBlipIsRejectedAndRealLoadingTrustsOnce` |
| one physical strike → exactly one trust rise | idem |
| load chatter inside $[0.25, 0.35)$ cannot toggle trust | `...loadChatterInsideSchmittBandCannotToggleTrust` |
| sub-dwell switch bursts never trust | `...subDwellSwitchChatterNeverTrusts` |
| NaN load degrades to switch-only trust | `...nanLoadDegradesToSwitchOnlyTrust` |
| $p \in [0,1]$ under arbitrary chatter | `...probabilityStaysInUnitIntervalUnderRandomChatter` |
| init: trusted, $p = 1$ | `...startsTrustedWithFeetPlanted` |
| ≤ 1 re-seed per sustained-low episode (incl. under random chatter) | `TouchdownReseedLatchTest` (all) |
| mid-strike dropout cannot double-fire the re-seed | `...midStrikeDropoutCannotDoubleFire` |
| re-seed covariance PSD / zero-release (pre-existing) | `InvariantEKFReseedTest` |

Replay validation (open-loop A/B on the logged sensors, `AlexEstimatorLogReplay`, window
2290–2360 s): see `.claude-reports/2026-07-17-contact-detection-old-vs-new.md`.
