# Leaf Node Execution API

This document describes how leaf nodes are executed today. It covers the shared API, how `BehaviorTreeRootNodeExecutor` orchestrates execution, and how each node type implements acceptance and completion logic.

The goal is to make the current system legible for **behavior generation**: assembling sequences and fallbacks from leaf node input and output criteria to reach a higher level goal. That framing follows sequential composition (Burridge et al., 1999), as discussed in the thesis and TRO paper on resilient behaviors.

## Background

In sequential composition, each primitive action has:

- An **acceptance condition** (can it start from the current state?)
- **Success** and **failure** conditions (when is it done, and how?)
- A **pre-authored ordering** of primitives

The thesis proposal maps this directly onto our system. Acceptance maps to `canExecute`. Success and failure map to how `isExecuting` and `failed` are cleared during execution. Ordering is the depth-first leaf sequence plus `executeAfter` concurrency edges.

Object-centric action definition (acting relative to perceived scene frames rather than accumulated robot pose) is the main mechanism for making sequential composition robust to state drift. That is separate from the execution API, but it shapes what `canExecute` checks in practice.

Our runtime is **not** a standard behavior tree. It keeps a persistent **next execution index** between ticks. That is more like a state machine than a tree that restarts from the root each tick. Control nodes (fallback, goto) modify that index rather than returning SUCCESS, FAILURE, or RUNNING.

## Architecture Overview

Each node has two parallel class hierarchies:

| Layer | Role |
|-------|------|
| **State** (`*NodeState`) | CRDT-synced data, frame updates, ROS2 messages |
| **Executor** (`*NodeExecutor`) | Robot-side logic: preconditions, commanding, completion |

Leaf nodes share:

- `LeafNodeDefinition` — includes `executeAfter` for concurrency
- `LeafNodeState` — execution status booleans
- `LeafNodeExecutor` — trigger and update hooks

`BehaviorTreeExecutor.update()` runs each tick:

1. `clock()` — mark all nodes inactive
2. `tick()` — mark active path
3. Recursive `update()` on every node (state maintenance)
4. `BehaviorTreeRootNodeExecutor.update()` — sequence orchestration

## Leaf Execution State (`LeafNodeState`)

Four CRDT-synced booleans form the leaf execution API:

| Field | Set by | Meaning |
|-------|--------|---------|
| `canExecute` | Node `update()` | Acceptance condition. Checked immediately before trigger. Default `true`. |
| `isNextForExecution` | Root executor | Leaf is in the current concurrent group and eligible for manual stepping. |
| `isExecuting` | `triggerExecution()` sets `true`; only `updateCurrentlyExecuting()` should clear it | Action is in progress. |
| `failed` | `updateCurrentlyExecuting()` or `triggerExecution()` | Action ended in failure. |

Other important fields:

- `leafIndex` — depth-first index over all leaves (0-based). Defines the global sequence order.
- `executeAfter` (on definition) — which prior leaf must finish (or not be blocking) before this one can start. Resolved via `getExecuteAfterLeafIndex()`.

`getExecuteAfterLeafIndex()` supports three modes:

- **Previous** (`leafIndex - 1`) — default sequential behavior
- **Beginning** (`-1`) — can start as soon as execution reaches this index
- **Named node** — explicit dependency, including non-leaf targets (resolved to the prior leaf)

When concurrency is disabled at the root, every leaf effectively executes after the previous one regardless of `executeAfter`.

## Leaf Execution Hooks (`LeafNodeExecutor`)

```java
// Default trigger — subclasses call super and then command the robot
public void triggerExecution() {
    state.setIsExecuting(true);
    state.setFailed(false);
}

// Called every tick only while isExecuting
public void updateCurrentlyExecuting() { }

// Called every tick via BehaviorTreeNode.update() default → state.update(),
// then executor override for precondition evaluation
public void update() { getState().update(); }  // interface default; executors override
```

`cantExecuteMessage` is a human-readable explanation when `canExecute` is false. The root executor logs it on blocked execution.

**Contract:**

1. `update()` — evaluate `canExecute` and refresh previews. No side effects that assume execution has started.
2. `triggerExecution()` — called once when the root executor decides to run this leaf. Start commanding.
3. `updateCurrentlyExecuting()` — poll completion. Clear `isExecuting`. Set `failed` when appropriate.

Only `updateCurrentlyExecuting()` should clear `isExecuting` after a trigger. Goto and checkpoint are exceptions that complete in one tick.

## Root Orchestration (`BehaviorTreeRootNodeExecutor`)

The root executor is the sequence engine. Each tick it:

### 1. Rebuild leaf lists

Depth-first traversal collects `orderedLeaves`, `orderedActions`, `fallbackNodes`, and leaves currently marked `isExecuting`.

### 2. Compute concurrent groups

For each leaf `i`:

```
after = effectiveExecuteAfterLeafIndex(leaf)
isNextForExecution = (i >= executionNextIndex) && (after < executionNextIndex)
```

`effectiveExecuteAfterLeafIndex` respects `executeAfter` when concurrency is enabled. Catch leaves inside a fallback are forced to wait until all try leaves in that fallback have passed.

### 3. Update executing leaves

For each currently executing leaf, call `updateCurrentlyExecuting()`. If `isExecuting` becomes false, call `leafCeasedExecution()`:

- Success → `successfulLeaves`
- Failure → `failedLeaves`, disable automatic execution, stay at failed index for debug/retry
- Fallback try leaf → may jump `executionNextIndex` to catch or past catch

### 4. Execution loop

While automatic execution is on (or a manual step was requested):

```
if end of sequence → stop
leaf = orderedLeaves[executionNextIndex]

if manual mode and leaf not isNextForExecution → stop
if fallback blocks this leaf → stop
if execute-after leaf still executing → stop

leaf.update()  // refresh canExecute
if canExecute:
    leaf.triggerExecution()
    unless GotoNode: executionNextIndex++
    if not isExecuting after trigger → handle immediate completion
else:
    log cantExecuteMessage, stop
```

Key root state (`BehaviorTreeRootNodeState`):

- `executionNextIndex` — pointer to next leaf to attempt
- `automaticExecution` — run the loop each tick
- `concurrencyEnabled` — honor `executeAfter` edges
- `manualExecutionRequested` — operator single-step

## Mapping to Sequential Composition

| Burridge / thesis concept | Implementation |
|---------------------------|----------------|
| Acceptance condition | `getCanExecute()`, set in `update()` |
| Start action | `triggerExecution()` |
| Action running | `getIsExecuting() == true` |
| Success | `isExecuting` cleared, `failed == false` |
| Failure | `isExecuting` cleared, `failed == true` |
| Sequence ordering | Depth-first `leafIndex` + `executionNextIndex` |
| Overlapping actions | `executeAfter` pointing earlier than previous leaf |
| Alternative strategy | `FallbackNodeExecutor` try/catch jump |
| Non-local jump | `GotoNodeExecutor` sets `executionNextIndex` |

**What is not explicit today:** there is no shared interface declaring input or output criteria per leaf. Preconditions and completion logic live inside each executor. A planner would need to extract or wrap those checks.

## Node-Specific Implementations

### Control leaves

| Node | `canExecute` | Trigger | Completion |
|------|-------------|---------|------------|
| **Goto** | Default `true` | Sets `executionNextIndex` to target or steps forward. Does not set `isExecuting`. | `updateCurrentlyExecuting()` clears `isExecuting` immediately. Does not advance index itself (special case in root loop). |
| **Checkpoint** | Default `true` | Base trigger only | `updateCurrentlyExecuting()` clears `isExecuting` immediately. Named landmark, no motion. |
| **Condition** | Delegates to sub-executor | Sub-type logic in `triggerExecution()` | Sub-type logic in `updateCurrentlyExecuting()` |

**Condition sub-types:**

| Type | Acceptance (`canExecute`) | Success | Failure |
|------|--------------------------|---------|---------|
| `ALWAYS_SUCCEED` | Default | Immediate complete | — |
| `ALWAYS_FAIL` | Default | — | Sets `failed` in trigger |
| `COUNTER` | Default | When count reaches target | Increments count, sets `failed` until target reached |
| `PROXIMITY` | Both frames exist in scene | Distance in range | Timeout or missing frames |
| `SHAPE_CONTAINS` | Valid shape frame (+ frame/points per mode) | Containment check passes | Check fails |

### Action leaves

| Node | Acceptance (`canExecute`) | Success | Failure |
|------|--------------------------|---------|---------|
| **Wait** | Default `true` | Timer elapsed | — |
| **Walk** | Parent frame in scene, goal frame child of world, stance ≠ focal, planners ready | Footsteps done, not walking, within tolerances | Planning failed, time limit, tolerances |
| **Arm** | Jointspace: always. Taskspace: palm frame child of world | Tracking within tolerances / screw primitive done | Time limit, IK/tracking errors |
| **Leg** | Foot frame child of world | Trajectory tracking complete | Time limit |
| **Pelvis** | Pelvis frame child of world | Pose tracking complete | Time limit |
| **Spine** | Jointspace-only, or chest frame child of world | Tracking complete | Time limit, invalid definition |
| **Neck** | Always `true` | Trajectory complete | Time limit |
| **EZ Gripper** | Knuckle joints present, calibrated, not needing reset | Knuckle angles within tolerance | Time limit |
| **Ability Hand** | Default `true` (connection checked at trigger) | Per-joint tolerance or wait-only | Timeout, not connected at trigger |
| **Scene** | Default `true` | Per action type (instant or async setup) | Freeze with no matched object |
| **Mimic** | Default `true` | Replay finished | Replay or transition failure |
| **Hand Wrench** | Default `true` | **Not implemented** — no `updateCurrentlyExecuting()` | Stays executing after publish |

### Structural nodes (not leaves)

**Fallback** splits children into try leaves (`executeAfter` before first child leaf) and catch leaves (everything else):

- Try failure → jump to first catch leaf index
- Last try success → jump past all catch leaves
- Catch leaves blocked while any try leaf is executing

## Common Acceptance Patterns

Most motion actions gate on **reference frame validity**:

```
frame.isChildOfWorld()   // or scene.containsFrame(parentFrameName)
```

This encodes "the object-centric parent frame is perceived and attached to the world tree." That is the practical acceptance condition for object-relative actions.

Other recurring checks:

- Hardware readiness (gripper calibrated)
- Async initialization (footstep planners, CUDA shape counter, ZED grab thread)
- Definition validity (stance ≠ focal for walk, screw frame valid)
- Jointspace modes often skip frame checks (`canExecute = true`)

Walk and EZ Gripper are the main nodes that populate `cantExecuteMessage` with detail.

## Common Completion Patterns

**Timer-based:** Wait, Ability Hand (timeout path)

**Trajectory tracking:** Arm, Leg, Pelvis, Spine, Neck — `TrajectoryTrackingErrorCalculator` with position/orientation or joint tolerances and time limits

**Controller status:** Walk — footstep tracker incomplete count, `controllerStatusTracker.isWalking()`

**Instant complete:** Goto, Checkpoint, some Scene actions, Condition counter increments

**State machine:** Walk — `WalkActionExecutionState` (planning → command → progress)

## Implications for Behavior Generation

A planner that assembles sequences from leaf criteria would need:

### 1. Extract acceptance predicates

Today these are imperative code in each `update()`. Candidates for declarative form:

- Required scene frames (name, child-of-world)
- Robot hardware state (gripper calibrated)
- Spatial relations (proximity condition already close to this)
- Definition validity constraints

### 2. Extract postconditions

Success clears `isExecuting` without `failed`. Postconditions are implicit:

- Frame reaches goal within tolerance (walk, arm, pelvis)
- Boolean scene check (condition nodes)
- Scene object created/frozen (scene action)

There is no uniform "output state" object. Effects are scattered across scene state, robot controller state, and node state fields.

### 3. Sequence feasibility

The planner must respect:

- Global `leafIndex` ordering as the default backbone
- `executeAfter` DAG for concurrency (when enabled)
- Fallback try/catch structure (catch only reachable on try failure)
- Goto targets (non-local jumps)

### 4. Gaps to close for planning

| Gap | Notes |
|-----|-------|
| No shared precondition/postcondition API | Each executor is ad hoc |
| `canExecute` defaults to `true` | Several nodes never set it |
| Incomplete executors | `HandWrenchActionExecutor` never completes |
| Object-centric outputs | Success often means "frame now at X relative to object Y" but that is not typed |
| High-level control nodes | Door Traversal, Building Exploration use hand-coded dispatch, not leaf criteria |
| Counter condition | Failure-until-count semantics are unusual for a planner |

A practical first step: define a small interface or annotation layer on top of existing executors that declares required frames, produced frames, and completion predicates without changing runtime behavior yet.

## Key Source Files

| File | Role |
|------|------|
| `LeafNodeState.java` | Execution status API |
| `LeafNodeExecutor.java` | Trigger and update hooks |
| `LeafNodeDefinition.java` | `executeAfter` concurrency |
| `BehaviorTreeRootNodeExecutor.java` | Sequence orchestration |
| `BehaviorTreeRootNodeState.java` | `executionNextIndex`, concurrency flag |
| `FallbackNodeExecutor.java` | Try/catch jump logic |
| `GotoNodeExecutor.java` | Index redirect |
| `ConditionNodeExecutor.java` | Condition dispatch |
| `action/actions/*Executor.java` | Per-action logic |

## Related Docs

- `HowToCreateBehaviorTreeNodes.md` — how to add new node types
- Thesis § Behavior Tree Structure, Object-Centric Action Definition (`phd-thesis-calvert/5_current_architecture.tex`)
- TRO resilient behaviors paper § architecture (`tro-resilient-behaviors/3_architecture.tex`)
