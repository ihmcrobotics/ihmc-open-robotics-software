# How to Create New Behavior Tree Nodes

These steps are in the rough order you should follow when adding a new node type.

Use `WaitAction` as a simple reference implementation when in doubt:

- Messages: `ihmc-interfaces/src/main/messages/ihmc_interfaces/behavior_msgs/msg/WaitAction*Message.msg`
- Core classes: `ihmc-high-level-behaviors/src/main/java/us/ihmc/behaviors/behaviorTree/action/actions/WaitAction*.java`
- RDX UI: `ihmc-high-level-behaviors/src/libgdx/java/us/ihmc/rdx/behaviorTree/actions/RDXWaitAction.java`

## 1. ROS2 message classes

Create node-specific messages in `ihmc-interfaces/src/main/messages/ihmc_interfaces/behavior_msgs/msg/`:

- `MyNodeDefinitionMessage.msg`
- `MyNodeStateMessage.msg`

Follow the existing pattern for your node category:

- **Actions** embed `behavior_msgs/ActionNodeDefinitionMessage definition` and `behavior_msgs/ActionNodeStateMessage state`.
- **Condition nodes** embed `behavior_msgs/LeafNodeDefinitionMessage definition` and `behavior_msgs/LeafNodeStateMessage state`.
- **Goto nodes** embed the leaf node definition/state messages via `GotoNodeDefinitionMessage` / `GotoNodeStateMessage`.
- **Control nodes** (action sequence, fallback, checkpoint, etc.) embed `BehaviorTreeNodeDefinitionMessage` / `BehaviorTreeNodeStateMessage`.

Register the node in `BehaviorTreeStateMessage.msg`:

1. Add a new `byte MY_NODE = N` constant (use the next available number).
2. Add a typed list field, e.g. `behavior_msgs/MyNodeStateMessage[<=120] my_nodes`.

Build the messages from `ihmc-interfaces`:

```bash
cd ihmc-interfaces
gradle generateMessages
```

## 2. Definition and state Java classes

Create node-specific classes under `ihmc-high-level-behaviors/src/main/java/us/ihmc/behaviors/behaviorTree/`:

- `MyNodeDefinition.java`
- `MyNodeState.java`

Pick the correct base class:

| Node kind | Definition base | State base |
|-----------|-----------------|------------|
| Action | `ActionNodeDefinition` | `ActionNodeState` |
| Condition node | `ConditionNodeDefinition` | `ConditionNodeState` |
| Goto node | `GotoNodeDefinition` | `GotoNodeState` |
| Control node (sequence, fallback, checkpoint, etc.) | extend `BehaviorTreeNodeDefinition` | extend `BehaviorTreeNodeState` |

Each class should implement the usual CRDT, JSON (`saveToFile` / `loadFromFile`), and ROS2 message (`toMessage` / `fromMessage`) methods. Copy an existing node of the same kind as a template.

Register the node in these registry classes:

- `BehaviorTreeDefinitionRegistry.java`
- `BehaviorTreeDefinitionBuilder.java`
- `ROS2BehaviorTreeSubscriptionNode.java`
- `ROS2BehaviorTreeMessageTools.java`

## 3. Executor and RDX UI classes

Create node-specific classes:

- `MyNodeExecutor.java` in `src/main/java/...`
- `RDXMyNode.java` in `src/libgdx/java/us/ihmc/rdx/behaviorTree/`

Register the node in:

- `BehaviorTreeExecutorNodeBuilder.java`
- `RDXBehaviorTreeNodeBuilder.java`
- `RDXBehaviorTreeNodeCreationMenu.java`

Optional, depending on the node:

- `ActionNodeInitialization.java` — add a branch if the action should be initialized with smart defaults when inserted in the UI (e.g. arm/leg actions copying the previous pose).
- `RDXActionProgressWidgets.java` — add rendering only if the action needs custom execution progress plots (walk, arm, gripper, etc.). Simple actions like Wait do not need changes here.
- `RDXActionProgressWidgetsManager.java` — update only if the manager needs to know about a new action category for grouped progress display.
- `RDXBehaviorTreeSVGNode.java` — add support if the node should appear correctly in SVG export.

## 4. JSON schema changes

Behavior trees are saved as JSON with a `"type"` field set to the definition class simple name (e.g. `"WaitActionDefinition"`). When you add node types or rename/refactor JSON fields, run the behavior tree JSON sanitizer to reload and resave all behavior tree files so existing trees stay loadable.

The sanitizer lives in `BehaviorTreeJSONSanitizer`. Each robot project provides a small `main` wrapper that passes its robot model, for example:

- `AlexBehaviorTreeJSONSanitizer` in the Alex project
- `H1BehaviorTreeJSONSanitizer` in the Unitree H1 project

Run the sanitizer for the robot whose `behaviorTrees` resource directory you want to update.
