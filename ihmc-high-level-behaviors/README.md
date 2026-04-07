# How to Create New Behavior Tree Nodes

This checklist was validated against the current code in this repo (April 7, 2026).
It is written for adding a new concrete node type (for example, a new action like `WaitAction`).

## 1. Add ROS2 message definitions

Create node-specific messages in:

- `ihmc-interfaces/src/main/messages/ihmc_interfaces/behavior_msgs/msg/MyNodeDefinitionMessage.msg`
- `ihmc-interfaces/src/main/messages/ihmc_interfaces/behavior_msgs/msg/MyNodeStateMessage.msg`

Then register the new node type in:

- `ihmc-interfaces/src/main/messages/ihmc_interfaces/behavior_msgs/msg/BehaviorTreeStateMessage.msg`

What to add in `BehaviorTreeStateMessage.msg`:

- A new type byte constant (for example `byte MY_NODE = <next value>`)
- A typed state list (for example `behavior_msgs/MyNodeStateMessage[<=120] my_nodes`)

## 2. Regenerate generated message code

From `ihmc-open-robotics-software/` run:

```bash
./gradlew :ihmc-interfaces:generateMessages
```

## 3. Create behavior-tree definition/state classes

Create node classes in `ihmc-high-level-behaviors`:

- `MyNodeDefinition.java`
- `MyNodeState.java`

For action nodes, these typically extend `ActionNodeDefinition` and `ActionNodeState`.

## 4. Register definition type mappings

Add the node to:

- `BehaviorTreeDefinitionRegistry.java` (definition class <-> message byte mapping)
- `BehaviorTreeDefinitionBuilder.java` (definition class -> constructor mapping)

## 5. Wire ROS2 tree message packing/unpacking

Add the node to:

- `ROS2BehaviorTreeSubscriptionNode.java`
- `ROS2BehaviorTreeMessageTools.java`

In `ROS2BehaviorTreeMessageTools`, update all relevant sections:

- `clearLists(...)`
- `packMessage(...)`
- `fromMessage(...)`
- `packSubscriptionNode(...)`

## 6. Create executor and RDX classes

Create node-specific runtime/UI classes:

- `MyNodeExecutor.java`
- `RDXMyNode.java`

Register them in:

- `BehaviorTreeExecutorNodeBuilder.java`
- `RDXBehaviorTreeNodeBuilder.java`
- `RDXBehaviorTreeNodeCreationMenu.java`

## 7. Action-node-specific integrations

If your node is an action, also check:

- `ActionNodeInitialization.java` (default side/frame/pose initialization if needed)
- `RDXActionProgressWidgetsManager.java` (if it needs custom progress-category rendering)
- `RDXActionProgressWidgets.java` (if it needs new custom progress plots/metrics)

## 8. Optional integrations

Depending on feature needs, you may also want:

- `BehaviorTreeSVGNode.java` (custom type label and duration handling in SVG export)

## 9. After JSON schema/type changes

If you change JSON schema details (type names, fields, renames), run the sanitizer:

- `BehaviorTreeJSONSanitizer` (current class name)

`NadiaBehaviorTreeJSONSanitizer` is not the current class name in this repo.
