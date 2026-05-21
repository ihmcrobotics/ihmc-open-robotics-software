# jros2 Migration Notes

Reference doc for the ihmc-ros2-library → jros2 migration on branch `jros2-conversion-2`.
Treat as a guide, not a journal — patterns to apply, decisions to remember, work that's deferred.

---

## Current Status

Last verified: **May 2026** on branch `jros2-conversion-2` (includes `develop` merge).

| Check | Result |
| ----- | ------ |
| `git diff develop...HEAD --stat` | **1391 files**, +73808 / −5851 lines |
| Hand-edited `.java` (excl. `generated-java`, `ihmc-interfaces-jros2/src/main/java`) | **750** files |
| `compositeTask -PtaskName=compileJava` | ✅ BUILD SUCCESSFUL |
| `compositeTask -PtaskName=compileTestJava` | ✅ BUILD SUCCESSFUL |

Hand-edited Java by module (top):

| Module | Files |
| ------ | ----- |
| `ihmc-high-level-behaviors` | 212 |
| `ihmc-avatar-interfaces` | 172 |
| `ihmc-humanoid-robotics` | 104 |
| `ihmc-communication` | 76 |
| `ihmc-common-walking-control-modules` | 56 |
| `ihmc-perception` | 51 |
| `ihmc-footstep-planning` | 48 |
| other (sensor, zulu, manipulation, …) | ≤10 each |

| Module | Status |
| ------ | ------ |
| `ihmc-interfaces-jros2` | New module — jros2-generated types + `EuclidXxxMessage` / `EuclidTransformMessage` wrappers |
| `ihmc-communication` … `zulu` | ✅ all composite `compileJava` targets build |
| `ihmc-avatar-interfaces-test` | ✅ compiles (~85 test files migrated) |
| `ihmc-high-level-behaviors-libgdx` | ✅ compiles (~77 libgdx sources migrated) |

```bash
cd ihmc-open-robotics-software
GRADLE_OPTS="-Xmx4g" ./gradlew compositeTask -PtaskName=compileJava
GRADLE_OPTS="-Xmx4g" ./gradlew compositeTask -PtaskName=compileTestJava
```

### Branch audit (`develop...HEAD`)

**Clean in production `src/main` (grep scan):**

| Anti-pattern | Count in `src/main` |
| ------------ | ------------------- |
| `*.msg.dds.*` imports | 0 (Javadoc-only mentions may remain) |
| `ROS2NodeBuilder` | 0 |
| `takeNextData` | 0 |
| `us.ihmc.ros2` (old package) | 0 |

**Expected / intentional exceptions:**

| Item | Notes |
| ---- | ----- |
| `us.ihmc.pubsub` | 4 files — `ROS2LogIOTools`, `ROS2LogSerialization`, `SCSROS2Visualizer`, `PubSubStatsTools` (legacy log/stats paths) |
| `RDXROS2StatsPanel` + `PubSub*Stats` | Stubbed UI — old pubsub metrics API removed |
| `BigVideoSwapData` | Stubbed (unused legacy) |
| `ihmc-interfaces` (removed) | Legacy DDS pub-sub generator project deleted; **runtime** uses `ihmc-interfaces-jros2` only |

**develop merge (perception):** `UInt16MultiArrayHack` → `Float32MultiArrayHack` for YOLOv8 mask polygons (`IDLFloatSequence`, `Float.BYTES` stride). Both hack types may exist in `ihmc-interfaces-jros2` until fully removed.

### Migration patterns (confirmed on branch)

| Pattern | On branch |
| ------- | --------- |
| Drop `.msg.dds.` in imports | ✅ migrated API modules |
| `prependedWith` / `appendedWith` / `withType` | Topic constants |
| `reader.read()` / `reader.read(buffer)` | Subscriptions |
| `close()` / `destroySubscription` | Node/sub lifecycle |
| `EuclidXxxMessage` accessors | `.getPoint()` / `.getQuaternion()` / `.getPose()` / `.getVector()` |
| `new T(); dest.set(src);` | Message deep copy |
| `ROS2MessageCdrFileTools` | CDR file I/O |
| `MessageTools.copyData(T[], IDLObjectSequence<T>)` | Deep copy sequences |
| `MessageTools.toMessage/toEuclid(RigidBodyTransform, geometry_msgs.Transform)` | TF / plain `geometry_msgs` transforms |
| `ControllerMessageConstants.INVALID_MESSAGE_ID` / `VALID_MESSAGE_DEFAULT_ID` | Replaces `Packet.*` ID constants |
| `AsyncROS2Node` | Realtime comms |
| `ihmc_hands_ros2:source` | Cloned hands repo on jros2 |

**Deferred / stubbed:** `RDXROS2StatsPanel` and pub/sub stats helpers. `ROS2Helper.publish(Pose3D)` and `ROS2PeerClockOffsetEstimator` subscription-matched callbacks remain commented (see Open TODOs).

### ihmc-avatar-interfaces (completed patterns)

- **Root wrench output**: `geometry_msgs/Wrench` uses plain `geometry_msgs/Vector3` for force/torque — set via `setX/setY/setZ`, not `EuclidVector3DMessage.getVector()`.
- **Footstep / status poses**: `FramePose3D(world, msg.getDesiredFootPositionInWorld().getPoint(), msg.getDesiredFootOrientationInWorld().getQuaternion())`.
- **IDL sequences in planning**: iterate `solution.getRobotConfigurations().size()` / `.get(i)` — not `List`.
- **Message copy**: `new T(); dest.set(src);` everywhere (reachability snapshots, rigid-body anchors).
- **Support polygons**: `IDLObjectSequence<EuclidPoint3DMessage>` — `add().getPoint().set(x, y, z)`.
- **Script JSON I/O**: `MultiContactEnvironmentDescription` + snapshot anchors use `ROS2MessageCdrFileTools` (base64 CDR); legacy DDS-JSON field names still accepted on read via `getMessageClassName()`.
- **Script statistics**: selection/weight matrices have no `epsilonEquals` — compare axis flags and weights manually; positions/orientations via `.getPoint()` / `.getQuaternion().getRotationVector(...)`.
- **Kinematics simulation**: `AsyncROS2Node` replaces `ROS2NodeBuilder.buildRealtime`; final `ros2Node` needs single assignment (`configured != null ? configured : new ROS2Node(...)`).
- **ihmc_hands_ros2** (cloned at `repository-group/ihmc_hands_ros2`, depend on `us.ihmc:ihmc_hands_ros2:source`): jros2 `generateMessages` on repo root (`package.xml` + `msg/`). Generated package is `ihmc_hands_ros2` (not `ihmc_hands_ros2.msg.dds`). Topics: `prependedWith("ability_hand")` / `appendedWith("left"|"right")`. Hand comms use `AsyncROS2Node` + `close()` / `spin()`.

For large composite builds, raise the Gradle heap — the default 512 MiB OOMs on full `compositeTask -PtaskName=compileJava`:

```bash
cd ihmc-open-robotics-software
GRADLE_OPTS="-Xmx4g" ./gradlew :ihmc-common-walking-control-modules:compileJava
```

---

## Removing the old `Packet` base class (not message type names)

Three different things were called "Packet" in the old stack:

| Kind | Example | Migration |
| ---- | ------- | --------- |
| **Removed base class** | `us.ihmc.communication.packets.Packet<T>` | Delete inheritance; use `ROS2Message<T>` / `Settable<T>` directly |
| **Message type names** | `TextToSpeechPacket`, `FootstepPlanningRequestPacket` | **Keep** — naming only |
| **Removed network glue** | `PacketConsumer`, `NewMessageListener`, `takeNextData` | `ROS2MessageReader` lambda (see below) |

`Packet.INVALID_MESSAGE_ID` is gone. Use `ControllerMessageConstants.INVALID_MESSAGE_ID` (`-1L`, matches `QueueableMessage` IDL default):

```java
import us.ihmc.communication.controllerAPI.ControllerMessageConstants;
// ...
message.setMessageId(ControllerMessageConstants.INVALID_MESSAGE_ID);
```

Target data flow for controller I/O:

```
DDS → ROS2Message (implements Settable) → reader.read() → CommandInputManager.submitMessage → Command objects
```

`CommandInputManager` already routes on `Settable<?>`; most work is subscribers, status publishers, and geometry/sequence call sites.

### Replacing `PacketConsumer` / `NewMessageListener`

Old pattern used `takeNextData(buffer, null)` with a pre-allocated buffer per message class. jros2 equivalent:

```java
// Heterogeneous message list: topic type is known at subscribe time; use reader.read() return value
@SuppressWarnings({"unchecked", "rawtypes"})
Class messageClassRaw = (Class) messageClass;
ros2Node.createSubscription(ControllerAPI.getTopic(baseTopic, messageClassRaw),
                             reader -> receivedMessage(reader.read()));

// Status publish from Class<? extends Settable<?>> map — raw cast at publish site
@SuppressWarnings({"unchecked", "rawtypes"})
ROS2Publisher publisher = (ROS2Publisher) publisherMap.get(message.getClass());
publisher.publish((ROS2Message) message);
```

`ControllerAPI.getTopic` / `getLowFrequencyTopic` require `Class<T extends ROS2Message<T>>`. When the class comes from `List<Class<? extends Settable<?>>>` or `List<Class<? extends ROS2Message<?>>>`, pass `(Class) messageClassRaw` (with `@SuppressWarnings({"unchecked", "rawtypes"})`).

`ROS2Message.createInstance(Class)` has the same F-bound; prefer `reader.read()` for subscribers, or `commandClass.getDeclaredConstructor().newInstance().getMessageClass()` when building topic lists from `Command` types (catch `InvocationTargetException` on `newInstance()`).

`QueuedROS2Subscription` is not ported — use `AsyncROS2Node.createSubscription` (same reader/callback patterns as `ROS2Node`).

Files deleted: `PacketConsumer.java`, `ROS2TopicList.java` (see Build section).

### `JSONSerializer` / `*PubSubType` file I/O (removed with old library)

`ihmc-pub-sub-serializers-extra` and `*PubSubType` are gone. For saving/loading messages to disk:

```java
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;

byte[] bytes = ROS2MessageCdrFileTools.serializeToBytes(message);
ROS2MessageCdrFileTools.deserializeInto(bytes, message);
```

**Footstep planner logs** (`FootstepPlannerLogger` / `FootstepPlannerLogLoader`) now write/read CDR bytes (filenames still end in `.json` for compatibility). **Legacy JSON logs** (files starting with `{`) throw a clear `IOException` on load — re-record with the current planner.

**Height map datasets** (`HeightMapDataSetName`) used `TerrainMapMessagePubSubType` + `JSONSerializer` on resources that are actually legacy `HeightMapMessage` JSON (`keys` + `heights` sparse arrays). Use `HeightMapDataSetIOTools` in `ihmc-path-planning-data-sets` instead.

### `ROS2Message` copy constructors

Generated jros2 messages only have a no-arg constructor. Deep copy with:

```java
KinematicsToolboxOutputStatus copy = new KinematicsToolboxOutputStatus();
copy.set(original);
```

Not `new KinematicsToolboxOutputStatus(original)`.

### `EuclidPose3DMessage` → `Pose3D` / `RigidBodyTransform`

Wrappers are not `Pose3DReadOnly`:

```java
timeStampedExternalPose.setTransform3D(packet.getPose().getPose());
bodyPathWaypoints.add(new Pose3D(waypointMessage.getPose()));
```

---

## Open TODOs in the source

Marked `// TODO jros2` (or `@Disabled // TODO: jros2 migration`) in code.

| File | Issue |
| ---- | ----- |
| `ROS2Helper.java` | `publish(ROS2Topic<Pose3D>, Pose3D)` commented — needs `Pose3DMessageWrapper` or use `EuclidPose3DMessage` topic |
| `ROS2PublisherMap.java` | Same Pose3D publish overload |
| `ROS2PeerClockOffsetEstimator.java` | `SampleInfo` + subscription-matched callback not in jros2 yet |
| `Float32MultiArrayHack.java` / `UInt16MultiArrayHack.java` | Generated TODO to remove hack types when no longer needed |
| `YOLOv8AnnotationInfoMessage.java` | Generator comment to swap IDL block when stable |

**Fixed in audit (May 2026):**

| File | Fix |
| ---- | --- |
| `MessageTools.java` | Typed `copyData` for `ROS2Message[]`, `List`, `Pose3D[]`, `Point3D[]`; `toMessage`/`toEuclid` for `geometry_msgs.Transform` |
| `HumanoidMessageTools.java` | Wrench + foot polygon conversions |
| `ROS2Helper.java` | `subscribe()` → `ROS2Input` **active** (not commented) |
| `ROS2FrameTools.java` | TF transform packing via `MessageTools.toMessage` |
| `ROS2MutableFrame.java` | Remote TF apply via `MessageTools.toEuclid` |
| `CRDTStatusFootstepList.java` | `getPoseReadOnly` → `getSolePose().getPose()` |
| `ROS2HumanoidFrames.java` | Javadoc: `tf2_msgs.TFMessage` (not `.msg.dds`) |

Disabled tests (`@Disabled // TODO: jros2 migration`):

| Test | Blocker |
| ---- | ------- |
| `RealtimeROS2PublisherSubscriberTest` | `QueuedROS2Subscription` not porting |
| `FrameRealtimeROS2PublisherSubscriberTest` | same |
| `ROS2PeerClockOffsetEstimatorTest` | `ROS2NodeBuilder` transport modes |
| `ROS2LogTest` | no `epsilonEquals` on `ROS2Message` |
| `MessageToolsTest` | old `Packet` serialize round-trip |
| `ROS2ToolsTest` (2 methods) | intraprocess / `ROS2NodeBuilder` |

---

## Style: no fully-qualified names

When writing or fixing code, **import the package — never use fully-qualified names inline**. Spot-check new edits; the 750 hand-edited files were migrated with this rule but not re-verified line-by-line in this audit.

**Gotcha learned the hard way**: do not use `Edit replace_all` to swap `us.ihmc.jros2.ROS2Message` → `ROS2Message`. It will also rewrite the `import us.ihmc.jros2.ROS2Message;` line into the broken `import ROS2Message;`. Add the import first as a stable, fully-qualified line, then replace_all the FQN, then `sed -i 's|^import ROS2Message;|import us.ihmc.jros2.ROS2Message;|'` over the affected files to restore the imports. Or just edit each file individually.

---

## Build / dependency changes

`ihmc-communication/build.gradle.kts`:

```kotlin
// REMOVED
api("us.ihmc:ros2-library:1.2.5")
api("us.ihmc:ihmc-pub-sub-serializers-extra:1.2.5")
api("us.ihmc:ihmc-interfaces:source")
testApi("us.ihmc:ros2-library-test:1.2.5")

// ADDED
api("us.ihmc:jros2:source")
api("us.ihmc:ihmc-interfaces-jros2:source")
```

`ihmc-interfaces-jros2/gradle.properties` sets `compositeSearchHeight = 2` so jros2 resolves from the parent directory.

Files deleted in the migration:
- `ihmc-communication/.../ROS2TopicList.java`
- `ihmc-communication/.../net/PacketConsumer.java`

---

## Import changes (drop-in renames)

| Old                                                    | New                                                        |
| ------------------------------------------------------ | ---------------------------------------------------------- |
| `controller_msgs.msg.dds.X`                            | `controller_msgs.X`                                        |
| `perception_msgs.msg.dds.X`                            | `perception_msgs.X`                                        |
| `toolbox_msgs.msg.dds.X`                               | `toolbox_msgs.X`                                           |
| `ihmc_common_msgs.msg.dds.X`                           | `ihmc_common_msgs.X`                                       |
| `std_msgs.msg.dds.X`                                   | `std_msgs.X`                                               |
| `std_msgs.msg.dds.String`                              | `std_msgs.String_` (underscore — `String` is reserved)     |
| `geometry_msgs.msg.dds.X`                              | `geometry_msgs.X`                                          |
| `sensor_msgs.msg.dds.X`                                | `sensor_msgs.X`                                            |
| `tf2_msgs.msg.dds.X`                                   | `tf2_msgs.X`                                               |
| `builtin_interfaces.msg.dds.Time`                      | `builtin_interfaces.Time`                                  |
| `us.ihmc.ros2.ROS2Node`                                | `us.ihmc.jros2.ROS2Node`                                   |
| `us.ihmc.ros2.RealtimeROS2Node`                        | `us.ihmc.jros2.AsyncROS2Node`                              |
| `us.ihmc.ros2.ROS2Topic`                               | `us.ihmc.jros2.ROS2Topic`                                  |
| `us.ihmc.ros2.ROS2QosProfile`                          | `us.ihmc.jros2.ROS2QoSProfile` (capital S)                 |
| `us.ihmc.ros2.ROS2Publisher`                           | `us.ihmc.jros2.ROS2Publisher`                              |
| `us.ihmc.ros2.ROS2Subscription`                        | `us.ihmc.jros2.ROS2Subscription`                           |
| `us.ihmc.pubsub.common.Guid`                           | `us.ihmc.jros2.Guid`                                       |
| `us.ihmc.idl.IDLSequence.Integer`                      | `us.ihmc.fastddsjava.cdr.idl.IDLIntSequence`               |
| `us.ihmc.idl.IDLSequence.Float`                        | `us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence`             |
| `us.ihmc.idl.IDLSequence.StringBuilderHolder`          | `us.ihmc.fastddsjava.cdr.idl.IDLStringSequence`            |
| `us.ihmc.idl.IDLSequence.Object<T>` / `us.ihmc.idl.IDLObjectSequence` | `us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<T>` |
| `us.ihmc.commons.lists.RecyclingArrayList<T>`          | `us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<T>` (hot path: `add().set`) |

Imports that are gone entirely (no replacement; the call site must be rewritten):
- `us.ihmc.ros2.ROS2NodeBuilder` — use `ROS2Node` / `AsyncROS2Node` constructors
- `us.ihmc.ros2.ROS2Input` — `us.ihmc.communication.ROS2Input` (uses `createSubscription` + `reader.read()`); use `ROS2Helper.subscribe(topic)` or `new ROS2Input<>(ros2Node, topic)`
- `us.ihmc.ros2.ROS2TopicNameTools` — use `ROS2Message.createInstance(topic.getType())`
- `us.ihmc.ros2.QueuedROS2Subscription` — **not porting**; use `AsyncROS2Node.createSubscription` for realtime paths
- `us.ihmc.pubsub.subscriber.Subscriber` — use `ROS2MessageReader<T>` in callback signature
- `us.ihmc.pubsub.TopicDataType` — no equivalent; subscribe via `ROS2Topic<T>` directly
- `us.ihmc.pubsub.common.SampleInfo` — not yet ported; comment out

---

## Pattern reference

### Node construction and lifecycle

```java
// Construction
new ROS2NodeBuilder().build(name)         → new ROS2Node(name)
new ROS2NodeBuilder().domainId(X).build(n) → new ROS2Node(name, X)
new ROS2NodeBuilder().buildRealtime(name) → new AsyncROS2Node(name)

// Lifecycle
ros2Node.destroy()                        → ros2Node.close()
subscription.remove()                     → ros2Node.destroySubscription(subscription)
publisher.remove()                        → ros2Node.destroyPublisher(publisher)
ros2Node.spin()                           → removed; use app-level wait (e.g. `ROS2Tools.blockUntilInterrupted()`)
```

### Euclid wrapper package (avatar)

Generated geometry fields use **`us.ihmc.euclid.jros2.messages`**, not `ihmc_common_msgs`:

```java
import us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage;
import us.ihmc.euclid.jros2.messages.EuclidVector3DMessage;
// footstepPose.getLocation().getPoint().set(...)
// message.getDesiredRootOrientation().getQuaternion()
```

### Snapshot / toolbox JSON logs

`JSONSerializer` + `*PubSubType` → `ROS2MessageCdrFileTools` with **base64 CDR** in Jackson text nodes (`messageToJsonNode` / `deserializeFromJsonNode`). Legacy DDS JSON objects throw on load with a clear `IOException`.

Message copy: `new T(); copy.set(source)` — no copy constructors. Removed `setDestination(...)` on toolbox output messages.

### Topic builders

```java
// Method renames
topic.withModule(x)                       → topic.appendedWith(x)
topic.withSuffix(x)                       → topic.appendedWith(x)
topic.withRobot(name)                     → topic.appendedWith(name)
topic.withInput()                         → topic.appendedWith("input")
topic.withOutput()                        → topic.appendedWith("output")
topic.withPrefix(x)                       → topic.prependedWith(x)
topic.withTypeName(C.class)               → topic.withType(C.class)
topic.withQoS(qos)                        → REMOVED; pass QoS at pub/sub creation

// Root topic
new ROS2Topic<>().withPrefix("ihmc")      → new ROS2Topic<>("/").prependedWith("ihmc")
```

### QoS

```java
ROS2QosProfile.RELIABLE()                 → ROS2QoSProfile.RELIABLE      // now a constant
ROS2QosProfile.BEST_EFFORT()              → ROS2QoSProfile.BEST_EFFORT   // now a constant
ROS2QosProfile.KEEP_HISTORY(n)            → no equivalent yet (TODO)

// QoS now at pub/sub creation site:
ros2Node.createPublisher(topic, ROS2QoSProfile.BEST_EFFORT);
ros2Node.createSubscription(topic, callback, ROS2QoSProfile.RELIABLE);
```

### Subscription callbacks

**Most common (IHMC):** `ROS2Helper.subscribeViaCallback` — already the “give me `T`” pattern:

```java
ros2Helper.subscribeViaCallback(topic, message -> { /* use message */ });
// equivalent: ros2Node.createSubscription(topic, reader -> callback.accept(reader.read()));
```

**Allocation-free (jros2):** `createSubscriptionSampler` — reuses one message instance per subscription (no `Consumer` overload on `ROS2Node`; use this or `subscribeViaVolatileCallback` / `subscribeViaSwapReference`):

```java
ros2Node.createSubscriptionSampler(topic, sample -> { /* use sample */ });
// internally: reader.read(reusedSample); sampler.consume(reusedSample);
```

**Latest-value holder:** `ROS2Input<T>` or `subscribeViaTypedNotification` → `TypedNotification<T>`.

**Reader-based (three shapes):**

```java
// A) jros2 allocates/returns sample
ros2Node.createSubscription(topic, reader -> {
   T msg = reader.read();
});

// B) Pre-allocated buffer (hot path)
ros2Node.createSubscription(topic, reader -> {
   reader.read(myMessageBuffer);
});

// C) Empty topic
ros2Node.createSubscription(emptyTopic, reader -> doSomething());
```

Old patterns to replace:
```java
subscriber.takeNextData(buffer, null)     → reader.read(buffer)  OR  reader.read()
subscriber.takeNextData(buffer, info)     → no SampleInfo yet — leave a TODO
node.createSubscription2(topic, cb)       → node.createSubscription(topic, reader -> ...)
```

### Generic bounds

`ROS2Topic<T>` requires `T extends ROS2Message<T>`. Any generic method or class accepting a topic or message needs the bound:

```java
// Wrong
public <T> void publish(ROS2Topic<T> topic, T message) { ... }

// Right (import us.ihmc.jros2.ROS2Message; — no FQN)
public <T extends ROS2Message<T>> void publish(ROS2Topic<T> topic, T message) { ... }
```

### Generic message instantiation

```java
// Old
TopicDataType<T> td = ROS2TopicNameTools.newMessageTopicDataTypeInstance(topic.getType());
T data = td.createData();

// New
T data = ROS2Message.createInstance(topic.getType());
```

### Message field renames

```java
// Old library had both uniqueId AND sequenceId.
// jros2 only has sequenceId. The old uniqueId calls were duplicates — delete them.
message.setUniqueId(id);                  → // delete the line entirely
message.getUniqueId();                    → message.getSequenceId();
```

### Message collections and numeric sequences

```java
// MessageCollection.sequences is uint32[] in IDL → IDLIntSequence (not IDLLongSequence)
messageCollectionNotification.setMessageCollectionSequenceId((int) collection.getSequenceId());

// IDLDoubleSequence — no set(index, value); use the buffer
sequence.getBuffer().put(index, value);

// IDLStringSequence
sequence.getAsString(i);                  // not getString(i)

// IDLObjectSequence — no fastRemove
sequence.remove(index);
sequence.capacity()                       → max allocated slots (not getCurrentCapacity())
// Deep copy into sequence
sequence.add().set(value)
// Shallow append (same references)
sequence.addAll(arrayOrCollection)
```

### IDLSequence

```java
// Method renames
sequence.reset()                          → sequence.clear()
sequence.resetQuick() (old ihmc.idl)      → sequence.getBuffer().reset() on byte/float buffers, or sequence.clear() on bool sequences
sequence.getLast()                        → sequence.get(sequence.size() - 1)

// Bulk ops — all implemented in jros2 fastddsjava
IDLIntSequence.addAll(int[])              → WORKS
IDLFloatSequence.addAll(float[])          → WORKS
IDLObjectSequence<T>.addAll(T[])          → WORKS (shallow — adds references)
IDLObjectSequence<T>.addAll(Collection)   → WORKS (shallow)
IDLStringSequence.addAll(...)             → WORKS

// Deep copy vs shallow: addAll does NOT call set() on elements — for ROS2Message deep copy use:
destination.clear();
for (T item : source)
   destination.add().set(item);          // MessageTools.copyData(T[], IDLObjectSequence)
```

### Geometry: Euclid wrapper messages

`ihmc-interfaces-jros2` includes custom IHMC message wrappers in `us.ihmc.euclid.jros2.messages`:

| Wrapper                   | Wraps             | Accessor              |
| ------------------------- | ----------------- | --------------------- |
| `EuclidPoint3DMessage`    | `Point3D`         | `.getPoint()`         |
| `EuclidVector3DMessage`   | `Vector3D`        | `.getVector()`        |
| `EuclidQuaternionMessage` | `Quaternion`      | `.getQuaternion()`    |
| `EuclidPose3DMessage`     | `Pose3D`          | `.getPose()`          |

Each wrapper has `set(...)` overloads for many Euclid interfaces (Point3DReadOnly, Vector3DReadOnly, Vector2DReadOnly, RotationMatrixReadOnly, QuaternionReadOnly, Orientation3DReadOnly, Pose3DReadOnly, Shape3DPoseReadOnly, RigidBodyTransformReadOnly, etc). Two equivalent ways to write into one:

```java
// Via the overload (cleaner)
message.getPosition().set(pointReadOnly);
message.getOrientation().set(rotationMatrix);
message.getPose().set(pose3DReadOnly);

// Via the wrapped Euclid object (gets you the full Euclid API)
message.getPosition().getPoint().set(x, y, z);
message.getOrientation().getQuaternion().interpolate(q1, q2, alpha);
```

Garbage-free packing into a sequence:

```java
for (Point3DReadOnly p : points)
   sequence.add().set(p);
```

The accessor returns the **live** wrapped object — modifications are visible to the message. The wrapper constructors that take an Euclid arg wrap by reference (no copy).

**Common mistakes when copying between Euclid frames and message fields:**

```java
// Wrong — FramePose/FramePoint .set() does not accept EuclidXxxMessage wrappers
framePose.set(footstep.getLocation());
frameQuaternion.set(status.getDesiredEndEffectorOrientation());

// Right — unwrap to the inner Euclid type
framePose.getPosition().set(footstep.getLocation().getPoint());
framePose.getOrientation().set(footstep.getOrientation().getQuaternion());
footstep.getLocation().set(framePose3D.getPosition());           // wrapper overload
footstep.getOrientation().set(framePose3D.getOrientation());

// setToNaN on wrappers — call on inner object
wrapper.getPoint().setToNaN();
wrapper.getQuaternion().setToNaN();

// 2D foot pose → 3D footstep message: use the FramePose3D copy, not FrameOrientation2D on the wrapper
footstep.getLocation().set(nextFootstepPose3D.getPosition());
footstep.getOrientation().set(nextFootstepPose3D.getOrientation());
// yaw-only from FramePose2D: yawPitchRoll.setYaw(footstepPose.getYaw()); wrapper.set(yawPitchRoll);
```

### Geometry: standard geometry_msgs types

Plain `geometry_msgs` types (TF, `TransformStamped`, upstream ROS messages) have no Euclid `set(RigidBodyTransform)` overload. Use field setters or `MessageTools`:

```java
// RigidBodyTransform ↔ geometry_msgs.Transform (TF publishing / subscription)
MessageTools.toMessage(rigidBodyTransform, transformStamped.getTransform());
MessageTools.toEuclid(transformStamped.getTransform(), rigidBodyTransformToPack);

// Field by field for Point / Vector3
msg.setX(p.getX()); msg.setY(p.getY()); msg.setZ(p.getZ());
```

Gotcha: Euclid quaternion scalar is `getS()`; `geometry_msgs.Quaternion` uses `getW()`. `MessageTools.toMessage/toEuclid(Transform, …)` handles this.

For IHMC custom messages, prefer `EuclidTransformMessage` (same wire type as `geometry_msgs/Transform`) where the IDL field uses the wrapper.

### MessageTools.copyData

```java
// Deep copy (implemented):
MessageTools.copyData(T[] source, IDLObjectSequence<T> destination)
MessageTools.copyData(List<T> source, IDLObjectSequence<T> destination)
MessageTools.copyData(Pose3DReadOnly[] source, IDLObjectSequence<EuclidPose3DMessage> destination)
MessageTools.copyData(Point3DReadOnly[] source, IDLObjectSequence<EuclidPoint3DMessage> destination)
MessageTools.copyData(Iterable<? extends Point3DReadOnly> source, IDLObjectSequence<EuclidPoint3DMessage> destination)

// Shallow bulk (references only):
target.clear();
target.addAll(collection);
```

---

## Custom additions for the migration

These are in jros2 itself (we own them; consider upstreaming):

- `us.ihmc.jros2.Guid` — 16-byte holder for DDS-RTPS GUIDs. Used by `ROS2PeerClockOffsetEstimator`. Implements `equals`/`hashCode`.
- `ROS2Message extends Settable<T>` — lets jros2 messages be used wherever IHMC code expects a `Settable<?>` (introduces an `euclid` dep on jros2; see "Architectural decisions" below).
- `IDLObjectSequence.addAll(T[])` / `addAll(Collection)` — shallow bulk append.
- `ROS2Node.createSubscriptionSampler` + `ROS2SubscriptionCallbackSampler` — allocation-free “here is your `T`” callback.
- `ROS2Publisher.getGuid()` / `ROS2Subscription.getGuid()` — expose underlying GUIDs.
- `ROS2QoSProfile.RELIABLE` and `BEST_EFFORT` as constants.
- `tf2_msgs` generated from the `geometry2` humble submodule.

In `ihmc-interfaces-jros2`:
- The four `EuclidXxxMessage` wrappers described above. Each implements `ROS2Message<T>`, has a static `name` matching the geometry_msgs IDL type, and serialises identically to `geometry_msgs/{Point,Vector3,Quaternion,Pose}`. On the wire they are interchangeable with the standard messages.

---

## Architectural decisions worth remembering

- **`ROS2Message` and euclid:** jros2 does **not** depend on euclid. `ROS2Message<T>` declares `void set(T from)` directly (same as generated messages). IHMC controller glue (`Command<M extends ROS2Message<M>>`, `CommandInputManager`, status APIs) uses `ROS2Message` bounds instead of `Settable` for wire types. **Commands** still extend Euclid `Settable<C>` for garbage-free command copies.
- **No `ROS2Node.spin()`:** blocking the caller is an application concern. IHMC uses `ROS2Tools.blockUntilInterrupted()` where a thread must stay alive while DDS callbacks run on listener threads.
- **`getGuid()`:** returns `us.ihmc.jros2.Guid` from publishers and subscriptions (not `byte[]`).
- **Why the Euclid wrapper messages instead of `geometry_msgs.Point` everywhere:** generated messages have plain double fields and no Euclid API. Wiring the wire format to the Euclid types we actually use (Point3D, Quaternion, ...) is what eliminates per-call-site boilerplate. The wrappers serialise byte-identically to the standard messages, so they're interoperable on the wire.
- **Why `MessageTools.toMessage/fromMessage` still exists for plain geometry_msgs:** TF messages (`TFMessage`, `TransformStamped`) and other upstream ROS messages use the standard geometry_msgs types. Those code paths still need explicit conversion.
- **`uniqueId` deletion is intentional, not lossy:** the old library carried `uniqueId` alongside `sequenceId` and code typically set both to the same value. jros2 only keeps `sequenceId`. Where you see `setUniqueId(x)` removed without a replacement, that's because `setSequenceId(x)` is already being called elsewhere in the same block.

---

## Quick error → fix table

| Error                                                                | Fix                                                              |
| -------------------------------------------------------------------- | ---------------------------------------------------------------- |
| `package us.ihmc.ros2 does not exist`                                | Import from `us.ihmc.jros2` (or `us.ihmc.fastddsjava.cdr.idl`)   |
| `cannot find symbol: setUniqueId`                                    | Delete the call — there's already a `setSequenceId` nearby       |
| `cannot find symbol: method takeNextData`                            | Replace subscriber lambda with `reader -> reader.read(...)`      |
| `cannot find symbol: method destroy()` on a node                     | `.close()`                                                       |
| `cannot find symbol: method remove()` on a sub/pub                   | `node.destroySubscription(sub)` / `node.destroyPublisher(pub)`   |
| `cannot find symbol: method withRobot / withModule / withInput`      | `.appendedWith(name)` (or `.appendedWith("input")`)              |
| `cannot find symbol: method withQoS`                                 | Pass QoS to `createPublisher` / `createSubscription`             |
| `cannot find symbol: method reset()` on a sequence                   | `.clear()`                                                       |
| `cannot find symbol: method addAll` on `IDLObjectSequence`           | Use `addAll` (shallow) or `add().set(item)` loop (deep copy)   |
| `cannot find symbol: getCurrentCapacity` on `IDLObjectSequence`    | `.capacity()`                                                    |
| `cannot find symbol: getString` on `IDLStringSequence`             | `getAsString(i)`                                                 |
| `incompatible types: int to short` on `setLogLevel` / `uint16`     | `(short) value`                                                  |
| `incompatible types: long to int` on `setSequenceId`             | `(int) getSequenceId()` where field is `int`                     |
| `cannot find symbol: method getLast()`                               | `seq.get(seq.size() - 1)`                                        |
| `incompatible types: geometry_msgs.Point cannot be converted to ...` | `MessageTools.fromMessage(msg, euclidType)`                      |
| `no suitable method: set(...)` on `EuclidXxxMessage`                 | Either add a `set(...)` overload, or call `.getPoint()/.getQuaternion()/...` first |
| `no suitable method: toMessage(Euclid, EuclidXxxMessage)`            | Use the wrapper's own `set(...)` overload — no MessageTools call needed |
| `type argument T is not within bounds`                               | Add `<T extends ROS2Message<T>>` to the generic; at call sites with `Class<? extends ROS2Message<?>>`, cast `(Class)` and suppress |
| `cannot find symbol: Packet` / `PacketConsumer`                      | See "Removing the old Packet base class"; use `ROS2Message` + `reader.read()` |
| `cannot find symbol: *PubSubType` / `JSONSerializer`                 | Use `ROS2MessageCdrFileTools` for file I/O; see section above                 |
| `constructor X cannot be applied; required: no arguments` on message | Use `new X(); copy.set(source);`                                              |
| `EuclidPose3DMessage cannot be converted to Pose3DReadOnly`          | `.getPose().getPose()` or `.getPose()` on wrapper depending on target type      |
| `cannot find symbol: INVALID_MESSAGE_ID`                             | `ControllerMessageConstants.INVALID_MESSAGE_ID`                  |
| `cannot find symbol: IDLLongSequence` (message collection)         | `IDLIntSequence`; cast `getSequenceId()` to `int` for notifications |
| `cannot find symbol: set(i, v)` on `IDLDoubleSequence`             | `getBuffer().put(i, v)`                                          |
| `cannot find symbol: fastRemove` on `IDLObjectSequence`            | `remove(index)`                                                  |
| `no suitable method: set(EuclidXxxMessage)` on Frame types         | Use `.getPoint()` / `.getQuaternion()` on the wrapper first        |
| `ROS2Message cannot be converted to T` in `reader.read(buffer)`    | Use `reader.read()` return value, or raw-cast buffer + reader      |
| `Settable cannot be converted to ROS2Message` on publish           | `publisher.publish((ROS2Message) message)` with raw `ROS2Publisher` |
| `cannot find symbol: getW()` on Euclid Quaternion                    | `.getS()` (Euclid uses `S` for scalar)                           |
| `no suitable method: copyData(array/list, IDLObjectSequence)`        | Use typed `copyData(T[]/List<T>, …)` or `clear(); add().set` loop |
| `getTransform().set(RigidBodyTransform)` on TF messages              | `MessageTools.toMessage(transform, msg.getTransform())` |
| `toTDoubleArrayList(IDLFloatSequence)` missing                       | Loop `input.size()` / `input.get(i)` into `TDoubleArrayList`     |
| `geometry_msgs.Vector3.applyTransform` missing                       | Copy to `Vector3D`, transform, write back with `setX/Y/Z`        |
| `ROS2MessageCdrFileTools.messageToJsonNode` in try/catch IOException | `messageToJsonNode` does not throw — drop empty catch blocks     |

---

## Migration checklist for a new file

1. Update imports per the rename table above. Drop `.msg.dds.` infixes.
2. Replace `ROS2NodeBuilder` chains with direct `ROS2Node` / `AsyncROS2Node` constructors.
3. Replace topic builder method calls (`withRobot/withModule/withSuffix/withQoS/...`).
4. Move QoS off the topic and onto the `createPublisher` / `createSubscription` call.
5. Replace subscriptions: `reader.read()` pattern, `createSubscriptionSampler`, or `ROS2Helper.subscribeViaCallback` / `ROS2Input`.
6. Replace `subscriber.takeNextData` / `subscriber.remove` / `node.destroy`.
7. Delete `setUniqueId(...)` calls.
8. Replace `IDLSequence.Integer/Float/Boolean` and `RecyclingArrayList` types.
9. For sequence ops, swap `reset()` → `clear()`, `getLast()` → `get(size()-1)`, expand `addAll`/array-add if the subtype doesn't support it.
10. For geometry, prefer the EuclidXxxMessage wrapper's own `set(...)` overloads. Fall back to `MessageTools.toMessage/fromMessage` for plain `geometry_msgs.*` types only.
11. Add `<T extends ROS2Message<T>>` to any generic method/class that takes a topic or message.
12. For “latest message” state: `ROS2Input<T>`, `subscribeViaTypedNotification`, or `subscribeViaCallback` depending on filtering/allocation needs.
13. Search for `new EuclidXxxMessage()` inside loops — replace with `sequence.add().set(value)`.
14. No fully-qualified names in source. Add the import.

---

## Resources

- jros2 repo: `/home/d/Desktop/repository-group/jros2`
- Custom message example: `jros2/examples/custom-message-class/`
- Migration script (regex-level renames only): `/home/d/Desktop/repository-group/migrate-to-jros2.sh`
- Build command for status check:
  ```bash
  ./gradlew :ihmc-communication:compileJava :ihmc-humanoid-robotics:compileJava
  ```

---

## jros2 API status (upstream / migration)

### Already in jros2 (use these — do not re-request)

| Item | API | Notes |
| ---- | --- | ----- |
| Object sequence bulk append | `IDLObjectSequence.addAll(T[])`, `addAll(Collection<? extends T>)` | **Shallow** — shares references. Deep copy: `add().set(item)` or `MessageTools.copyData(T[], …)`. |
| “Consumer&lt;T&gt;” subscription | `ROS2Node.createSubscriptionSampler(topic, ROS2SubscriptionCallbackSampler<T>)` | Reuses one sample; calls `sampler.consume(sample)` after `reader.read(sample)`. Not named `createSubscription(Consumer)` — that name was never added. |
| IHMC wrapper for simple callbacks | `ROS2Helper.subscribeViaCallback(topic, Consumer<T>)` | `reader -> callback.accept(reader.read())` — allocates per sample unless you use volatile/swap/queue helpers. |

### Still open (reasonable future work)

1. **Deprecated aliases on `ROS2Topic`** (`@Deprecated withRobot/withModule/...`) — optional; branch already migrated call sites to new names.
2. **`ROS2NodeBuilder` (or equivalent)** — intraprocess / shared-memory / address-restricted nodes; blocks `@Disabled` tests in `ROS2ToolsTest`, `ROS2PeerClockOffsetEstimatorTest`.
3. **`MessageTestTools.epsilonEquals`** for plain `geometry_msgs` types — blocks `ROS2LogTest`.
4. **`SampleInfo` + matched subscription callback** — blocks `ROS2PeerClockOffsetEstimator` clock sync TODO.
5. **`Pose3D` wrapper in jros2** — or keep using `EuclidPose3DMessage`; commented `publish(ROS2Topic<Pose3D>, …)` stubs in `ROS2Helper` / `ROS2PublisherMap`.
6. **`ROS2NodeBuilder` transport modes** — intraprocess / shared-memory; blocks disabled `ROS2ToolsTest` methods.

### Explicitly not porting

- **`QueuedROS2Subscription`** — not useful for jros2; keep tests `@Disabled`. Use `AsyncROS2Node.createSubscription` instead (standard `reader.read()` / `reader.read(buffer)` callbacks).

---

## Disabled / not-yet-migrated tests

Search for `@Disabled // TODO: jros2 migration` to find these:
- `RealtimeROS2PublisherSubscriberTest` — `QueuedROS2Subscription` **not porting**; rewrite against `AsyncROS2Node.createSubscription` if re-enabled
- `FrameRealtimeROS2PublisherSubscriberTest` — same
- `ROS2PeerClockOffsetEstimatorTest` — needs `ROS2NodeBuilder` transport modes / `SampleInfo`
- `ROS2LogTest` — needs geometry `epsilonEquals` helper
- `MessageToolsTest` — old Packet serialize/deserialize
- Several individual `@Test` methods inside `ROS2ToolsTest` (class not disabled; intraprocess/loopback tests)
