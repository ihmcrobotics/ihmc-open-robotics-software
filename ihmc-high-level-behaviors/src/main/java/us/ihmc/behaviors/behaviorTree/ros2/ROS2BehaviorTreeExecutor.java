package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.BehaviorCommandMessage;
import behavior_msgs.BehaviorResultMessage;
import behavior_msgs.BehaviorTreeLeafStatusMessage;
import behavior_msgs.BehaviorTreeYoDataMessage;
import org.apache.commons.lang3.function.TriFunction;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.log.LogTools;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Top level class for the robot's behavior tree.
 */
public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{
   private final ROS2BehaviorTree<BehaviorTreeNodeExecutor<?, ?>> ros2BehaviorTree;

   private final BehaviorTreeYoDataMessage yoDataMessage = new BehaviorTreeYoDataMessage();
   private final ROS2Publisher<BehaviorTreeYoDataMessage> yoDataPublisher;
   private final ROS2Node ros2Node;
   private final ROS2Subscription<BehaviorCommandMessage> commandSubscription;
   private final ROS2Publisher<BehaviorResultMessage> resultPublisher;
   private final BehaviorResultMessage resultMessage = new BehaviorResultMessage();
   private final ROS2Publisher<BehaviorTreeLeafStatusMessage> leafStatusPublisher;
   private final BehaviorTreeLeafStatusMessage leafStatusMessage = new BehaviorTreeLeafStatusMessage();
   /** File name from the last successful start(Tree). Empty when this process has not loaded a file. */
   private String loadedTreeFile = "";
   private int previousStatusSubscribers;
   private Object publishedRoot;
   private int publishedLeafCount = -1;
   private static final int MAX_STATUS_LEAVES = 120;
   private static final long ELAPSED_PUBLISH_PERIOD_NANOS = 1_000_000_000L;
   /** Bitfield of the five leaf flags. -1 means this index has not been sent. */
   private final int[] lastFlags = new int[MAX_STATUS_LEAVES];
   private final long[] lastElapsedPublishNanos = new long[MAX_STATUS_LEAVES];
   /** Commands copied out of the ROS callback. Applied on this update thread. */
   private final ConcurrentLinkedQueue<QueuedCommand> commands = new ConcurrentLinkedQueue<>();
   private boolean hasActiveBehavior;
   private long activeRequestId;
   private String activeName = "";
   private byte activeKind;
   private MimicStarter mimicStarter;
   private FollowStarter followStarter;
   private Runnable followTicker;
   private Runnable followStopper = () -> { };
   private final AtomicLong nextNestedId = new AtomicLong();
   private final Tracked mimic = new Tracked();
   private final Tracked follow = new Tracked();
   private final ConcurrentLinkedQueue<Completion> mimicCompletions = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<Completion> followCompletions = new ConcurrentLinkedQueue<>();

   public ROS2BehaviorTreeExecutor(
         ROS2ControllerHelper ros2ControllerHelper,
         ROS2SyncedRobotModel syncedRobot,
         TriFunction<DRCRobotModel, ROS2Node, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
         ImageSensor imageSensor,
         YOLOv8DetectionExecutor yolo,
         IsaacROSFoundationPoseCommunicatorMap foundationPose,
         TerrainMapData terrainMapData,
         ROS2PeerClockOffsetEstimator peerClockEstimator,
         WorkspaceResourceDirectory behaviorTreesDirectory)
   {
      super(syncedRobot,
            peerClockEstimator,
            ros2ControllerHelper,
            kinematicsSimulationBuilder,
            imageSensor,
            yolo,
            foundationPose,
            terrainMapData,
            behaviorTreesDirectory);

      Arrays.fill(lastFlags, -1);
      ros2Node = ros2ControllerHelper.getROS2Node();
      ros2BehaviorTree = new ROS2BehaviorTree<>((BehaviorTree) this, ros2Node);

      yoDataPublisher = ros2Node.createPublisher(AutonomyAPI.BEHAVIOR_YO_DATA);
      resultPublisher = ros2Node.createPublisher(AutonomyAPI.BEHAVIOR_RESULT);
      leafStatusPublisher = ros2Node.createPublisher(AutonomyAPI.BEHAVIOR_TREE_LEAF_STATUS);
      commandSubscription = ros2Node.createSubscription(AutonomyAPI.BEHAVIOR_COMMAND, reader ->
      {
         BehaviorCommandMessage message = reader.read();
         if (message == null)
            return;
         commands.add(new QueuedCommand(message.getRequestId(),
                                        message.getKind(),
                                        message.getNameAsString(),
                                        message.getNested(),
                                        message.getClosestPerson()));
      });
   }

   /** Expected to be called at the {@link ROS2BehaviorTree#SYNC_FREQUENCY} */
   public void update()
   {
      QueuedCommand command;
      while ((command = commands.poll()) != null)
         applyCommand(command);
      drain(mimicCompletions, mimic, false);
      drain(followCompletions, follow, true);
      if (followTicker != null)
         followTicker.run();
      drain(followCompletions, follow, true);

      ROS2BehaviorTreeMessageTools.packYoData((BehaviorTreeExecutor) ros2BehaviorTree.getBehaviorTree(), yoDataMessage);
      yoDataPublisher.publish(yoDataMessage);

      // A status subscriber is a monitor. It wins over an editor subscription on the same topic pair:
      // skip the CRDT both ways so an open editor cannot change a tree that is only being watched.
      // With nobody subscribed, skip both publishes. The tree still ticks below.
      int statusSubscribers = leafStatusPublisher.getPublicationMatchedStatus();
      boolean monitor = statusSubscribers > 0;
      boolean subscriberJoined = statusSubscribers > previousStatusSubscribers;
      if (!monitor && ros2BehaviorTree.getStateSubscriberCount() > 0)
      {
         ros2BehaviorTree.updatePublication();
         ros2BehaviorTree.updateSubscription();
      }

      // TODO: Consider updating this at a higher rate than the comms
      super.update();
      publishCompletionIfFinished();
      if (monitor)
         publishLeafStatus(subscriberJoined);
      previousStatusSubscribers = statusSubscribers;
   }

   /**
    * Sends leaf flags after this tick has written them. A new status subscriber, a replaced root,
    * or a changed leaf count gets every leaf. Otherwise a leaf is included when a flag changes,
    * and an executing leaf is included again once a second so the timeline can move between samples.
    */
   private void publishLeafStatus(boolean subscriberJoined)
   {
      List<LeafNodeState<?>> leaves = getRootNode() == null ? List.of() : getRootNode().getState().getOrderedLeaves();
      int leafCount = Math.min(leaves.size(), MAX_STATUS_LEAVES);
      boolean rootChanged = getRootNode() != publishedRoot || leafCount != publishedLeafCount;
      boolean forceSnapshot = subscriberJoined || rootChanged;
      if (rootChanged)
      {
         Arrays.fill(lastFlags, -1);
         publishedRoot = getRootNode();
         publishedLeafCount = leafCount;
      }

      leafStatusMessage.setSnapshot(forceSnapshot);
      leafStatusMessage.setTreeFile(forceSnapshot ? loadedTreeFile : "");
      leafStatusMessage.getLeafIndex().clear();
      leafStatusMessage.getIsActive().clear();
      leafStatusMessage.getIsNextForExecution().clear();
      leafStatusMessage.getCanExecute().clear();
      leafStatusMessage.getIsExecuting().clear();
      leafStatusMessage.getFailed().clear();
      leafStatusMessage.getElapsedSeconds().clear();

      long now = System.nanoTime();
      for (int i = 0; i < leafCount; i++)
      {
         LeafNodeState<?> leaf = leaves.get(i);
         int index = leaf.getLeafIndex();
         if (index < 0 || index >= MAX_STATUS_LEAVES)
            continue;

         boolean active = leaf.getIsActive();
         boolean next = leaf.getIsNextForExecution();
         boolean canExecute = leaf.getCanExecute();
         boolean executing = leaf.getIsExecuting();
         boolean failed = leaf.getFailed();
         double elapsed = Double.NaN;
         if (leaf instanceof ActionNodeState<?> action)
            elapsed = action.getElapsedExecutionTime();

         int flags = (active ? 1 : 0) | (next ? 2 : 0) | (canExecute ? 4 : 0) | (executing ? 8 : 0) | (failed ? 16 : 0);
         boolean elapsedDue = executing && now - lastElapsedPublishNanos[index] >= ELAPSED_PUBLISH_PERIOD_NANOS;
         if (!forceSnapshot && lastFlags[index] == flags && !elapsedDue)
            continue;

         leafStatusMessage.getLeafIndex().add(index);
         leafStatusMessage.getIsActive().add(active);
         leafStatusMessage.getIsNextForExecution().add(next);
         leafStatusMessage.getCanExecute().add(canExecute);
         leafStatusMessage.getIsExecuting().add(executing);
         leafStatusMessage.getFailed().add(failed);
         leafStatusMessage.getElapsedSeconds().add(elapsed);

         lastFlags[index] = flags;
         if (executing)
            lastElapsedPublishNanos[index] = now;
      }

      if (!forceSnapshot && leafStatusMessage.getLeafIndex().size() == 0)
         return;

      leafStatusPublisher.publish(leafStatusMessage);
   }

   public void destroy()
   {
      ros2Node.destroySubscription(commandSubscription);
      ros2BehaviorTree.destroy();

      super.destroy();
   }

   /**
    * A tree starts only after its file loads, so a missing file leaves the running tree in place.
    * A mimic or follow is rejected before it can cancel the tree.
    * A nested command does not replace the tree, so a later leaf can run a mimic or follow.
    */
   private void applyCommand(QueuedCommand command)
   {
      if (command.kind == BehaviorCommandMessage.CANCEL)
      {
         cancelActive(command.requestId);
         return;
      }
      String missing = missingField(command);
      if (missing != null)
      {
         publishResult(command.requestId, BehaviorResultMessage.REJECTED, missing, true);
         return;
      }
      switch (command.kind)
      {
         case BehaviorCommandMessage.TREE -> startTree(command);
         case BehaviorCommandMessage.MIMIC -> startMimic(command);
         case BehaviorCommandMessage.FOLLOW -> startFollow(command);
         default -> publishResult(command.requestId, BehaviorResultMessage.REJECTED, "Unknown behavior command kind: " + command.kind, true);
      }
   }

   private void startTree(QueuedCommand command)
   {
      if (command.nested)
      {
         publishResult(command.requestId, BehaviorResultMessage.REJECTED, "A tree cannot be started from inside a behavior", true);
         return;
      }

      long previousRequestId = activeRequestId;
      boolean replace = hasActiveBehavior;
      // Unarmed load: a missing or unreadable file leaves the running behavior in place, and the
      // install tick does not start a leaf. Stop the previous walk before arming the new root.
      if (!loadBehavior(command.name))
      {
         publishResult(command.requestId, BehaviorResultMessage.REJECTED, "Cannot load " + command.name, true);
         return;
      }
      if (replace)
         releaseActiveBehavior(previousRequestId, true, false);
      else
         followStopper.run();

      hasActiveBehavior = true;
      activeRequestId = command.requestId;
      activeName = command.name;
      activeKind = BehaviorCommandMessage.TREE;
      loadedTreeFile = command.name;
      getRootNode().getState().setAutomaticExecution(true);
      getRootNode().update();
      publishResult(command.requestId, BehaviorResultMessage.ACCEPTED, "Running " + command.name, false);
      publishCompletionIfFinished();
   }

   /** An unknown name or a player that is busy is rejected before anything is cancelled. */
   private void startMimic(QueuedCommand command)
   {
      if (mimicStarter == null)
      {
         reject(mimic, command, "Mimic player is not connected");
         return;
      }

      int outcome = mimicStarter.start(command.requestId, command.name);
      if (outcome == MimicStarter.UNKNOWN)
      {
         reject(mimic, command, "Cannot play " + command.name);
         return;
      }
      if (outcome == MimicStarter.BUSY)
      {
         reject(mimic, command, "Hang on, I'm still finishing up.");
         return;
      }

      if (!command.nested && hasActiveBehavior)
         releaseActiveBehavior(activeRequestId, true, true);

      boolean alreadyDone = outcome == MimicStarter.ALREADY_DONE;
      accept(mimic, command, BehaviorCommandMessage.MIMIC, alreadyDone ? BehaviorResultMessage.SUCCEEDED : BehaviorResultMessage.ACCEPTED,
             alreadyDone ? "Already standing." : "Playing " + command.name);
      if (alreadyDone)
      {
         publishResult(command.requestId, BehaviorResultMessage.SUCCEEDED, "Already standing.", false);
         if (!command.nested)
            hasActiveBehavior = false;
      }
   }

   /**
    * The root clears automatic execution at the end of the sequence and when a leaf fails.
    * Speech does not wait for this. A later leaf will.
    */
   private void publishCompletionIfFinished()
   {
      if (!hasActiveBehavior || activeKind != BehaviorCommandMessage.TREE)
         return;
      BehaviorTreeRootNodeExecutor root = getRootNode();
      if (root == null || root.getState().getAutomaticExecution())
         return;

      if (root.isEndOfSequence())
      {
         publishResult(activeRequestId, BehaviorResultMessage.SUCCEEDED, "Finished " + activeName, false);
         hasActiveBehavior = false;
      }
      else if (!root.getFailedLeaves().isEmpty())
      {
         publishResult(activeRequestId, BehaviorResultMessage.FAILED, "Failed " + activeName, false);
         hasActiveBehavior = false;
      }
   }

   private void cancelActive(long requestId)
   {
      if (hasActiveBehavior)
      {
         releaseActiveBehavior(activeRequestId, true, true);
         publishResult(requestId, BehaviorResultMessage.CANCELLED, "Cancelled", false);
      }
      else
      {
         publishResult(requestId, BehaviorResultMessage.CANCELLED, "Nothing is running", false);
      }
   }

   private static String missingField(QueuedCommand command)
   {
      if (command.kind == BehaviorCommandMessage.MIMIC && command.name.isBlank())
         return "Missing mimic name";
      if (command.kind == BehaviorCommandMessage.TREE && command.name.isBlank())
         return "Missing behavior file";
      if (command.kind == BehaviorCommandMessage.FOLLOW && command.name.isBlank())
         return "Missing follow target";
      return null;
   }

   private void startFollow(QueuedCommand command)
   {
      if (followStarter == null)
      {
         reject(follow, command, "Follow player is not connected");
         return;
      }

      int outcome = followStarter.start(command.requestId, command.name, command.closestPerson);
      if (outcome == FollowStarter.BUSY)
      {
         reject(follow, command, "Hang on, I'm still finishing up.");
         return;
      }

      if (!command.nested && hasActiveBehavior)
         releaseActiveBehavior(activeRequestId, false, true);
      accept(follow, command, BehaviorCommandMessage.FOLLOW, BehaviorResultMessage.ACCEPTED, "Following " + command.name);
   }

   private void reject(Tracked tracked, QueuedCommand command, String summary)
   {
      publishResult(command.requestId, BehaviorResultMessage.REJECTED, summary, true);
      if (command.nested)
         tracked.remember(command, BehaviorResultMessage.REJECTED);
   }

   /** Nested calls leave the running tree in place. {@code rememberedPhase} is what a leaf reads immediately. */
   private void accept(Tracked tracked, QueuedCommand command, byte kind, byte rememberedPhase, String summary)
   {
      if (!command.nested)
      {
         hasActiveBehavior = true;
         activeRequestId = command.requestId;
         activeName = command.name;
         activeKind = kind;
      }
      tracked.remember(command, rememberedPhase);
      publishResult(command.requestId, BehaviorResultMessage.ACCEPTED, summary, false);
   }

   private long nestedRequestId()
   {
      return -nextNestedId.incrementAndGet();
   }

   @Override
   public long startNestedMimic(String catalogName)
   {
      long requestId = nestedRequestId();
      startMimic(new QueuedCommand(requestId, BehaviorCommandMessage.MIMIC, catalogName, true, false));
      return requestId;
   }

   @Override
   public byte nestedMimicPhase(long requestId)
   {
      return mimic.phase(requestId);
   }

   public void completeMimic(long requestId, boolean succeeded)
   {
      mimicCompletions.add(new Completion(requestId, succeeded ? BehaviorResultMessage.SUCCEEDED : BehaviorResultMessage.FAILED));
   }

   public void setMimicStarter(MimicStarter mimicStarter)
   {
      this.mimicStarter = mimicStarter;
   }

   @Override
   public long startNestedFollow(String targetLabel, boolean closestPerson)
   {
      long requestId = nestedRequestId();
      startFollow(new QueuedCommand(requestId, BehaviorCommandMessage.FOLLOW, targetLabel, true, closestPerson));
      return requestId;
   }

   @Override
   public byte nestedFollowPhase(long requestId)
   {
      return follow.phase(requestId);
   }

   public void completeFollow(long requestId, byte phase)
   {
      followCompletions.add(new Completion(requestId, phase));
   }

   public void setFollowStarter(FollowStarter followStarter)
   {
      this.followStarter = followStarter;
   }

   public void setFollowTicker(Runnable followTicker)
   {
      this.followTicker = followTicker;
   }

   public void setFollowStopper(Runnable followStopper)
   {
      this.followStopper = followStopper == null ? () -> { } : followStopper;
   }

   private void drain(ConcurrentLinkedQueue<Completion> completions, Tracked tracked, boolean lostOnFailure)
   {
      Completion completion;
      while ((completion = completions.poll()) != null)
      {
         if (completion.requestId != tracked.requestId || tracked.phase != BehaviorResultMessage.ACCEPTED)
            continue;
         tracked.phase = completion.phase;
         String summary = switch (completion.phase)
         {
            case BehaviorResultMessage.FAILED -> (lostOnFailure ? "Lost " : "Failed ") + tracked.name;
            case BehaviorResultMessage.CANCELLED -> "Cancelled";
            default -> "Finished " + tracked.name;
         };
         publishResult(completion.requestId, completion.phase, summary, false);
         if (!tracked.nested && hasActiveBehavior && activeRequestId == completion.requestId)
            hasActiveBehavior = false;
      }
   }

   /**
    * Cancels the active top-level behavior.
    * A newly loaded tree passes {@code clearAutomaticExecution} false, because load already armed the new root.
    * A follow that is about to retarget passes {@code stopFollowController} false, because its starter already called start.
    */
   private void releaseActiveBehavior(long requestId, boolean stopFollowController, boolean clearAutomaticExecution)
   {
      if (stopFollowController)
         followStopper.run();
      publishResult(requestId, BehaviorResultMessage.CANCELLED, "Cancelled", false);
      if (stopFollowController && follow.phase == BehaviorResultMessage.ACCEPTED && follow.requestId != requestId)
         publishResult(follow.requestId, BehaviorResultMessage.CANCELLED, "Cancelled", false);
      if (follow.phase == BehaviorResultMessage.ACCEPTED && (follow.requestId == requestId || stopFollowController))
         follow.phase = BehaviorResultMessage.CANCELLED;
      hasActiveBehavior = false;
      if (clearAutomaticExecution && getRootNode() != null)
         getRootNode().getState().setAutomaticExecution(false);
   }

   private void publishResult(long requestId, byte phase, String summary, boolean rejected)
   {
      LogTools.info("Behavior {}: {} ({})", requestId, phase, summary);
      resultMessage.setRequestId(requestId);
      resultMessage.setPhase(phase);
      resultMessage.setSummary(summary);
      resultMessage.setRejected(rejected);
      resultPublisher.publish(resultMessage);
   }

   /**
    * Publishes one catalog mimic. Returns {@link #UNKNOWN}, {@link #STARTED}, {@link #ALREADY_DONE}, or {@link #BUSY}.
    * Does not decide whether the running tree is cancelled.
    */
   @FunctionalInterface
   public interface MimicStarter
   {
      int UNKNOWN = 0;
      int STARTED = 1;
      int ALREADY_DONE = 2;
      int BUSY = 3;

      int start(long requestId, String name);
   }

   /**
    * Starts one person follow. Returns {@link #STARTED} or {@link #BUSY}.
    * Does not decide whether the running tree is cancelled.
    */
   @FunctionalInterface
   public interface FollowStarter
   {
      int STARTED = 1;
      int BUSY = 3;

      int start(long requestId, String name, boolean closestPerson);
   }

   private static final class Tracked
   {
      private long requestId;
      private byte phase = -1;
      private boolean nested;
      private String name = "";

      private void remember(QueuedCommand command, byte phase)
      {
         requestId = command.requestId;
         this.phase = phase;
         nested = command.nested;
         name = command.name;
      }

      private byte phase(long requestId)
      {
         return requestId == this.requestId ? phase : BehaviorResultMessage.REJECTED;
      }
   }

   private static final class Completion
   {
      private final long requestId;
      private final byte phase;

      private Completion(long requestId, byte phase)
      {
         this.requestId = requestId;
         this.phase = phase;
      }
   }

   private static final class QueuedCommand
   {
      private final long requestId;
      private final byte kind;
      private final String name;
      private final boolean nested;
      private final boolean closestPerson;

      private QueuedCommand(long requestId, byte kind, String name, boolean nested, boolean closestPerson)
      {
         this.requestId = requestId;
         this.kind = kind;
         this.name = name == null ? "" : name;
         this.nested = nested;
         this.closestPerson = closestPerson;
      }
   }
}
