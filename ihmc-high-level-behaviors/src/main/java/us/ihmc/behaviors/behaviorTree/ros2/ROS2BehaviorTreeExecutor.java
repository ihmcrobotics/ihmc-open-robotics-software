package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.BehaviorCommandMessage;
import behavior_msgs.BehaviorResultMessage;
import behavior_msgs.BehaviorTreeYoDataMessage;
import org.apache.commons.lang3.function.TriFunction;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.sensors.ImageSensor;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

/**
 * Top level class for the robot's behavior tree.
 */
public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{
   private final ROS2Node ros2Node;
   private final ROS2BehaviorTree<BehaviorTreeNodeExecutor<?, ?>> ros2BehaviorTree;

   private final BehaviorTreeYoDataMessage yoDataMessage = new BehaviorTreeYoDataMessage();
   private final ROS2Publisher<BehaviorTreeYoDataMessage> yoDataPublisher;
   private final ROS2Publisher<BehaviorResultMessage> resultPublisher;
   private final ROS2Subscription<BehaviorCommandMessage> commandSubscription;
   private final Queue<BehaviorCommandMessage> commands = new ConcurrentLinkedQueue<>();
   private volatile Starter mimicStarter;
   private volatile Starter followStarter;
   private volatile Consumer<Byte> stopKind;
   private byte activeKind;
   private boolean hasActive;
   private boolean applying;

   public ROS2BehaviorTreeExecutor(
         ROS2ControllerHelper ros2ControllerHelper,
         ROS2SyncedRobotModel syncedRobot,
         TriFunction<DRCRobotModel, ROS2Node, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
         ImageSensor imageSensor,
         YOLOv8DetectionExecutor yolo,
         IsaacROSFoundationPoseCommunicatorMap foundationPose,
         TerrainMapData terrainMapData,
         ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      super(syncedRobot, peerClockEstimator, ros2ControllerHelper, kinematicsSimulationBuilder, imageSensor, yolo, foundationPose, terrainMapData);

      ros2Node = ros2ControllerHelper.getROS2Node();
      ros2BehaviorTree = new ROS2BehaviorTree<>((BehaviorTree) this, ros2Node);

      yoDataPublisher = ros2Node.createPublisher(AutonomyAPI.BEHAVIOR_YO_DATA);
      resultPublisher = ros2Node.createPublisher(AutonomyAPI.BEHAVIOR_RESULT);
      commandSubscription = ros2Node.createSubscription(AutonomyAPI.BEHAVIOR_COMMAND,
                                                        reader -> commands.add(new BehaviorCommandMessage(reader.read())));
   }

   public void setMimicStarter(Starter starter)
   {
      mimicStarter = starter;
   }

   public void setFollowStarter(Starter starter)
   {
      followStarter = starter;
   }

   public void setStopKind(Consumer<Byte> stopKind)
   {
      this.stopKind = stopKind;
   }

   /** Expected to be called at the {@link ROS2BehaviorTree#SYNC_FREQUENCY} */
   public void update()
   {
      if (!applying)
      {
         applying = true;
         try
         {
            for (BehaviorCommandMessage command; (command = commands.poll()) != null; )
               apply(command);
         }
         finally
         {
            applying = false;
         }
      }

      ROS2BehaviorTreeMessageTools.packYoData((BehaviorTreeExecutor) ros2BehaviorTree.getBehaviorTree(), yoDataMessage);
      yoDataPublisher.publish(yoDataMessage);

      ros2BehaviorTree.updatePublication();
      ros2BehaviorTree.updateSubscription();

      // TODO: Consider updating this at a higher rate than the comms
      super.update();
   }

   public void destroy()
   {
      if (commandSubscription != null)
         ros2Node.destroySubscription(commandSubscription);
      ros2BehaviorTree.destroy();

      super.destroy();
   }

   private void apply(BehaviorCommandMessage command)
   {
      if (command.getNameAsString().isBlank())
      {
         publish(command.getRequestId(), BehaviorResultMessage.REJECTED, "Missing name", true);
         return;
      }
      if (command.getKind() == BehaviorCommandMessage.TREE)
      {
         startTree(command);
         return;
      }

      Starter starter = command.getKind() == BehaviorCommandMessage.FOLLOW ? followStarter
            : command.getKind() == BehaviorCommandMessage.MIMIC ? mimicStarter : null;
      if (starter == null && command.getKind() != BehaviorCommandMessage.MIMIC && command.getKind() != BehaviorCommandMessage.FOLLOW)
      {
         publish(command.getRequestId(), BehaviorResultMessage.REJECTED, "Unknown behavior command kind: " + command.getKind(), true);
         return;
      }
      StartResult result = start(starter, command);
      if (result.rejected())
      {
         publish(command.getRequestId(), result.phase(), result.summary(), true);
         return;
      }
      cancelActive(command.getKind());
      if (result.phase() != BehaviorResultMessage.SUCCEEDED)
         remember(command);
      publish(command.getRequestId(), result.phase(), result.summary(), false);
   }

   private void startTree(BehaviorCommandMessage command)
   {
      String name = command.getNameAsString();
      if (!canLoadBehavior(name))
      {
         publish(command.getRequestId(), BehaviorResultMessage.REJECTED, "Cannot load " + name, true);
         return;
      }
      cancelActive(BehaviorCommandMessage.TREE);
      if (!loadBehavior(name))
      {
         publish(command.getRequestId(), BehaviorResultMessage.REJECTED, "Cannot load " + name, true);
         return;
      }
      remember(command);
      publish(command.getRequestId(), BehaviorResultMessage.ACCEPTED, "Running " + name, false);
   }

   private StartResult start(Starter starter, BehaviorCommandMessage command)
   {
      if (starter == null)
         return new StartResult(BehaviorResultMessage.REJECTED, "Behavior player is not connected", true);
      return starter.start(command.getRequestId(), command.getNameAsString(), command.getClosestPerson());
   }

   private void cancelActive(byte newKind)
   {
      if (!hasActive)
         return;
      if (activeKind == BehaviorCommandMessage.TREE)
      {
         if (getRootNode() != null)
            getRootNode().getState().setAutomaticExecution(false);
      }
      else if (activeKind != newKind && stopKind != null)
         stopKind.accept(activeKind);
      hasActive = false;
   }

   private void remember(BehaviorCommandMessage command)
   {
      hasActive = true;
      activeKind = command.getKind();
   }

   private void publish(long requestId, byte phase, String summary, boolean rejected)
   {
      BehaviorResultMessage message = new BehaviorResultMessage();
      message.setRequestId(requestId);
      message.setPhase(phase);
      message.setSummary(summary);
      message.setRejected(rejected);
      resultPublisher.publish(message);
   }

   @FunctionalInterface
   public interface Starter
   {
      StartResult start(long requestId, String name, boolean closestPerson);
   }

   public record StartResult(byte phase, String summary, boolean rejected) {}
}
