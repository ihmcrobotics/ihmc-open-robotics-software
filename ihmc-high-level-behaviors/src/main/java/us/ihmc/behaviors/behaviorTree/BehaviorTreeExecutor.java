package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.lang3.function.TriFunction;
import behavior_msgs.BehaviorResultMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionComms;
import us.ihmc.behaviors.behaviorTree.condition.LLMConditionExecutor;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeExecutor extends BehaviorTree<BehaviorTreeRootNodeExecutor, BehaviorTreeNodeExecutor<?, ?>>
{
   private final ControllerStatusTracker controllerStatusTracker;
   private final SideDependentList<AbilityHandActionComms> abilityHandComms = new SideDependentList<>();

   public BehaviorTreeExecutor(
         ROS2SyncedRobotModel syncedRobot,
         ROS2PeerClockOffsetEstimator peerClockEstimator,
         ROS2ControllerHelper ros2ControllerHelper,
         TriFunction<DRCRobotModel, ROS2Node, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
         ImageSensor imageSensor,
         YOLOv8DetectionExecutor yolo,
         IsaacROSFoundationPoseCommunicatorMap foundationPose,
         TerrainMapData terrainMapData)
   {
      this(syncedRobot,
           peerClockEstimator,
           ros2ControllerHelper,
           kinematicsSimulationBuilder,
           imageSensor,
           yolo,
           foundationPose,
           terrainMapData,
           new WorkspaceResourceDirectory(BehaviorTreeExecutor.class, "/behaviorTrees"));
   }

   public BehaviorTreeExecutor(
         ROS2SyncedRobotModel syncedRobot,
         ROS2PeerClockOffsetEstimator peerClockEstimator,
         ROS2ControllerHelper ros2ControllerHelper,
         TriFunction<DRCRobotModel, ROS2Node, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
         ImageSensor imageSensor,
         YOLOv8DetectionExecutor yolo,
         IsaacROSFoundationPoseCommunicatorMap foundationPose,
         TerrainMapData terrainMapData,
         WorkspaceResourceDirectory behaviorTreesDirectory)
   {
      super(syncedRobot,
            ROS2ActorDesignation.ROBOT,
            peerClockEstimator,
            behaviorTreesDirectory,
            new BehaviorTreeExecutorNodeBuilder());

      controllerStatusTracker = new ControllerStatusTracker(new LogToolsLogger(), ros2ControllerHelper.getROS2Node(), syncedRobot);
      for (RobotSide robotSide : RobotSide.values)
         abilityHandComms.put(robotSide, new AbilityHandActionComms(robotSide, ros2ControllerHelper.getROS2Node()));

      ((BehaviorTreeExecutorNodeBuilder) getNodeBuilder()).initialize(this,
                                                                      saveFileDirectory,
                                                                      ros2ControllerHelper,
                                                                      kinematicsSimulationBuilder,
                                                                      syncedRobot,
                                                                      controllerStatusTracker,
                                                                      abilityHandComms,
                                                                      imageSensor,
                                                                      yolo,
                                                                      foundationPose,
                                                                      terrainMapData);
   }

   public void update()
   {
      for (RobotSide side : abilityHandComms.sides())
         abilityHandComms.get(side).update();

      if (rootNode != null)
      {
         rootNode.clock();

         rootNode.tick();

         update(rootNode);
      }
   }

   private void update(BehaviorTreeNodeExecutor<?, ?> node)
   {
      node.update();

      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         update(child);
      }
   }

   public void destroy()
   {
      modifyTreeTopology(BehaviorTreeTopologyOperationQueue::queueDestroyEntireTree);
      LLMConditionExecutor.destroy();
   }

   /** Default fails the leaf. {@link us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeExecutor} plays the mimic. */
   public long startNestedMimic(String catalogName)
   {
      return 0L;
   }

   public byte nestedMimicPhase(long requestId)
   {
      return BehaviorResultMessage.REJECTED;
   }

   /** Default fails the leaf. {@link us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeExecutor} runs follow. */
   public long startNestedFollow(String targetLabel, boolean closestPerson)
   {
      return 0L;
   }

   public byte nestedFollowPhase(long requestId)
   {
      return BehaviorResultMessage.REJECTED;
   }

}
