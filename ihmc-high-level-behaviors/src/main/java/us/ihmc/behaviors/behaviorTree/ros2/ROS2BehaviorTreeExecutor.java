package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeYoDataMessage;
import org.apache.commons.lang3.function.TriFunction;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractionThread;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.ImageSensor;

/**
 * Top level class for the robot's behavior tree.
 */
public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{
   private final ROS2BehaviorTree<BehaviorTreeNodeExecutor<?, ?>> ros2BehaviorTree;

   private final BehaviorTreeYoDataMessage yoDataMessage = new BehaviorTreeYoDataMessage();
   private final ROS2Publisher<BehaviorTreeYoDataMessage> yoDataPublisher;

   public ROS2BehaviorTreeExecutor(
         ROS2ControllerHelper ros2ControllerHelper,
         ROS2SyncedRobotModel syncedRobot,
         TriFunction<DRCRobotModel, ROS2NodeBuilder, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
         ImageSensor imageSensor,
         YOLOv8DetectionExecutor yolo,
         IsaacROSFoundationPoseCommunicatorMap foundationPose,
         RapidPlanarRegionsExtractionThread planarRegions,
         TerrainMapData terrainMapData,
         ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      super(syncedRobot, peerClockEstimator, ros2ControllerHelper, kinematicsSimulationBuilder, imageSensor, yolo, foundationPose, planarRegions, terrainMapData);

      ros2BehaviorTree = new ROS2BehaviorTree<>((BehaviorTree) this, ros2ControllerHelper);

      yoDataPublisher = ros2ControllerHelper.getROS2Node().createPublisher(AutonomyAPI.BEHAVIOR_YO_DATA);
   }

   /** Expected to be called at the {@link ROS2BehaviorTree#SYNC_FREQUENCY} */
   public void update()
   {
      ROS2BehaviorTreeMessageTools.packYoData((BehaviorTreeExecutor) ros2BehaviorTree.getBehaviorTree(), yoDataMessage);
      yoDataPublisher.publish(yoDataMessage);

      ros2BehaviorTree.updatePublication();
      ros2BehaviorTree.updateSubscription();

      // TODO: Consider updating this at a higher rate than the comms
      super.update();
   }

   public void destroy()
   {
      ros2BehaviorTree.destroy();

      super.destroy();
   }
}
