package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;

/**
 * Top level class for the robot's behavior tree.
 */
public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{
   private final ROS2BehaviorTree<BehaviorTreeNodeExecutor<?, ?>> ros2BehaviorTree;

   public ROS2BehaviorTreeExecutor(ROS2ControllerHelper ros2ControllerHelper,
                                   ROS2SyncedRobotModel syncedRobot,
                                   YOLOv8DetectionExecutor yolo,
                                   IsaacROSFoundationPoseCommunicatorMap foundationPose,
                                   TerrainMapData terrainMapData,
                                   ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      super(syncedRobot, peerClockEstimator, ros2ControllerHelper, yolo, foundationPose, terrainMapData);

      ros2BehaviorTree = new ROS2BehaviorTree<>((BehaviorTree) this, ros2ControllerHelper); // FIXME
   }

   /** Expected to be called at the {@link ROS2BehaviorTree#SYNC_FREQUENCY} */
   public void update()
   {
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
