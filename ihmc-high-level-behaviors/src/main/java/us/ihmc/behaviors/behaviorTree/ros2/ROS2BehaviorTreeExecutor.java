package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

/**
 * Top level class for the robot's behavior tree.
 */
public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{
   private final ROS2BehaviorTree<BehaviorTreeNodeExecutor<?, ?>> ros2BehaviorTree;

   public ROS2BehaviorTreeExecutor(ROS2ControllerHelper ros2ControllerHelper,
                                   DRCRobotModel robotModel,
                                   ROS2SyncedRobotModel syncedRobot,
                                   ROS2PeerClockOffsetEstimator peerClockEstimator,
                                   ReferenceFrameLibrary referenceFrameLibrary,
                                   DetectionManager detectionManager)
   {
      super(robotModel, syncedRobot, peerClockEstimator, referenceFrameLibrary, detectionManager, ros2ControllerHelper);

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
