package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class RDXROS2BehaviorTree extends RDXBehaviorTree
{
   private final ROS2BehaviorTreeState ros2BehaviorTreeState;

   public RDXROS2BehaviorTree(DRCRobotModel robotModel,
                              ROS2SyncedRobotModel syncedRobot,
                              RobotCollisionModel selectionCollisionModel,
                              RDXBaseUI baseUI,
                              RDX3DPanel panel3D,
                              ReferenceFrameLibrary referenceFrameLibrary,
                              ROS2ControllerPublishSubscribeAPI ros2)
   {
      super(robotModel, syncedRobot, selectionCollisionModel, baseUI, panel3D, referenceFrameLibrary, ros2);

      ros2BehaviorTreeState = new ROS2BehaviorTreeState(getBehaviorTreeState(), ros2, ROS2ActorDesignation.OPERATOR);

   }

   public void update()
   {
      ros2BehaviorTreeState.updateSubscription();

      super.update();

      ros2BehaviorTreeState.updatePublication();
   }

   public void destroy()
   {
      // TODO
   }
}
