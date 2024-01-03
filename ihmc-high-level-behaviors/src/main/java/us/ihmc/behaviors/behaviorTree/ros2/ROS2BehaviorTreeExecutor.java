package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

/**
 * Top level class for the robot's behavior tree.
 */
public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{
   private final ROS2BehaviorTreeState ros2BehaviorTreeState;

   public ROS2BehaviorTreeExecutor(ROS2ControllerHelper ros2ControllerHelper,
                                   ROS2ActorDesignation ros2ActorDesignation,
                                   DRCRobotModel robotModel,
                                   ROS2SyncedRobotModel syncedRobot,
                                   ReferenceFrameLibrary referenceFrameLibrary,
                                   WalkingFootstepTracker footstepTracker,
                                   FootstepPlanningModule footstepPlanner,
                                   FootstepPlannerParametersBasics footstepPlannerParameters,
                                   WalkingControllerParameters walkingControllerParameters)
   {
      super(robotModel,
            syncedRobot,
            referenceFrameLibrary,
            footstepTracker,
            footstepPlanner,
            footstepPlannerParameters,
            walkingControllerParameters,
            ros2ControllerHelper);

      ros2BehaviorTreeState = new ROS2BehaviorTreeState(getState(), this::setRootNode, ros2ControllerHelper);
   }

   public void update()
   {
      ros2BehaviorTreeState.updateSubscription();

      super.update();

      ros2BehaviorTreeState.updatePublication();
   }

   public void destroy()
   {
      ros2BehaviorTreeState.destroy();

      super.destroy();
   }
}
