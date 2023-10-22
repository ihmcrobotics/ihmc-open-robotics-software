package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeExtensionSubtreeRebuilder;
import us.ihmc.behaviors.tools.ROS2HandWrenchCalculator;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.SideDependentList;

public class BehaviorTreeExecutor
{
   private final BehaviorTreeState behaviorTreeState;
   private final BehaviorTreeExecutorNodeBuilder nodeBuilder;
   private final BehaviorTreeExtensionSubtreeRebuilder treeRebuilder;

   public BehaviorTreeExecutor(DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               ReferenceFrameLibrary referenceFrameLibrary,
                               WalkingFootstepTracker footstepTracker,
                               SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators,
                               FootstepPlanningModule footstepPlanner,
                               FootstepPlannerParametersBasics footstepPlannerParameters,
                               WalkingControllerParameters walkingControllerParameters,
                               ROS2ControllerHelper ros2ControllerHelper)
   {
      nodeBuilder = new BehaviorTreeExecutorNodeBuilder(robotModel,
                                                        syncedRobot,
                                                        referenceFrameLibrary,
                                                        footstepTracker,
                                                        handWrenchCalculators,
                                                        footstepPlanner,
                                                        footstepPlannerParameters,
                                                        walkingControllerParameters,
                                                        ros2ControllerHelper);
      treeRebuilder = new BehaviorTreeExtensionSubtreeRebuilder(this::getRootNode);

      behaviorTreeState = new BehaviorTreeState(nodeBuilder, treeRebuilder, this::getRootNode);
   }

   public void update()
   {
      getRootNode().clock();

      getRootNode().tick();
   }

   public BehaviorTreeNodeExecutor getRootNode()
   {
      return (BehaviorTreeNodeExecutor) behaviorTreeState.getRootNode();
   }

   public BehaviorTreeState getState()
   {
      return behaviorTreeState;
   }
}
