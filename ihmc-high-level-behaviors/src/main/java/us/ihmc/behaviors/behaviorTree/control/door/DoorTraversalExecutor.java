package us.ihmc.behaviors.behaviorTree.control.door;

import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitDurationActionState;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNodeTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
   private boolean waitForPullScrewToFinish = false;
   private boolean waitForGraspToFinish = false;

   public DoorTraversalExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new DoorTraversalState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void tick()
   {
      super.tick();

      // TODO: Tick children
   }

   @Override
   public void update()
   {
      super.update();

      updateSubtree(this);

      DetectableSceneNode yoloDoorHandleNode = (DetectableSceneNode) sceneGraph.getNamesToNodesMap().get("YOLO door lever");
      StaticRelativeSceneNode staticHandleClosedDoor = (StaticRelativeSceneNode) sceneGraph.getNamesToNodesMap().get(DoorNodeTools.DOOR_HELPER_NODE_NAME_PREFIX);

      boolean shouldClearStaticHandles = false;
      for (WaitDurationActionState action : state.getSetStaticForGraspActions())
         shouldClearStaticHandles |= action.getIsExecuting();
      for (WaitDurationActionState action : state.getSetStaticForApproachActions())
         shouldClearStaticHandles |= action.getIsExecuting();

      if (shouldClearStaticHandles)
      {
//         for (String nodeName : sceneGraph.getNodeNameList())
//         {
//            if (nodeName.startsWith(DoorNodeTools.DOOR_HELPER_NODE_NAME_PREFIX))
//            {
//               if (sceneGraph.getNamesToNodesMap().get(nodeName) instanceof RigidBodySceneNode staticHandleNode)
//               {
//                  staticHandleNode.clearOffset();
//                  staticHandleNode.freeze();
//               }
//            }
//         }
      }

      if (state.arePullRetryNodesPresent())
      {
         // Check that it pulled the door far enough to consider it open and secured with other hand
         if (!state.getPostPullDoorEvaluationAction().getIsExecuting())
         { // Here we are preventing the below logic from triggering more than once at a time
            waitForPullScrewToFinish = false;
         }
         if (!waitForPullScrewToFinish && state.getPostPullDoorEvaluationAction().getIsExecuting())
         {
            if (yoloDoorHandleNode != null)
            {
               double openedDoorHandleDistanceFromStart = definition.getOpenedDoorHandleDistanceFromStart().getValue();
               double distanceHandleFromStart = yoloDoorHandleNode.getNodeToParentFrameTransformReadOnly().getTranslation()
                                                                  .differenceNorm(staticHandleClosedDoor.getNodeToParentFrameTransformReadOnly().getTranslation());
               state.getDoorHandleDistanceFromStart().setValue(distanceHandleFromStart);
               if (state.getDoorHandleDistanceFromStart().getValue() < openedDoorHandleDistanceFromStart)
               {
                  state.getLogger().info("""
                                         Retrying pull door. Distance door handle from start %.2f / %.2f [m].
                                         Stopping all trajectories.
                                         Going back to %s.
                                         """.formatted(state.getDoorHandleDistanceFromStart().getValue(), openedDoorHandleDistanceFromStart, state.getWaitToOpenRightHandAction().getDefinition().getName()));
                  ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
                  waitForPullScrewToFinish = true;
                  state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getLeafIndex());
               }
            }
         }

         // Check that it grasped the door handle effectively, evaluate distance hand-handle at the end of grasp action
         if (!state.getPostGraspEvaluationAction().getIsExecuting())
         {
            waitForGraspToFinish = false;
         }
         if (!waitForGraspToFinish && state.getPostGraspEvaluationAction().getIsExecuting())
         {
            if (staticHandleClosedDoor != null)
            {
               double handToHandleDistance = syncedRobot.getFullRobotModel().getHandControlFrame(RobotSide.RIGHT).
                     getTransformToDesiredFrame(staticHandleClosedDoor.getNodeFrame()).getTranslation().norm();
               if (handToHandleDistance > 0.19)
               {
                  state.getLogger().info("""
                                      Retrying reach door handle. Distance hand to door handle %.2f / %.2f [m].
                                      Stopping all trajectories.
                                      Going back to %s.
                                      """.formatted(handToHandleDistance, 0.19, state.getWaitToOpenRightHandAction().getDefinition().getName()));
                  ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
                  waitForGraspToFinish = true;
                  state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getLeafIndex());
               }
            }
         }
      }
   }

   public void updateSubtree(BehaviorTreeNodeExecutor<?, ?> node)
   {
      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         if (child instanceof ActionNodeExecutor<?, ?> actionNode)
         {

         }
         else
         {
            updateSubtree(child);
         }
      }
   }
}
