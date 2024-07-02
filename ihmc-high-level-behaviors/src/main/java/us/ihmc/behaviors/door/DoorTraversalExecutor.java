package us.ihmc.behaviors.door;

import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNodeTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   private final DoorTraversalState state;
   private final DoorTraversalDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;

   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
   private boolean waitForPullScrewToFinish = false;
   private boolean waitForGraspToFinish = false;

   public DoorTraversalExecutor(long id,
                                CRDTInfo crdtInfo,
                                WorkspaceResourceDirectory saveFileDirectory,
                                ROS2ControllerHelper ros2ControllerHelper,
                                ROS2SyncedRobotModel syncedRobot,
                                SceneGraph sceneGraph)
   {
      super(new DoorTraversalState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.sceneGraph = sceneGraph;
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

      updateActionSubtree(this);

      DetectableSceneNode yoloDoorHandleNode = (DetectableSceneNode) sceneGraph.getNamesToNodesMap().get("YOLO door lever");
      StaticRelativeSceneNode staticHandleClosedDoor = (StaticRelativeSceneNode) sceneGraph.getNamesToNodesMap().get(DoorNodeTools.DOOR_STATIC_HANDLE_NAME);

      if (state.getSetStaticForGraspAction() != null && state.getSetStaticForGraspAction().getIsExecuting() ||
          state.getSetStaticForApproachAction() != null && state.getSetStaticForApproachAction().getIsExecuting())
      {
         for (String nodeName : sceneGraph.getNodeNameList())
         {
            if (nodeName.startsWith(DoorNodeTools.DOOR_STATIC_HANDLE_NAME))
            {
               if (sceneGraph.getNamesToNodesMap().get(nodeName) instanceof StaticRelativeSceneNode staticHandleNode)
               {
                  staticHandleNode.clearOffset();
                  staticHandleNode.freeze();
               }
            }
         }
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
               double openedDoorHandleDistanceFromStart = getDefinition().getOpenedDoorHandleDistanceFromStart().getValue();
               double distanceHandleFromStart = yoloDoorHandleNode.getNodeToParentFrameTransform().getTranslation()
                                                                  .differenceNorm(staticHandleClosedDoor.getNodeToParentFrameTransform().getTranslation());
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
                  state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getActionIndex());
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
                  state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getActionIndex());
               }
            }
         }
      }
   }

   public void updateActionSubtree(BehaviorTreeNodeExecutor<?, ?> node)
   {
      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         if (child instanceof ActionNodeExecutor<?, ?> actionNode)
         {

         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }
}
