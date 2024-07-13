package us.ihmc.behaviors.buildingExploration;

import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNodeTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BuildingExplorationExecutor extends BehaviorTreeNodeExecutor<BuildingExplorationState, BuildingExplorationDefinition>
{
   private final BuildingExplorationState state;
   private final BuildingExplorationDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;

   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();

   public BuildingExplorationExecutor(long id,
                                      CRDTInfo crdtInfo,
                                      WorkspaceResourceDirectory saveFileDirectory,
                                      ROS2ControllerHelper ros2ControllerHelper,
                                      ROS2SyncedRobotModel syncedRobot,
                                      SceneGraph sceneGraph)
   {
      super(new BuildingExplorationState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.sceneGraph = sceneGraph;
   }

   // TODO: finish
   @Override
   public void update()
   {
      super.update();

      updateActionSubtree(this);

      boolean shouldClearStaticHandles = false;
      for (WaitDurationActionState action : state.getSetStaticForGraspActions())
         shouldClearStaticHandles |= action.getIsExecuting();
      for (WaitDurationActionState action : state.getSetStaticForApproachActions())
         shouldClearStaticHandles |= action.getIsExecuting();

      if (shouldClearStaticHandles)
      {
         for (String nodeName : sceneGraph.getNodeNameList())
         {
            if (nodeName.startsWith(DoorNodeTools.DOOR_HELPER_NODE_NAME_PREFIX))
            {
               if (sceneGraph.getNamesToNodesMap().get(nodeName) instanceof RigidBodySceneNode staticHandleNode)
               {
                  staticHandleNode.clearOffset();
                  staticHandleNode.freeze();
               }
            }
         }
      }

//      if (!state.getPostGraspEvaluationAction().getIsExecuting())
//      {
//         waitForGraspToFinish = false;
//      }
//      if (!waitForGraspToFinish && state.getPostGraspEvaluationAction().getIsExecuting())
//      {
//         ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
//         waitForGraspToFinish = true;
//         state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getActionIndex());
//      }

      // if any of the behaviors ended and Tom is detected, then jump to salute Tom behavior
      if ( (state.getEndScanAction().getIsNextForExecution() ||
          state.getEndPushDoorAction().getIsNextForExecution() ||
          state.getEndPullDoorAction().getIsNextForExecution() ||
          state.getEndTrashCanAction().getIsNextForExecution() ||
          state.getEndCouchAction().getIsNextForExecution() ||
          state.getEndTableLeftAction().getIsNextForExecution() ||
          state.getEndTableRightAction().getIsNextForExecution() ||
          state.getEndTableRightAction().getIsNextForExecution() ||
          state.getEndWalkDoorAAction().getIsNextForExecution() ||
          state.getEndWalkDoorBAction().getIsNextForExecution() ||
          state.getEndTurnDoorAAction().getIsNextForExecution() ||
          state.getEndTurnDoorBAction().getIsNextForExecution() ||
          state.getEndWalkCouchAction().getIsNextForExecution()))
      {

   
         state.getActionSequence().setExecutionNextIndex(state.getStartSaluteAction().getActionIndex());
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
