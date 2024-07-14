package us.ihmc.behaviors.buildingExploration;

import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNodeTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.HashMap;
import java.util.Map;

public class BuildingExplorationExecutor extends BehaviorTreeNodeExecutor<BuildingExplorationState, BuildingExplorationDefinition>
{
   private final BuildingExplorationState state;
   private final BuildingExplorationDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;
   boolean tomDetected = false;
   private final Map<String, Boolean> doorTraversed = new HashMap<>();

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
      doorTraversed.put("First", false);
      doorTraversed.put("A", false);
      doorTraversed.put("B", false);
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
         for (String nodeName : sceneGraph.getNodeNameList())
         {
            if (nodeName.startsWith("tom"))
            {
               tomDetected = true;
               state.getActionSequence().setExecutionNextIndex(state.getStartSaluteAction().getActionIndex());
               doorTraversed.put("First", false);
               doorTraversed.put("A", false);
               doorTraversed.put("B", false);
               break;
            }
         }
      }

      if (!tomDetected)
      {
         if (state.getEndFirstDoorAction().getIsNextForExecution())
         {
            doorTraversed.put("First", true);
         }
         // PULL DOOR after WALK to pull door
         if (state.getEndWalkDoorAAction().getIsNextForExecution())
         {
            doorTraversed.put("A", true);
            state.getActionSequence().setExecutionNextIndex(state.getStartPullDoorAction().getActionIndex());
         }
         if (state.getEndWalkDoorBAction().getIsNextForExecution())
         {
            doorTraversed.put("B", true);
            state.getActionSequence().setExecutionNextIndex(state.getStartPullDoorAction().getActionIndex());
         }

         // PULL DOOR after TRASHCAN
         if (state.getEndTrashCanAction().getIsNextForExecution())
         {
            state.getActionSequence().setExecutionNextIndex(state.getStartPullDoorAction().getActionIndex());
         }
         // PUSH DOOR after Turn to face door
         if (state.getEndTurnDoorAAction().getIsNextForExecution() || state.getEndTurnDoorBAction().getIsNextForExecution())
         {
            state.getActionSequence().setExecutionNextIndex(state.getStartPushDoorAction().getActionIndex());
         }
         // SCAN after PULL DOOR
         if (state.getEndPullDoorAction().getIsNextForExecution())
         {
            state.getActionSequence().setExecutionNextIndex(state.getStartScanAction().getActionIndex());
         }
         // after SCAN
         if (state.getEndScanAction().getIsNextForExecution() && doorTraversed.get("A"))
         {
            boolean isTableDetected = false;
            for (String nodeName : sceneGraph.getNodeNameList())
            {
               // TABLE if there's a Table
               if (nodeName.startsWith("table"))
               {
                  RigidBodyTransform transformTableToRobotMidFeetFrame = sceneGraph.getNamesToNodesMap()
                                                                                   .get(nodeName)
                                                                                   .getNodeFrame()
                                                                                   .getTransformToDesiredFrame(syncedRobot.getReferenceFrames()
                                                                                                                          .getMidFeetZUpFrame());
                  LogTools.info("Transform Table node - midFeetZUp {}", transformTableToRobotMidFeetFrame.getTranslationY());
                  // TABLE RIGHT or LEFT according to where the table is
                  if (transformTableToRobotMidFeetFrame.getTranslationY() < 0.0)
                  {
                     state.getActionSequence().setExecutionNextIndex(state.getStartTableRightAction().getActionIndex());
                  }
                  else
                  {
                     state.getActionSequence().setExecutionNextIndex(state.getStartTableLeftAction().getActionIndex());
                  }
                  isTableDetected = true;
                  break;
               }
            }
            // TURN if no table
            if (!isTableDetected)
            {
               if (doorTraversed.get("B"))
                  state.getActionSequence().setExecutionNextIndex(state.getTurnDoorBAction().getActionIndex());
               else
                  state.getActionSequence().setExecutionNextIndex(state.getTurnDoorAAction().getActionIndex());
            }
         }
         // TURN after TABLE
         if (state.getEndTableRightAction().getIsNextForExecution() || state.getEndTableLeftAction().getIsNextForExecution() )
         {
            state.getActionSequence().setExecutionNextIndex(state.getEndTurnDoorAAction().getActionIndex());
         }

         //// USING THIS TO EARLY TERMINATION FOR TESTING
         if (state.getEndPushDoorAction().getIsNextForExecution())
         {
            state.getActionSequence().setExecutionNextIndex(state.getEndDemoAction().getActionIndex());
         }
         // WALK to DOOR B after PUSH DOOR A
         // WALK TO COUCH after PUSH DOOR B
         // COUCH after WALK to COUCH
      }

      // skip salute if it's next for execution and tom is not detected
      if (state.getStartSaluteAction().getIsNextForExecution() && !tomDetected)
      {
         state.getActionSequence().setExecutionNextIndex(state.getEndDemoAction().getActionIndex());
         doorTraversed.put("First", false);
         doorTraversed.put("A", false);
         doorTraversed.put("B", false);
      }
      tomDetected = false;
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
