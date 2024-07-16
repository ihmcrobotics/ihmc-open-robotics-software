package us.ihmc.behaviors.buildingExploration;

import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphClearSubtree;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeRemoval;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
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

      boolean shouldClearStaticHandle = false;
      shouldClearStaticHandle |= state.getSetStaticForApproachAction().getIsExecuting();
      shouldClearStaticHandle |= state.getSetStaticForGraspAction().getIsExecuting();
      shouldClearStaticHandle |= state.getSetStaticForApproachActionPush().getIsExecuting();
      shouldClearStaticHandle |= state.getSetStaticForGraspActionPush().getIsExecuting();
      shouldClearStaticHandle |= state.getSetStaticForApproachActionPull().getIsExecuting();
      shouldClearStaticHandle |= state.getSetStaticForGraspActionPull().getIsExecuting();
      shouldClearStaticHandle |= state.getSetStaticForApproachActionTrash().getIsExecuting();

      if (shouldClearStaticHandle)
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
      if ( (state.getStartDemoAction().getIsExecuting() ||
            state.getEndScanAction().getIsExecuting() ||
            state.getEndScanAction().getIsExecuting() ||
            state.getEndPushDoorAction().getIsExecuting() ||
            state.getEndPullDoorAction().getIsExecuting() ||
            state.getEndTrashCanAction().getIsExecuting() ||
            state.getEndCouchAction().getIsExecuting() ||
            state.getEndTableLeftAction().getIsExecuting() ||
            state.getEndTableRightAction().getIsExecuting() ||
            state.getEndTableRightAction().getIsExecuting() ||
            state.getEndWalkDoorAAction().getIsExecuting() ||
            state.getEndWalkDoorBAction().getIsExecuting() ||
            state.getEndTurnDoorAAction().getIsExecuting() ||
            state.getEndTurnDoorBAction().getIsExecuting() ||
            state.getEndWalkCouchAction().getIsExecuting()) )
      {
         for (String nodeName : sceneGraph.getNodeNameList())
         {
            if (nodeName.startsWith("tom"))
            {
               tomDetected = true;
               state.getActionSequence().setConcurrencyEnabled(false);
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
         if (state.getEndFirstDoorAction().getIsExecuting())
         {
            doorTraversed.put("First", true);
         }

         if (state.getStartPullDoorAction().getIsExecuting() ||
             state.getStartPushDoorAction().getIsExecuting() ||
             state.getStartScanAction().getIsExecuting() ||
             state.getStartTableLeftAction().getIsExecuting() ||
             state.getStartTableRightAction().getIsExecuting() ||
             state.getStartTrashCanAction().getIsExecuting() ||
             state.getStartCouchAction().getIsExecuting() ||
             state.getWalkDoorBAction().getIsExecuting() ||
             state.getTurnDoorAAction().getIsExecuting() ||
             state.getTurnDoorBAction().getIsExecuting() ||
             state.getWalkCouchAction().getIsExecuting() ||
             state.getStartSaluteAction().getIsExecuting())
         {
            state.getActionSequence().setConcurrencyEnabled(true);
         }

         if (state.getStartPushDoorAction().getIsExecuting() || state.getStartPullDoorAction().getIsExecuting())
         {
            // Remove the door nodes
            sceneGraph.modifyTree(modificationQueue -> {
               for (String nodeName : sceneGraph.getNodeNameList())
               {
                  if (nodeName.startsWith(DoorNodeTools.DOOR_HELPER_NODE_NAME_PREFIX))
                  {
                     if (sceneGraph.getNamesToNodesMap().get(nodeName) instanceof PredefinedRigidBodySceneNode staticHandleNode)
                     {
                        // Remove the door node from the scene graph
                        modificationQueue.accept(new SceneGraphClearSubtree(staticHandleNode));
                        modificationQueue.accept(new SceneGraphNodeRemoval(sceneGraph.getIDToNodeMap().get( staticHandleNode.getInitialParentNodeID()), sceneGraph));

                        // Remove the static handle node from the scene graph
                        modificationQueue.accept(new SceneGraphClearSubtree(staticHandleNode));
                        modificationQueue.accept(new SceneGraphNodeRemoval(staticHandleNode, sceneGraph));
                     }
                  }
               }
            });
         }

         if (state.getDisableDoorAction().getIsExecuting())
         {
            sceneGraph.setAllowNewDoorNodes(false);
         }

         if (state.getEnableDoorAction().getIsExecuting())
         {
            sceneGraph.setAllowNewDoorNodes(true);
         }

         // PULL DOOR or TRASH CAN after WALK to pull door
         if (state.getEndWalkDoorAAction().getIsExecuting())
         {
            boolean isTrashCanPresent = false;
            doorTraversed.put("A", true);
            for (String nodeName : sceneGraph.getNodeNameList())
            {
               if (nodeName.startsWith("trash"))
               {
                  state.getActionSequence().setConcurrencyEnabled(false);
                  state.getActionSequence().setExecutionNextIndex(state.getStartTrashCanAction().getActionIndex());
                  isTrashCanPresent = true;
                  break;
               }
            }
            if (!isTrashCanPresent)
            {
               state.getActionSequence().setConcurrencyEnabled(false);
               state.getActionSequence().setExecutionNextIndex(state.getStartPullDoorAction().getActionIndex());
            }
            // Remove the door nodes
            sceneGraph.modifyTree(modificationQueue -> {
               for (String nodeName : sceneGraph.getNodeNameList())
               {
                  if (nodeName.startsWith(DoorNodeTools.DOOR_HELPER_NODE_NAME_PREFIX))
                  {
                     if (sceneGraph.getNamesToNodesMap().get(nodeName) instanceof PredefinedRigidBodySceneNode staticHandleNode)
                     {
                        // Remove the door node from the scene graph
                        modificationQueue.accept(new SceneGraphClearSubtree(staticHandleNode));
                        modificationQueue.accept(new SceneGraphNodeRemoval(sceneGraph.getIDToNodeMap().get( staticHandleNode.getInitialParentNodeID()), sceneGraph));

                        // Remove the static handle node from the scene graph
                        modificationQueue.accept(new SceneGraphClearSubtree(staticHandleNode));
                        modificationQueue.accept(new SceneGraphNodeRemoval(staticHandleNode, sceneGraph));
                     }
                  }
               }
            });
         }

         if (state.getEndWalkDoorBAction().getIsExecuting())
         {
            boolean isTrashCanPresent = false;
            doorTraversed.put("B", true);
            for (String nodeName : sceneGraph.getNodeNameList())
            {
               if (nodeName.startsWith("trash"))
               {
                  state.getActionSequence().setConcurrencyEnabled(false);
                  state.getActionSequence().setExecutionNextIndex(state.getStartTrashCanAction().getActionIndex());
                  isTrashCanPresent = true;
                  break;
               }
            }
            if (!isTrashCanPresent)
            {
               state.getActionSequence().setConcurrencyEnabled(false);
               state.getActionSequence().setExecutionNextIndex(state.getStartPullDoorAction().getActionIndex());
            }
            // Remove the door nodes
            sceneGraph.modifyTree(modificationQueue -> {
               for (String nodeName : sceneGraph.getNodeNameList())
               {
                  if (nodeName.startsWith(DoorNodeTools.DOOR_HELPER_NODE_NAME_PREFIX))
                  {
                     if (sceneGraph.getNamesToNodesMap().get(nodeName) instanceof PredefinedRigidBodySceneNode staticHandleNode)
                     {
                        // Remove the door node from the scene graph
                        modificationQueue.accept(new SceneGraphClearSubtree(staticHandleNode));
                        modificationQueue.accept(new SceneGraphNodeRemoval(sceneGraph.getIDToNodeMap().get( staticHandleNode.getInitialParentNodeID()), sceneGraph));

                        // Remove the static handle node from the scene graph
                        modificationQueue.accept(new SceneGraphClearSubtree(staticHandleNode));
                        modificationQueue.accept(new SceneGraphNodeRemoval(staticHandleNode, sceneGraph));
                     }
                  }
               }
            });
         }

         // PULL DOOR after TRASHCAN
         if (state.getEndTrashCanAction().getIsExecuting())
         {
            state.getActionSequence().setConcurrencyEnabled(false);
            state.getActionSequence().setExecutionNextIndex(state.getStartPullDoorAction().getActionIndex());
         }
         // PUSH DOOR after Turn to face door
         if (state.getEndTurnDoorAAction().getIsExecuting() || state.getEndTurnDoorBAction().getIsExecuting())
         {
            state.getActionSequence().setConcurrencyEnabled(false);
            state.getActionSequence().setExecutionNextIndex(state.getStartPushDoorAction().getActionIndex());
         }
         // SCAN after PULL DOOR
         if (state.getEndPullDoorAction().getIsExecuting())
         {
            state.getActionSequence().setConcurrencyEnabled(false);
            state.getActionSequence().setExecutionNextIndex(state.getStartScanAction().getActionIndex());
         }
         // after SCAN
         if (state.getEndScanAction().getIsExecuting() && doorTraversed.get("A"))
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
                     state.getActionSequence().setConcurrencyEnabled(false);
                     state.getActionSequence().setExecutionNextIndex(state.getStartTableRightAction().getActionIndex());
                  }
                  else
                  {
                     state.getActionSequence().setConcurrencyEnabled(false);
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
               {
                  state.getActionSequence().setConcurrencyEnabled(false);
                  state.getActionSequence().setExecutionNextIndex(state.getTurnDoorBAction().getActionIndex());
               }
               else
               {
                  state.getActionSequence().setConcurrencyEnabled(false);
                  state.getActionSequence().setExecutionNextIndex(state.getTurnDoorAAction().getActionIndex());
               }
            }
         }
         // TURN after TABLE
         if (state.getEndTableRightAction().getIsExecuting() || state.getEndTableLeftAction().getIsExecuting() )
         {
            state.getActionSequence().setConcurrencyEnabled(false);
            state.getActionSequence().setExecutionNextIndex(state.getEndTurnDoorAAction().getActionIndex());
         }

         //// USING THIS TO EARLY TERMINATION FOR TESTING
         if (state.getEndPushDoorAction().getIsExecuting())
         {
            state.getActionSequence().setConcurrencyEnabled(false);
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
         tomDetected = false;
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
