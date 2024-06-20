package us.ihmc.behaviors.behaviorTree.trashCan;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class TrashCanInteractionExecutor extends BehaviorTreeNodeExecutor<TrashCanInteractionState, TrashCanInteractionDefinition>
{
   private final TrashCanInteractionState state;
   private final TrashCanInteractionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;

   public TrashCanInteractionExecutor(long id,
                                      CRDTInfo crdtInfo,
                                      WorkspaceResourceDirectory saveFileDirectory,
                                      ROS2ControllerHelper ros2ControllerHelper,
                                      ROS2SyncedRobotModel syncedRobot,
                                      SceneGraph sceneGraph)
   {
      super(new TrashCanInteractionState(id, crdtInfo, saveFileDirectory));

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

      if (state.areLogicNodesPresent())
      {
         // compute stance
         if (state.getComputeStanceAction().getIsExecuting())
         {
            String obstructedNodeName = definition.getObstructedNode().getValue();
            if (!obstructedNodeName.isEmpty())
            {
               if (obstructedNodeName.contains(DoorSceneNodeDefinitions.RIGHT_DOOR_PANEL_NAME))
               {
                  state.getStance().setValue(InteractionStance.RIGHT);
               }
               else if (obstructedNodeName.contains(DoorSceneNodeDefinitions.LEFT_DOOR_PANEL_NAME))
               {
                  state.getStance().setValue(InteractionStance.LEFT);
               }
               else if (obstructedNodeName.contains(RigidBodySceneObjectDefinitions.COUCH_NAME))
               {
                  state.getStance().setValue(InteractionStance.FRONT);
               }
            }
            else
            {
               state.getStance().setValue(InteractionStance.FRONT);
            }
         }

         // skip action if not related to the selected stance
         if (state.getApproachingLeftAction().getIsExecuting() && state.getStance().getValue() != InteractionStance.LEFT)
         {
            int nextNextIndex = state.getApproachingFrontAction().getActionIndex();
            state.getActionSequence().setExecutionNextIndex(nextNextIndex);
         }
         else if (state.getApproachingFrontAction().getIsExecuting() && state.getStance().getValue() != InteractionStance.FRONT)
         {
            int nextNextIndex = state.getApproachingRightAction().getActionIndex();
            state.getActionSequence().setExecutionNextIndex(nextNextIndex);
         }

         if (state.getSetLeftFootDownAction().getIsExecuting())
         {
            int nextNextIndex = state.getEndAction().getActionIndex();
            state.getActionSequence().setExecutionNextIndex(nextNextIndex);
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
