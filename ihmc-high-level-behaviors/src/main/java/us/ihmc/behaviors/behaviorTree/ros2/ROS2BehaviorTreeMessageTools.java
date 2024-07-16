package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionDefinition;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionState;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationState;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.door.DoorTraversalState;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.actions.*;

/**
 * All the stuff that for packing/unpacking the specific types goes in here
 * to keep the other classes clean. It also gives one place to add/remove/edit
 * the specific types.
 */
public class ROS2BehaviorTreeMessageTools
{
   public static void clearLists(BehaviorTreeStateMessage treeStateMessage)
   {
      treeStateMessage.getBehaviorTreeTypes().clear();
      treeStateMessage.getBehaviorTreeIndices().clear();
      treeStateMessage.getRootNodes().clear();
      treeStateMessage.getBasicNodes().clear();
      treeStateMessage.getActionSequences().clear();
      treeStateMessage.getDoorTraversals().clear();
      treeStateMessage.getTrashCanInteractions().clear();
      treeStateMessage.getBuildingExplorations().clear();
      treeStateMessage.getChestOrientationActions().clear();
      treeStateMessage.getFootstepPlanActions().clear();
      treeStateMessage.getHandPoseActions().clear();
      treeStateMessage.getHandWrenchActions().clear();
      treeStateMessage.getScrewPrimitiveActions().clear();
      treeStateMessage.getPelvisHeightActions().clear();
      treeStateMessage.getSakeHandCommandActions().clear();
      treeStateMessage.getWaitDurationActions().clear();
      treeStateMessage.getFootPoseActions().clear();
   }

   public static void packMessage(BehaviorTreeNodeState nodeState, BehaviorTreeStateMessage treeStateMessage)
   {
      boolean packBasicNode = false;

      // Only allow packing full node types if we have updated data
      if (nodeState.getDefinition().isFrozen() || nodeState.hasStatus())
      {
         if (nodeState instanceof BehaviorTreeRootNodeState rootNodeState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.ROOT_NODE);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getRootNodes().size());
            rootNodeState.toMessage(treeStateMessage.getRootNodes().add());
         }
         else if (nodeState instanceof ActionSequenceState actionSequenceState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.ACTION_SEQUENCE);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getActionSequences().size());
            actionSequenceState.toMessage(treeStateMessage.getActionSequences().add());
         }
         else if (nodeState instanceof DoorTraversalState doorTraversalState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.DOOR_TRAVERSAL);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getDoorTraversals().size());
            doorTraversalState.toMessage(treeStateMessage.getDoorTraversals().add());
         }
         else if (nodeState instanceof TrashCanInteractionState trashCanInteractionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.TRASH_CAN_INTERACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getTrashCanInteractions().size());
            trashCanInteractionState.toMessage(treeStateMessage.getTrashCanInteractions().add());
         }
         else if (nodeState instanceof BuildingExplorationState buildingExplorationState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.BUILDING_EXPLORATION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getDoorTraversals().size());
            buildingExplorationState.toMessage(treeStateMessage.getBuildingExplorations().add());
         }
         else if (nodeState instanceof ChestOrientationActionState chestOrientationActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.CHEST_ORIENTATION_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getChestOrientationActions().size());
            chestOrientationActionState.toMessage(treeStateMessage.getChestOrientationActions().add());
         }
         else if (nodeState instanceof FootstepPlanActionState footstepPlanActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.FOOTSTEP_PLAN_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getFootstepPlanActions().size());
            footstepPlanActionState.toMessage(treeStateMessage.getFootstepPlanActions().add());
         }
         else if (nodeState instanceof SakeHandCommandActionState sakeHandCommandActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.SAKE_HAND_COMMAND_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getSakeHandCommandActions().size());
            sakeHandCommandActionState.toMessage(treeStateMessage.getSakeHandCommandActions().add());
         }
         else if (nodeState instanceof HandPoseActionState handPoseActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.HAND_POSE_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getHandPoseActions().size());
            handPoseActionState.toMessage(treeStateMessage.getHandPoseActions().add());
         }
         else if (nodeState instanceof HandWrenchActionState handWrenchActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.HAND_WRENCH_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getHandWrenchActions().size());
            handWrenchActionState.toMessage(treeStateMessage.getHandWrenchActions().add());
         }
         else if (nodeState instanceof ScrewPrimitiveActionState screwPrimitiveActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.SCREW_PRIMITIVE_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getScrewPrimitiveActions().size());
            screwPrimitiveActionState.toMessage(treeStateMessage.getScrewPrimitiveActions().add());
         }
         else if (nodeState instanceof PelvisHeightOrientationActionState pelvisHeightActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.PELVIS_HEIGHT_ORIENTATION_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getPelvisHeightActions().size());
            pelvisHeightActionState.toMessage(treeStateMessage.getPelvisHeightActions().add());
         }
         else if (nodeState instanceof WaitDurationActionState waitDurationActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.WAIT_DURATION_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getWaitDurationActions().size());
            waitDurationActionState.toMessage(treeStateMessage.getWaitDurationActions().add());
         }
         else if (nodeState instanceof FootPoseActionState footPoseActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.FOOT_POSE_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getFootPoseActions().size());
            footPoseActionState.toMessage(treeStateMessage.getFootPoseActions().add());
         }
         else
         {
            packBasicNode = true;
         }
      }
      else
      {
         packBasicNode = true;
      }

      if (packBasicNode)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.BASIC_NODE);
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getBasicNodes().size());
         BasicNodeStateMessage basicNodeMessage = treeStateMessage.getBasicNodes().add();
         nodeState.toMessage(basicNodeMessage.getState());
         nodeState.getDefinition().toMessage(basicNodeMessage.getDefinition());
      }
   }

   public static void fromMessage(ROS2BehaviorTreeSubscriptionNode subscriptionNode, BehaviorTreeNodeState<?> nodeState)
   {
      // Here we check that the incoming data has the full type data as well as that the local node is of that type
      // When the full data is not necessary to send, we only send the basic node information
      if (subscriptionNode.getType() == BehaviorTreeRootNodeDefinition.class && nodeState instanceof BehaviorTreeRootNodeState rootNodeState)
      {
         rootNodeState.fromMessage(subscriptionNode.getBehaviorTreeRootNodeStateMessage());
      }
      else if (subscriptionNode.getType() == ActionSequenceDefinition.class && nodeState instanceof ActionSequenceState actionSequenceState)
      {
         actionSequenceState.fromMessage(subscriptionNode.getActionSequenceStateMessage());
      }
      else if (subscriptionNode.getType() == DoorTraversalDefinition.class && nodeState instanceof DoorTraversalState doorTraversalState)
      {
         doorTraversalState.fromMessage(subscriptionNode.getDoorTraversalStateMessage());
      }
      else if (subscriptionNode.getType() == TrashCanInteractionDefinition.class && nodeState instanceof TrashCanInteractionState trashCanInteractionState)
      {
         trashCanInteractionState.fromMessage(subscriptionNode.getTrashCanInteractionStateMessage());
      }
      else if (subscriptionNode.getType() == BuildingExplorationDefinition.class && nodeState instanceof BuildingExplorationState buildingExplorationState)
      {
         buildingExplorationState.fromMessage(subscriptionNode.getBuildingExplorationStateMessage());
      }
      else if (subscriptionNode.getType() == ChestOrientationActionDefinition.class && nodeState instanceof ChestOrientationActionState chestOrientationActionState)
      {
         chestOrientationActionState.fromMessage(subscriptionNode.getChestOrientationActionStateMessage());
      }
      else if (subscriptionNode.getType() == FootstepPlanActionDefinition.class && nodeState instanceof FootstepPlanActionState footstepPlanActionState)
      {
         footstepPlanActionState.fromMessage(subscriptionNode.getFootstepPlanActionStateMessage());
      }
      else if (subscriptionNode.getType() == SakeHandCommandActionDefinition.class && nodeState instanceof SakeHandCommandActionState sakeHandCommandActionState)
      {
         sakeHandCommandActionState.fromMessage(subscriptionNode.getSakeHandCommandActionStateMessage());
      }
      else if (subscriptionNode.getType() == HandPoseActionDefinition.class && nodeState instanceof HandPoseActionState handPoseActionState)
      {
         handPoseActionState.fromMessage(subscriptionNode.getHandPoseActionStateMessage());
      }
      else if (subscriptionNode.getType() == HandWrenchActionDefinition.class && nodeState instanceof HandWrenchActionState handWrenchActionState)
      {
         handWrenchActionState.fromMessage(subscriptionNode.getHandWrenchActionStateMessage());
      }
      else if (subscriptionNode.getType() == ScrewPrimitiveActionDefinition.class && nodeState instanceof ScrewPrimitiveActionState screwPrimitiveActionState)
      {
         screwPrimitiveActionState.fromMessage(subscriptionNode.getScrewPrimitiveActionStateMessage());
      }
      else if (subscriptionNode.getType() == PelvisHeightOrientationActionDefinition.class && nodeState instanceof PelvisHeightOrientationActionState pelvisHeightActionState)
      {
         pelvisHeightActionState.fromMessage(subscriptionNode.getPelvisHeightOrientationActionStateMessage());
      }
      else if (subscriptionNode.getType() == WaitDurationActionDefinition.class && nodeState instanceof WaitDurationActionState waitDurationActionState)
      {
         waitDurationActionState.fromMessage(subscriptionNode.getWaitDurationActionStateMessage());
      }
      else if (subscriptionNode.getType() == FootPoseActionDefinition.class && nodeState instanceof FootPoseActionState footPoseActionState)
      {
         footPoseActionState.fromMessage(subscriptionNode.getFootPoseActionStateMessage());
      }
      else
      {
         nodeState.fromMessage(subscriptionNode.getBehaviorTreeNodeStateMessage());
         nodeState.getDefinition().fromMessage(subscriptionNode.getBehaviorTreeNodeDefinitionMessage());
      }
   }

   public static void packSubscriptionNode(byte nodeType,
                                           int indexInTypesList,
                                           BehaviorTreeStateMessage treeStateMessage,
                                           ROS2BehaviorTreeSubscriptionNode subscriptionNode)
   {
      switch (nodeType)
      {
         case BehaviorTreeStateMessage.BASIC_NODE ->
         {
            BasicNodeStateMessage basicNodeStateMessage = treeStateMessage.getBasicNodes().get(indexInTypesList);
            subscriptionNode.setBehaviorTreeNodeStateMessage(basicNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(basicNodeStateMessage.getDefinition());
         }
         case BehaviorTreeStateMessage.ROOT_NODE ->
         {
            BehaviorTreeRootNodeStateMessage rootNodeStateMessage = treeStateMessage.getRootNodes().get(indexInTypesList);
            subscriptionNode.setBehaviorTreeRootNodeStateMessage(rootNodeStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(rootNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(rootNodeStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.ACTION_SEQUENCE ->
         {
            ActionSequenceStateMessage actionSequenceStateMessage = treeStateMessage.getActionSequences().get(indexInTypesList);
            subscriptionNode.setActionSequenceStateMessage(actionSequenceStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(actionSequenceStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(actionSequenceStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.DOOR_TRAVERSAL ->
         {
            DoorTraversalStateMessage doorTraversalStateMessage = treeStateMessage.getDoorTraversals().get(indexInTypesList);
            subscriptionNode.setDoorTraversalStateMessage(doorTraversalStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(doorTraversalStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(doorTraversalStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.TRASH_CAN_INTERACTION ->
         {
            TrashCanInteractionStateMessage trashCanInteractionStateMessage = treeStateMessage.getTrashCanInteractions().get(indexInTypesList);
            subscriptionNode.setTrashCanInteractionStateMessage(trashCanInteractionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(trashCanInteractionStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(trashCanInteractionStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.BUILDING_EXPLORATION ->
         {
            BuildingExplorationStateMessage buildingExplorationStateMessage = treeStateMessage.getBuildingExplorations().get(indexInTypesList);
            subscriptionNode.setBuildingExplorationStateMessage(buildingExplorationStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(buildingExplorationStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(buildingExplorationStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.CHEST_ORIENTATION_ACTION ->
         {
            ChestOrientationActionStateMessage chestOrientationActionStateMessage = treeStateMessage.getChestOrientationActions().get(indexInTypesList);
            subscriptionNode.setChestOrientationActionStateMessage(chestOrientationActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(chestOrientationActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(chestOrientationActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.FOOTSTEP_PLAN_ACTION ->
         {
            FootstepPlanActionStateMessage footstepPlanActionStateMessage = treeStateMessage.getFootstepPlanActions().get(indexInTypesList);
            subscriptionNode.setFootstepPlanActionStateMessage(footstepPlanActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(footstepPlanActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(footstepPlanActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.HAND_POSE_ACTION ->
         {
            HandPoseActionStateMessage handPoseActionStateMessage = treeStateMessage.getHandPoseActions().get(indexInTypesList);
            subscriptionNode.setHandPoseActionStateMessage(handPoseActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(handPoseActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(handPoseActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.HAND_WRENCH_ACTION ->
         {
            HandWrenchActionStateMessage handWrenchActionStateMessage = treeStateMessage.getHandWrenchActions().get(indexInTypesList);
            subscriptionNode.setHandWrenchActionStateMessage(handWrenchActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(handWrenchActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(handWrenchActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.SCREW_PRIMITIVE_ACTION ->
         {
            ScrewPrimitiveActionStateMessage screwPrimitiveActionStateMessage = treeStateMessage.getScrewPrimitiveActions().get(indexInTypesList);
            subscriptionNode.setScrewPrimitiveActionStateMessage(screwPrimitiveActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(screwPrimitiveActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(screwPrimitiveActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.PELVIS_HEIGHT_ORIENTATION_ACTION ->
         {
            PelvisHeightOrientationActionStateMessage pelvisHeightOrientationActionStateMessage = treeStateMessage.getPelvisHeightActions().get(indexInTypesList);
            subscriptionNode.setPelvisHeightOrientationActionStateMessage(pelvisHeightOrientationActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(pelvisHeightOrientationActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(pelvisHeightOrientationActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.SAKE_HAND_COMMAND_ACTION ->
         {
            SakeHandCommandActionStateMessage sakeHandCommandActionStateMessage = treeStateMessage.getSakeHandCommandActions().get(indexInTypesList);
            subscriptionNode.setSakeHandCommandActionStateMessage(sakeHandCommandActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(sakeHandCommandActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(sakeHandCommandActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.WAIT_DURATION_ACTION ->
         {
            WaitDurationActionStateMessage waitDurationActionStateMessage = treeStateMessage.getWaitDurationActions().get(indexInTypesList);
            subscriptionNode.setWaitDurationActionStateMessage(waitDurationActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(waitDurationActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(waitDurationActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.FOOT_POSE_ACTION ->
         {
            FootPoseActionStateMessage footPoseActionStateMessage = treeStateMessage.getFootPoseActions().get(indexInTypesList);
            subscriptionNode.setFootPoseActionStateMessage(footPoseActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(footPoseActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(footPoseActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
      }
   }
}
