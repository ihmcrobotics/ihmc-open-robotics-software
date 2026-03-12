package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeState;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationState;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalState;
import us.ihmc.behaviors.behaviorTree.action.actions.CheckPointNodeState;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.behaviors.behaviorTree.control.GotoNodeState;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceState;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusSE3Trajectory;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * All the stuff that for packing/unpacking the specific types goes in here
 * to keep the other classes clean. It also gives one place to add/remove/edit
 * the specific types.
 */
public class ROS2BehaviorTreeMessageTools
{
   public static void clearLists(BehaviorTreeStateMessage treeStateMessage)
   {
      treeStateMessage.getBehaviorTreeTypes().resetQuick();
      treeStateMessage.getBehaviorTreeIndices().clear();
      treeStateMessage.getPartialDataNodes().clear();
      treeStateMessage.getRootNodes().clear();
      treeStateMessage.getBasicNodes().clear();
      treeStateMessage.getActionSequences().clear();
      treeStateMessage.getFallbackNodes().clear();
      treeStateMessage.getConditionNodes().clear();
      treeStateMessage.getGotoNodes().clear();
      treeStateMessage.getCheckpointNodes().clear();
      treeStateMessage.getSceneActions().clear();
      treeStateMessage.getAi2rNodes().clear();
      treeStateMessage.getDoorTraversals().clear();
      treeStateMessage.getBuildingExplorations().clear();
      treeStateMessage.getNeckActions().clear();
      treeStateMessage.getChestOrientationActions().clear();
      treeStateMessage.getFootstepPlanActions().clear();
      treeStateMessage.getHandPoseActions().clear();
      treeStateMessage.getHandWrenchActions().clear();
      treeStateMessage.getScrewPrimitiveActions().clear();
      treeStateMessage.getPelvisHeightActions().clear();
      treeStateMessage.getAbilityHandActions().clear();
      treeStateMessage.getSakeHandCommandActions().clear();
      treeStateMessage.getWaitDurationActions().clear();
      treeStateMessage.getFootPoseActions().clear();
   }

   public static void packMessage(CRDTInfo crdtInfo, BehaviorTreeNodeState<?> nodeState, BehaviorTreeStateMessage treeStateMessage)
   {
      // Only allow packing full node types if we have updated data
      boolean modificationOutgoing = nodeState.getDefinition().pollNeedSendFullData();
      boolean hasStatus = nodeState.hasStatus();
      if (modificationOutgoing || hasStatus)
      {
         if (modificationOutgoing)
            LogTools.debug("%s: Seq # %d Packing full data: %s outgoing = %b  status = %b"
                                .formatted(crdtInfo.getActorDesignation().name(),
                                           treeStateMessage.getSequenceId(),
                                           nodeState.getDefinition().getName(),
                                           modificationOutgoing,
                                           hasStatus));
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
         else if (nodeState instanceof FallbackNodeState fallbackNodeState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.FALLBACK_NODE);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getFallbackNodes().size());
            fallbackNodeState.toMessage(treeStateMessage.getFallbackNodes().add());
         }
         else if (nodeState instanceof ConditionNodeState conditionNodeState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.CONDITION_NODE);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getConditionNodes().size());
            conditionNodeState.toMessage(treeStateMessage.getConditionNodes().add());
         }
         else if (nodeState instanceof GotoNodeState gotoNodeState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.GOTO_NODE);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getGotoNodes().size());
            gotoNodeState.toMessage(treeStateMessage.getGotoNodes().add());
         }
         else if (nodeState instanceof CheckPointNodeState checkPointNodeState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.CHECKPOINT_NODE);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getCheckpointNodes().size());
            checkPointNodeState.toMessage(treeStateMessage.getCheckpointNodes().add());
         }
         else if (nodeState instanceof SceneActionNodeState sceneActionNodeState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.SCENE_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getSceneActions().size());
            sceneActionNodeState.toMessage(treeStateMessage.getSceneActions().add());
         }
         else if (nodeState instanceof AI2RNodeState ai2rNodeState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.AI2R_NODE);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getAi2rNodes().size());
            ai2rNodeState.toMessage(treeStateMessage.getAi2rNodes().add());
         }
         else if (nodeState instanceof DoorTraversalState doorTraversalState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.DOOR_TRAVERSAL);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getDoorTraversals().size());
            doorTraversalState.toMessage(treeStateMessage.getDoorTraversals().add());
         }
         else if (nodeState instanceof BuildingExplorationState buildingExplorationState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.BUILDING_EXPLORATION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getDoorTraversals().size());
            buildingExplorationState.toMessage(treeStateMessage.getBuildingExplorations().add());
         }
         else if (nodeState instanceof NeckActionState neckActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.NECK_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getNeckActions().size());
            neckActionState.toMessage(treeStateMessage.getNeckActions().add());
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
         else if (nodeState instanceof AbilityHandActionState abilityHandActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.ABILITY_HAND_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getAbilityHandActions().size());
            abilityHandActionState.toMessage(treeStateMessage.getAbilityHandActions().add());
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
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.BASIC_NODE);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getBasicNodes().size());
            BasicNodeStateMessage basicNodeMessage = treeStateMessage.getBasicNodes().add();
            nodeState.toMessage(basicNodeMessage.getState());
            nodeState.getDefinition().toMessage(basicNodeMessage.getDefinition());
         }
      }
      else
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.PARTIAL_DATA);
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getPartialDataNodes().size());
         BasicNodeStateMessage partialDataMessage = treeStateMessage.getPartialDataNodes().add();
         nodeState.toMessage(partialDataMessage.getState());
         nodeState.getDefinition().toMessage(partialDataMessage.getDefinition());
      }
   }

   public static void fromMessage(ROS2BehaviorTreeSubscriptionNode subscriptionNode, BehaviorTreeNodeState<?> nodeState)
   {
      if (nodeState instanceof BehaviorTreeRootNodeState rootNodeState)
      {
         rootNodeState.fromMessage(subscriptionNode.getBehaviorTreeRootNodeStateMessage());
      }
      else if (nodeState instanceof ActionSequenceState actionSequenceState)
      {
         actionSequenceState.fromMessage(subscriptionNode.getActionSequenceStateMessage());
      }
      else if (nodeState instanceof FallbackNodeState fallbackNodeState)
      {
         fallbackNodeState.fromMessage(subscriptionNode.getFallbackNodeStateMessage());
      }
      else if (nodeState instanceof ConditionNodeState conditionNodeState)
      {
         conditionNodeState.fromMessage(subscriptionNode.getConditionNodeStateMessage());
      }
      else if (nodeState instanceof GotoNodeState gotoNodeState)
      {
         gotoNodeState.fromMessage(subscriptionNode.getGotoNodeStateMessage());
      }
      else if (nodeState instanceof CheckPointNodeState checkPointNodeState)
      {
         checkPointNodeState.fromMessage(subscriptionNode.getCheckPointNodeStateMessage());
      }
      else if (nodeState instanceof SceneActionNodeState sceneActionNodeState)
      {
         sceneActionNodeState.fromMessage(subscriptionNode.getSceneActionNodeStateMessage());
      }
      else if (nodeState instanceof AI2RNodeState ai2rNodeState)
      {
         ai2rNodeState.fromMessage(subscriptionNode.getAI2RNodeStateMessage());
      }
      else if (nodeState instanceof DoorTraversalState doorTraversalState)
      {
         doorTraversalState.fromMessage(subscriptionNode.getDoorTraversalStateMessage());
      }
      else if (nodeState instanceof BuildingExplorationState buildingExplorationState)
      {
         buildingExplorationState.fromMessage(subscriptionNode.getBuildingExplorationStateMessage());
      }
      else if (nodeState instanceof NeckActionState neckActionState)
      {
         neckActionState.fromMessage(subscriptionNode.getNeckActionStateMessage());
      }
      else if (nodeState instanceof ChestOrientationActionState chestOrientationActionState)
      {
         chestOrientationActionState.fromMessage(subscriptionNode.getChestOrientationActionStateMessage());
      }
      else if (nodeState instanceof FootstepPlanActionState footstepPlanActionState)
      {
         footstepPlanActionState.fromMessage(subscriptionNode.getFootstepPlanActionStateMessage());
      }
      else if (nodeState instanceof AbilityHandActionState abilityHandActionState)
      {
         abilityHandActionState.fromMessage(subscriptionNode.getAbilityHandActionStateMessage());
      }
      else if (nodeState instanceof SakeHandCommandActionState sakeHandCommandActionState)
      {
         sakeHandCommandActionState.fromMessage(subscriptionNode.getSakeHandCommandActionStateMessage());
      }
      else if (nodeState instanceof HandPoseActionState handPoseActionState)
      {
         handPoseActionState.fromMessage(subscriptionNode.getHandPoseActionStateMessage());
      }
      else if (nodeState instanceof HandWrenchActionState handWrenchActionState)
      {
         handWrenchActionState.fromMessage(subscriptionNode.getHandWrenchActionStateMessage());
      }
      else if (nodeState instanceof ScrewPrimitiveActionState screwPrimitiveActionState)
      {
         screwPrimitiveActionState.fromMessage(subscriptionNode.getScrewPrimitiveActionStateMessage());
      }
      else if (nodeState instanceof PelvisHeightOrientationActionState pelvisHeightActionState)
      {
         pelvisHeightActionState.fromMessage(subscriptionNode.getPelvisHeightOrientationActionStateMessage());
      }
      else if (nodeState instanceof WaitDurationActionState waitDurationActionState)
      {
         waitDurationActionState.fromMessage(subscriptionNode.getWaitDurationActionStateMessage());
      }
      else if (nodeState instanceof FootPoseActionState footPoseActionState)
      {
         footPoseActionState.fromMessage(subscriptionNode.getFootPoseActionStateMessage());
      }
      else // Basic node
      {
         nodeState.getDefinition().fromMessage(subscriptionNode.getBehaviorTreeNodeDefinitionMessage());
         nodeState.fromMessage(subscriptionNode.getBehaviorTreeNodeStateMessage());
      }
   }

   public static void packSubscriptionNode(byte nodeType,
                                           int indexInTypesList,
                                           BehaviorTreeStateMessage treeStateMessage,
                                           ROS2BehaviorTreeSubscriptionNode subscriptionNode)
   {
      switch (nodeType)
      {
         case BehaviorTreeStateMessage.PARTIAL_DATA ->
         {
            BasicNodeStateMessage partialDataStateMessage = treeStateMessage.getPartialDataNodes().get(indexInTypesList);
            subscriptionNode.setBehaviorTreeNodeStateMessage(partialDataStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(partialDataStateMessage.getDefinition());
         }
         case BehaviorTreeStateMessage.ROOT_NODE ->
         {
            BehaviorTreeRootNodeStateMessage rootNodeStateMessage = treeStateMessage.getRootNodes().get(indexInTypesList);
            subscriptionNode.setBehaviorTreeRootNodeStateMessage(rootNodeStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(rootNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(rootNodeStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.BASIC_NODE ->
         {
            BasicNodeStateMessage basicNodeStateMessage = treeStateMessage.getBasicNodes().get(indexInTypesList);
            subscriptionNode.setBehaviorTreeNodeStateMessage(basicNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(basicNodeStateMessage.getDefinition());
         }
         case BehaviorTreeStateMessage.ACTION_SEQUENCE ->
         {
            ActionSequenceStateMessage actionSequenceStateMessage = treeStateMessage.getActionSequences().get(indexInTypesList);
            subscriptionNode.setActionSequenceStateMessage(actionSequenceStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(actionSequenceStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(actionSequenceStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.FALLBACK_NODE ->
         {
            FallbackNodeStateMessage fallbackNodeStateMessage = treeStateMessage.getFallbackNodes().get(indexInTypesList);
            subscriptionNode.setFallbackNodeStateMessage(fallbackNodeStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(fallbackNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(fallbackNodeStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.CONDITION_NODE ->
         {
            ConditionNodeStateMessage conditionNodeStateMessage = treeStateMessage.getConditionNodes().get(indexInTypesList);
            subscriptionNode.setConditionNodeStateMessage(conditionNodeStateMessage);
            subscriptionNode.setLeafNodeStateMessage(conditionNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(conditionNodeStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(conditionNodeStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.GOTO_NODE ->
         {
            GotoNodeStateMessage gotoNodeStateMessage = treeStateMessage.getGotoNodes().get(indexInTypesList);
            subscriptionNode.setGotoNodeStateMessage(gotoNodeStateMessage);
            subscriptionNode.setLeafNodeStateMessage(gotoNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(gotoNodeStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(gotoNodeStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.CHECKPOINT_NODE ->
         {
            CheckPointNodeStateMessage checkPointNodeStateMessage = treeStateMessage.getCheckpointNodes().get(indexInTypesList);
            subscriptionNode.setCheckPointNodeStateMessage(checkPointNodeStateMessage);
            subscriptionNode.setLeafNodeStateMessage(checkPointNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(checkPointNodeStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(checkPointNodeStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.SCENE_ACTION ->
         {
            SceneActionNodeStateMessage sceneActionNodeStateMessage = treeStateMessage.getSceneActions().get(indexInTypesList);
            subscriptionNode.setSceneActionNodeStateMessage(sceneActionNodeStateMessage);
            subscriptionNode.setActionNodeStateMessage(sceneActionNodeStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(sceneActionNodeStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(sceneActionNodeStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(sceneActionNodeStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.AI2R_NODE ->
         {
            AI2RNodeStateMessage ai2rNodeStateMessage = treeStateMessage.getAi2rNodes().get(indexInTypesList);
            subscriptionNode.setAI2RNodeStateMessage(ai2rNodeStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(ai2rNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(ai2rNodeStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.DOOR_TRAVERSAL ->
         {
            DoorTraversalStateMessage doorTraversalStateMessage = treeStateMessage.getDoorTraversals().get(indexInTypesList);
            subscriptionNode.setDoorTraversalStateMessage(doorTraversalStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(doorTraversalStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(doorTraversalStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.BUILDING_EXPLORATION ->
         {
            BuildingExplorationStateMessage buildingExplorationStateMessage = treeStateMessage.getBuildingExplorations().get(indexInTypesList);
            subscriptionNode.setBuildingExplorationStateMessage(buildingExplorationStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(buildingExplorationStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(buildingExplorationStateMessage.getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.NECK_ACTION ->
         {
            NeckActionStateMessage neckActionStateMessage = treeStateMessage.getNeckActions().get(indexInTypesList);
            subscriptionNode.setNeckActionStateMessage(neckActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(neckActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(neckActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(neckActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(neckActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.CHEST_ORIENTATION_ACTION ->
         {
            ChestOrientationActionStateMessage chestOrientationActionStateMessage = treeStateMessage.getChestOrientationActions().get(indexInTypesList);
            subscriptionNode.setChestOrientationActionStateMessage(chestOrientationActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(chestOrientationActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(chestOrientationActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(chestOrientationActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(chestOrientationActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.FOOTSTEP_PLAN_ACTION ->
         {
            FootstepPlanActionStateMessage footstepPlanActionStateMessage = treeStateMessage.getFootstepPlanActions().get(indexInTypesList);
            subscriptionNode.setFootstepPlanActionStateMessage(footstepPlanActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(footstepPlanActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(footstepPlanActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(footstepPlanActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(footstepPlanActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.HAND_POSE_ACTION ->
         {
            HandPoseActionStateMessage handPoseActionStateMessage = treeStateMessage.getHandPoseActions().get(indexInTypesList);
            subscriptionNode.setHandPoseActionStateMessage(handPoseActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(handPoseActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(handPoseActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(handPoseActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(handPoseActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.HAND_WRENCH_ACTION ->
         {
            HandWrenchActionStateMessage handWrenchActionStateMessage = treeStateMessage.getHandWrenchActions().get(indexInTypesList);
            subscriptionNode.setHandWrenchActionStateMessage(handWrenchActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(handWrenchActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(handWrenchActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(handWrenchActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(handWrenchActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.SCREW_PRIMITIVE_ACTION ->
         {
            ScrewPrimitiveActionStateMessage screwPrimitiveActionStateMessage = treeStateMessage.getScrewPrimitiveActions().get(indexInTypesList);
            subscriptionNode.setScrewPrimitiveActionStateMessage(screwPrimitiveActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(screwPrimitiveActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(screwPrimitiveActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(screwPrimitiveActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(screwPrimitiveActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.PELVIS_HEIGHT_ORIENTATION_ACTION ->
         {
            PelvisHeightOrientationActionStateMessage pelvisHeightOrientationActionStateMessage = treeStateMessage.getPelvisHeightActions().get(indexInTypesList);
            subscriptionNode.setPelvisHeightOrientationActionStateMessage(pelvisHeightOrientationActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(pelvisHeightOrientationActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(pelvisHeightOrientationActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(pelvisHeightOrientationActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(pelvisHeightOrientationActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.ABILITY_HAND_ACTION ->
         {
            AbilityHandActionStateMessage abilityHandActionStateMessage = treeStateMessage.getAbilityHandActions().get(indexInTypesList);
            subscriptionNode.setAbilityHandActionStateMessage(abilityHandActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(abilityHandActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(abilityHandActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(abilityHandActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(abilityHandActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.SAKE_HAND_COMMAND_ACTION ->
         {
            SakeHandCommandActionStateMessage sakeHandCommandActionStateMessage = treeStateMessage.getSakeHandCommandActions().get(indexInTypesList);
            subscriptionNode.setSakeHandCommandActionStateMessage(sakeHandCommandActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(sakeHandCommandActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(sakeHandCommandActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(sakeHandCommandActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(sakeHandCommandActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.WAIT_DURATION_ACTION ->
         {
            WaitDurationActionStateMessage waitDurationActionStateMessage = treeStateMessage.getWaitDurationActions().get(indexInTypesList);
            subscriptionNode.setWaitDurationActionStateMessage(waitDurationActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(waitDurationActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(waitDurationActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(waitDurationActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(waitDurationActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.FOOT_POSE_ACTION ->
         {
            FootPoseActionStateMessage footPoseActionStateMessage = treeStateMessage.getFootPoseActions().get(indexInTypesList);
            subscriptionNode.setFootPoseActionStateMessage(footPoseActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(footPoseActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(footPoseActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(footPoseActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(footPoseActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
      }
   }

   public static void packYoData(BehaviorTreeExecutor tree, BehaviorTreeYoDataMessage yoDataMessage)
   {
      BehaviorTreeSceneExecutor scene = tree.getScene();

      yoDataMessage.setNumberOfPersistentDetections((byte) scene.getPersistentDetections().size());
      yoDataMessage.setNumberOfSceneObjects((byte) scene.getObjects().size());

      for (int i = 0; i < yoDataMessage.getSceneObjectPose().length; i++)
      {
         if (scene.getObjects().size() > i)
            yoDataMessage.getSceneObjectPose()[i].set(scene.getObjects().get(i).getTransformToWorld());
         else
            yoDataMessage.getSceneObjectPose()[i].setToNaN();
      }

      BehaviorTreeRootNodeExecutor rootNode = tree.getRootNode();
      if (rootNode != null)
      {
         yoDataMessage.setAutomaticExecution(rootNode.getState().getAutomaticExecution());
         yoDataMessage.setExecutionNextIndex((byte) rootNode.getState().getExecutionNextIndex());
         yoDataMessage.setConcurrencyEnabled(rootNode.getState().getConcurrencyEnabled());
         yoDataMessage.setNumberOfExecutingActions((byte) rootNode.getCurrentlyExecutingLeaves().size());
         yoDataMessage.setNumberOfFailedActions((byte) rootNode.getFailedLeaves().size());

         for (int i = 0; i < yoDataMessage.getExecutingActionType().length; i++)
         {
            if (rootNode.getCurrentlyExecutingLeaves().size() > i)
            {
               yoDataMessage.getExecutingActionType()[i]
                     = BehaviorTreeDefinitionRegistry.getMessageByte(rootNode.getCurrentlyExecutingLeaves().get(i).getDefinition().getClass());
               yoDataMessage.getExecutingActionId()[i] = (short) rootNode.getCurrentlyExecutingLeaves().get(i).getState().getID();
               if (rootNode.getCurrentlyExecutingLeaves().get(i).getState() instanceof ActionNodeState<?> actionState)
                  yoDataMessage.getElapsedExecutionTime()[i] = (float) actionState.getElapsedExecutionTime();
            }
            else
            {
               yoDataMessage.getExecutingActionType()[i] = -1;
               yoDataMessage.getExecutingActionId()[i] = -1;
               yoDataMessage.getElapsedExecutionTime()[i] = Float.NaN;
            }
         }

         SideDependentList<HandPoseActionExecutor> lastHandPoseActions = new SideDependentList<>();
         for (LeafNodeExecutor<?, ?> leaf : rootNode.getCurrentlyExecutingLeaves())
            if (leaf instanceof HandPoseActionExecutor handPoseAction)
               lastHandPoseActions.put(handPoseAction.getDefinition().getSide(), handPoseAction);
         for (RobotSide side : lastHandPoseActions.sides())
         {
            HandPoseActionExecutor action = lastHandPoseActions.get(side);
            yoDataMessage.getCurrentHandPose()[side.ordinal()].set(action.getState().getCurrentPose().getValueReadOnly());
            CRDTStatusSE3Trajectory commandedTrajectory = action.getState().getCommandedTrajectory();
            if (!commandedTrajectory.isEmpty())
               yoDataMessage.getGoalHandPose()[side.ordinal()].set(commandedTrajectory.getLastValueReadOnly().getOrientation(),
                                                                   commandedTrajectory.getLastValueReadOnly().getPosition());
         }
      }
   }
}
