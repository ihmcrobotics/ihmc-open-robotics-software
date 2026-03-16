package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeState;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationState;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalState;
import us.ihmc.behaviors.behaviorTree.action.actions.CheckpointNodeState;
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
      treeStateMessage.getSpineActions().clear();
      treeStateMessage.getWalkActions().clear();
      treeStateMessage.getArmActions().clear();
      treeStateMessage.getHandWrenchActions().clear();
      treeStateMessage.getScrewPrimitiveActions().clear();
      treeStateMessage.getPelvisActions().clear();
      treeStateMessage.getAbilityHandActions().clear();
      treeStateMessage.getEzGripperActions().clear();
      treeStateMessage.getWaitActions().clear();
      treeStateMessage.getLegActions().clear();
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
         else if (nodeState instanceof CheckpointNodeState checkpointNodeState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.CHECKPOINT_NODE);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getCheckpointNodes().size());
            checkpointNodeState.toMessage(treeStateMessage.getCheckpointNodes().add());
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
         else if (nodeState instanceof SpineActionState spineActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.SPINE_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getSpineActions().size());
            spineActionState.toMessage(treeStateMessage.getSpineActions().add());
         }
         else if (nodeState instanceof WalkActionState walkActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.WALK_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getWalkActions().size());
            walkActionState.toMessage(treeStateMessage.getWalkActions().add());
         }
         else if (nodeState instanceof AbilityHandActionState abilityHandActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.ABILITY_HAND_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getAbilityHandActions().size());
            abilityHandActionState.toMessage(treeStateMessage.getAbilityHandActions().add());
         }
         else if (nodeState instanceof EZGripperActionState ezGripperActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.EZGRIPPER_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getEzGripperActions().size());
            ezGripperActionState.toMessage(treeStateMessage.getEzGripperActions().add());
         }
         else if (nodeState instanceof ArmActionState armActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.ARM_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getArmActions().size());
            armActionState.toMessage(treeStateMessage.getArmActions().add());
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
         else if (nodeState instanceof PelvisActionState pelvisActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.PELVIS_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getPelvisActions().size());
            pelvisActionState.toMessage(treeStateMessage.getPelvisActions().add());
         }
         else if (nodeState instanceof WaitActionState waitActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.WAIT_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getWaitActions().size());
            waitActionState.toMessage(treeStateMessage.getWaitActions().add());
         }
         else if (nodeState instanceof LegActionState legActionState)
         {
            treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.LEG_ACTION);
            treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getLegActions().size());
            legActionState.toMessage(treeStateMessage.getLegActions().add());
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
      else if (nodeState instanceof CheckpointNodeState checkpointNodeState)
      {
         checkpointNodeState.fromMessage(subscriptionNode.getCheckpointNodeStateMessage());
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
      else if (nodeState instanceof SpineActionState spineActionState)
      {
         spineActionState.fromMessage(subscriptionNode.getSpineActionStateMessage());
      }
      else if (nodeState instanceof WalkActionState walkActionState)
      {
         walkActionState.fromMessage(subscriptionNode.getWalkActionStateMessage());
      }
      else if (nodeState instanceof AbilityHandActionState abilityHandActionState)
      {
         abilityHandActionState.fromMessage(subscriptionNode.getAbilityHandActionStateMessage());
      }
      else if (nodeState instanceof EZGripperActionState ezGripperActionState)
      {
         ezGripperActionState.fromMessage(subscriptionNode.getEZGripperActionStateMessage());
      }
      else if (nodeState instanceof ArmActionState armActionState)
      {
         armActionState.fromMessage(subscriptionNode.getArmActionStateMessage());
      }
      else if (nodeState instanceof HandWrenchActionState handWrenchActionState)
      {
         handWrenchActionState.fromMessage(subscriptionNode.getHandWrenchActionStateMessage());
      }
      else if (nodeState instanceof ScrewPrimitiveActionState screwPrimitiveActionState)
      {
         screwPrimitiveActionState.fromMessage(subscriptionNode.getScrewPrimitiveActionStateMessage());
      }
      else if (nodeState instanceof PelvisActionState pelvisActionState)
      {
         pelvisActionState.fromMessage(subscriptionNode.getPelvisActionStateMessage());
      }
      else if (nodeState instanceof WaitActionState waitActionState)
      {
         waitActionState.fromMessage(subscriptionNode.getWaitActionStateMessage());
      }
      else if (nodeState instanceof LegActionState legActionState)
      {
         legActionState.fromMessage(subscriptionNode.getLegActionStateMessage());
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
            CheckpointNodeStateMessage checkpointNodeStateMessage = treeStateMessage.getCheckpointNodes().get(indexInTypesList);
            subscriptionNode.setCheckpointNodeStateMessage(checkpointNodeStateMessage);
            subscriptionNode.setLeafNodeStateMessage(checkpointNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(checkpointNodeStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(checkpointNodeStateMessage.getDefinition().getDefinition().getDefinition());
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
         case BehaviorTreeStateMessage.SPINE_ACTION ->
         {
            SpineActionStateMessage spineActionStateMessage = treeStateMessage.getSpineActions().get(indexInTypesList);
            subscriptionNode.setSpineActionStateMessage(spineActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(spineActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(spineActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(spineActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(spineActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.WALK_ACTION ->
         {
            WalkActionStateMessage walkActionStateMessage = treeStateMessage.getWalkActions().get(indexInTypesList);
            subscriptionNode.setWalkActionStateMessage(walkActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(walkActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(walkActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(walkActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(walkActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.ARM_ACTION ->
         {
            ArmActionStateMessage armActionStateMessage = treeStateMessage.getArmActions().get(indexInTypesList);
            subscriptionNode.setArmActionStateMessage(armActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(armActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(armActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(armActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(armActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
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
         case BehaviorTreeStateMessage.PELVIS_ACTION ->
         {
            PelvisActionStateMessage pelvisActionStateMessage = treeStateMessage.getPelvisActions().get(indexInTypesList);
            subscriptionNode.setPelvisActionStateMessage(pelvisActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(pelvisActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(pelvisActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(pelvisActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(pelvisActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
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
         case BehaviorTreeStateMessage.EZGRIPPER_ACTION ->
         {
            EZGripperActionStateMessage ezGripperActionStateMessage = treeStateMessage.getEzGripperActions().get(indexInTypesList);
            subscriptionNode.setEZGripperActionStateMessage(ezGripperActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(ezGripperActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(ezGripperActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(ezGripperActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(ezGripperActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.WAIT_ACTION ->
         {
            WaitActionStateMessage waitActionStateMessage = treeStateMessage.getWaitActions().get(indexInTypesList);
            subscriptionNode.setWaitActionStateMessage(waitActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(waitActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(waitActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(waitActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(waitActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
         case BehaviorTreeStateMessage.LEG_ACTION ->
         {
            LegActionStateMessage legActionStateMessage = treeStateMessage.getLegActions().get(indexInTypesList);
            subscriptionNode.setLegActionStateMessage(legActionStateMessage);
            subscriptionNode.setActionNodeStateMessage(legActionStateMessage.getState());
            subscriptionNode.setLeafNodeStateMessage(legActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeStateMessage(legActionStateMessage.getState().getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(legActionStateMessage.getDefinition().getDefinition().getDefinition().getDefinition());
         }
      }
   }

   public static void packYoData(BehaviorTreeExecutor tree, BehaviorTreeYoDataMessage yoDataMessage)
   {
      BehaviorTreeRootNodeExecutor rootNode = tree.getRootNode();
      if (rootNode != null)
      {
         BehaviorTreeSceneExecutor scene = rootNode.getScene();
         yoDataMessage.setNumberOfPersistentDetections((byte) scene.getPersistentDetections().size());
         yoDataMessage.setNumberOfSceneObjects((byte) scene.getObjects().size());

         for (int i = 0; i < yoDataMessage.getSceneObjectPose().length; i++)
         {
            if (scene.getObjects().size() > i)
               yoDataMessage.getSceneObjectPose()[i].set(scene.getObjects().get(i).getTransformToWorld());
            else
               yoDataMessage.getSceneObjectPose()[i].setToNaN();
         }

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

         SideDependentList<ArmActionExecutor> lastArmActions = new SideDependentList<>();
         for (LeafNodeExecutor<?, ?> leaf : rootNode.getCurrentlyExecutingLeaves())
            if (leaf instanceof ArmActionExecutor armAction)
               lastArmActions.put(armAction.getDefinition().getSide(), armAction);
         for (RobotSide side : lastArmActions.sides())
         {
            ArmActionExecutor action = lastArmActions.get(side);
            yoDataMessage.getCurrentHandPose()[side.ordinal()].set(action.getState().getCurrentPose().getValueReadOnly());
            CRDTStatusSE3Trajectory commandedTrajectory = action.getState().getCommandedTrajectory();
            if (!commandedTrajectory.isEmpty())
               yoDataMessage.getGoalHandPose()[side.ordinal()].set(commandedTrajectory.getLastValueReadOnly().getOrientation(),
                                                                   commandedTrajectory.getLastValueReadOnly().getPosition());
         }
      }
   }
}
