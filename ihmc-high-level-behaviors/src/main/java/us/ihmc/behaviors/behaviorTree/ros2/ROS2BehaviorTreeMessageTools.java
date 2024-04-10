package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.door.DoorTraversalState;
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
      treeStateMessage.getBasicNodes().clear();
      treeStateMessage.getActionSequences().clear();
      treeStateMessage.getDoorTraversals().clear();
      treeStateMessage.getChestOrientationActions().clear();
      treeStateMessage.getFootstepPlanActions().clear();
      treeStateMessage.getHandPoseActions().clear();
      treeStateMessage.getHandWrenchActions().clear();
      treeStateMessage.getScrewPrimitiveActions().clear();
      treeStateMessage.getPelvisHeightActions().clear();
      treeStateMessage.getSakeHandCommandActions().clear();
      treeStateMessage.getWaitDurationActions().clear();
      treeStateMessage.getKickDoorActions().clear();
      treeStateMessage.getKickDoorApproachActions().clear();
   }

   public static void packMessage(BehaviorTreeNodeState nodeState, BehaviorTreeStateMessage treeStateMessage)
   {
      if (nodeState instanceof ActionSequenceState actionSequenceState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.ACTION_SEQUENCE.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getActionSequences().size());
         actionSequenceState.toMessage(treeStateMessage.getActionSequences().add());
      }
      else if (nodeState instanceof DoorTraversalState doorTraversalState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.DOOR_TRAVERSAL.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getDoorTraversals().size());
         doorTraversalState.toMessage(treeStateMessage.getDoorTraversals().add());
      }
      else if (nodeState instanceof ChestOrientationActionState chestOrientationActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.CHEST_ORIENTATION_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getChestOrientationActions().size());
         chestOrientationActionState.toMessage(treeStateMessage.getChestOrientationActions().add());
      }
      else if (nodeState instanceof FootstepPlanActionState footstepPlanActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.FOOTSTEP_PLAN_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getFootstepPlanActions().size());
         footstepPlanActionState.toMessage(treeStateMessage.getFootstepPlanActions().add());
      }
      else if (nodeState instanceof SakeHandCommandActionState sakeHandCommandActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.SAKE_HAND_COMMAND_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getSakeHandCommandActions().size());
         sakeHandCommandActionState.toMessage(treeStateMessage.getSakeHandCommandActions().add());
      }
      else if (nodeState instanceof HandPoseActionState handPoseActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.HAND_POSE_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getHandPoseActions().size());
         handPoseActionState.toMessage(treeStateMessage.getHandPoseActions().add());
      }
      else if (nodeState instanceof HandWrenchActionState handWrenchActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.HAND_WRENCH_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getHandWrenchActions().size());
         handWrenchActionState.toMessage(treeStateMessage.getHandWrenchActions().add());
      }
      else if (nodeState instanceof ScrewPrimitiveActionState screwPrimitiveActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.SCREW_PRIMITIVE_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getScrewPrimitiveActions().size());
         screwPrimitiveActionState.toMessage(treeStateMessage.getScrewPrimitiveActions().add());
      }
      else if (nodeState instanceof PelvisHeightPitchActionState pelvisHeightActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.PELVIS_HEIGHT_PITCH_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getPelvisHeightActions().size());
         pelvisHeightActionState.toMessage(treeStateMessage.getPelvisHeightActions().add());
      }
      else if (nodeState instanceof WaitDurationActionState waitDurationActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.WAIT_DURATION_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getWaitDurationActions().size());
         waitDurationActionState.toMessage(treeStateMessage.getWaitDurationActions().add());
      }
      else if (nodeState instanceof KickDoorActionState kickDoorActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.KICK_DOOR_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getKickDoorActions().size());
         kickDoorActionState.toMessage(treeStateMessage.getKickDoorActions().add());
      }
      else if (nodeState instanceof KickDoorApproachPlanActionState kickDoorApproachPlanActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.KICK_DOOR_APPROACH_ACTION.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getKickDoorActions().size());
         kickDoorApproachPlanActionState.toMessage(treeStateMessage.getKickDoorApproachActions().add());
      }
      else
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeDefinitionRegistry.BASIC_NODE.toByte());
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getBasicNodes().size());
         BasicNodeStateMessage basicNodeMessage = treeStateMessage.getBasicNodes().add();
         nodeState.toMessage(basicNodeMessage.getState());
         nodeState.getDefinition().toMessage(basicNodeMessage.getDefinition());
      }
   }

   public static void fromMessage(ROS2BehaviorTreeSubscriptionNode subscriptionNode, BehaviorTreeNodeState<?> nodeState)
   {
      if (nodeState instanceof ActionSequenceState actionSequenceState)
      {
         actionSequenceState.fromMessage(subscriptionNode.getActionSequenceStateMessage());
      }
      else if (nodeState instanceof DoorTraversalState doorTraversalState)
      {
         doorTraversalState.fromMessage(subscriptionNode.getDoorTraversalStateMessage());
      }
      else if (nodeState instanceof ChestOrientationActionState chestOrientationActionState)
      {
         chestOrientationActionState.fromMessage(subscriptionNode.getChestOrientationActionStateMessage());
      }
      else if (nodeState instanceof FootstepPlanActionState footstepPlanActionState)
      {
         footstepPlanActionState.fromMessage(subscriptionNode.getFootstepPlanActionStateMessage());
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
      else if (nodeState instanceof PelvisHeightPitchActionState pelvisHeightActionState)
      {
         pelvisHeightActionState.fromMessage(subscriptionNode.getPelvisHeightPitchActionStateMessage());
      }
      else if (nodeState instanceof WaitDurationActionState waitDurationActionState)
      {
         waitDurationActionState.fromMessage(subscriptionNode.getWaitDurationActionStateMessage());
      }
      else if (nodeState instanceof KickDoorApproachPlanActionState kickDoorApproachPlanActionState)
      {
         kickDoorApproachPlanActionState.fromMessage(subscriptionNode.getKickDoorApproachPlanActionStateMessage());
      }
      else if (nodeState instanceof KickDoorActionState kickDoorActionState)
      {
         kickDoorActionState.fromMessage(subscriptionNode.getKickDoorActionStateMessage());
      }
      else
      {
         nodeState.fromMessage(subscriptionNode.getBehaviorTreeNodeStateMessage());
      }
   }

   public static void packSubscriptionNode(byte nodeType,
                                           int indexInTypesList,
                                           BehaviorTreeStateMessage treeStateMessage,
                                           ROS2BehaviorTreeSubscriptionNode subscriptionNode)
   {
      packSubscriptionNode(BehaviorTreeDefinitionRegistry.fromByte(nodeType), indexInTypesList, treeStateMessage, subscriptionNode);
   }

   public static void packSubscriptionNode(BehaviorTreeDefinitionRegistry nodeEnum,
                                           int indexInTypesList,
                                           BehaviorTreeStateMessage treeStateMessage,
                                           ROS2BehaviorTreeSubscriptionNode subscriptionNode)
   {
      switch (nodeEnum)
      {
         case BASIC_NODE ->
         {
            BasicNodeStateMessage basicNodeStateMessage = treeStateMessage.getBasicNodes().get(indexInTypesList);
            subscriptionNode.setBehaviorTreeNodeStateMessage(basicNodeStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(basicNodeStateMessage.getDefinition());
         }
         case ACTION_SEQUENCE ->
         {
            ActionSequenceStateMessage actionSequenceStateMessage = treeStateMessage.getActionSequences().get(indexInTypesList);
            subscriptionNode.setActionSequenceStateMessage(actionSequenceStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(actionSequenceStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(actionSequenceStateMessage.getDefinition().getDefinition());
         }
         case DOOR_TRAVERSAL ->
         {
            DoorTraversalStateMessage doorTraversalStateMessage = treeStateMessage.getDoorTraversals().get(indexInTypesList);
            subscriptionNode.setDoorTraversalStateMessage(doorTraversalStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(doorTraversalStateMessage.getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(doorTraversalStateMessage.getDefinition().getDefinition());
         }
         case CHEST_ORIENTATION_ACTION ->
         {
            ChestOrientationActionStateMessage chestOrientationActionStateMessage = treeStateMessage.getChestOrientationActions().get(indexInTypesList);
            subscriptionNode.setChestOrientationActionStateMessage(chestOrientationActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(chestOrientationActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(chestOrientationActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case FOOTSTEP_PLAN_ACTION ->
         {
            FootstepPlanActionStateMessage footstepPlanActionStateMessage = treeStateMessage.getFootstepPlanActions().get(indexInTypesList);
            subscriptionNode.setFootstepPlanActionStateMessage(footstepPlanActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(footstepPlanActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(footstepPlanActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case HAND_POSE_ACTION ->
         {
            HandPoseActionStateMessage handPoseActionStateMessage = treeStateMessage.getHandPoseActions().get(indexInTypesList);
            subscriptionNode.setHandPoseActionStateMessage(handPoseActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(handPoseActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(handPoseActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case HAND_WRENCH_ACTION ->
         {
            HandWrenchActionStateMessage handWrenchActionStateMessage = treeStateMessage.getHandWrenchActions().get(indexInTypesList);
            subscriptionNode.setHandWrenchActionStateMessage(handWrenchActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(handWrenchActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(handWrenchActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case SCREW_PRIMITIVE_ACTION ->
         {
            ScrewPrimitiveActionStateMessage screwPrimitiveActionStateMessage = treeStateMessage.getScrewPrimitiveActions().get(indexInTypesList);
            subscriptionNode.setScrewPrimitiveActionStateMessage(screwPrimitiveActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(screwPrimitiveActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(screwPrimitiveActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case PELVIS_HEIGHT_PITCH_ACTION ->
         {
            PelvisHeightPitchActionStateMessage pelvisHeightPitchActionStateMessage = treeStateMessage.getPelvisHeightActions().get(indexInTypesList);
            subscriptionNode.setPelvisHeightPitchActionStateMessage(pelvisHeightPitchActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(pelvisHeightPitchActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(pelvisHeightPitchActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case SAKE_HAND_COMMAND_ACTION ->
         {
            SakeHandCommandActionStateMessage sakeHandCommandActionStateMessage = treeStateMessage.getSakeHandCommandActions().get(indexInTypesList);
            subscriptionNode.setSakeHandCommandActionStateMessage(sakeHandCommandActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(sakeHandCommandActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(sakeHandCommandActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case WAIT_DURATION_ACTION ->
         {
            WaitDurationActionStateMessage waitDurationActionStateMessage = treeStateMessage.getWaitDurationActions().get(indexInTypesList);
            subscriptionNode.setWaitDurationActionStateMessage(waitDurationActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(waitDurationActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(waitDurationActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case KICK_DOOR_ACTION ->
         {
            KickDoorActionStateMessage kickDoorActionStateMessage = treeStateMessage.getKickDoorActions().get(indexInTypesList);
            subscriptionNode.setKickDoorActionStateMessage(kickDoorActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(kickDoorActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(kickDoorActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         case KICK_DOOR_APPROACH_ACTION ->
         {
            KickDoorApproachPlanStateMessage kickDoorApproachPlanActionStateMessage = treeStateMessage.getKickDoorApproachActions().get(indexInTypesList);
            subscriptionNode.setKickDoorApproachPlanActionStateMessage(kickDoorApproachPlanActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(kickDoorApproachPlanActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(kickDoorApproachPlanActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
         default ->
         {
            Class<?> behaviorClass = nodeEnum.getTypeClass();
            String name = behaviorClass != null ? behaviorClass.getName() : "Undefined";
            throw new RuntimeException("Undefined behavior tree state message type " + name);
         }
      }
   }
}
