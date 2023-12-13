package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
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
      treeStateMessage.getArmJointAnglesActions().clear();
      treeStateMessage.getChestOrientationActions().clear();
      treeStateMessage.getFootstepPlanActions().clear();
      treeStateMessage.getHandPoseActions().clear();
      treeStateMessage.getHandWrenchActions().clear();
      treeStateMessage.getPelvisHeightActions().clear();
      treeStateMessage.getSakeHandCommandActions().clear();
      treeStateMessage.getWaitDurationActions().clear();
      treeStateMessage.getWalkActions().clear();
   }

   public static void packMessage(BehaviorTreeNodeState nodeState, BehaviorTreeStateMessage treeStateMessage)
   {
      if (nodeState instanceof ActionSequenceState actionSequenceState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.ACTION_SEQUENCE);
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getActionSequences().size());
         actionSequenceState.toMessage(treeStateMessage.getActionSequences().add());
      }
      else if (nodeState instanceof ArmJointAnglesActionState armJointAnglesActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.ARM_JOINT_ANGLES_ACTION);
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getArmJointAnglesActions().size());
         armJointAnglesActionState.toMessage(treeStateMessage.getArmJointAnglesActions().add());
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
      else if (nodeState instanceof PelvisHeightPitchActionState pelvisHeightActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.PELVIS_HEIGHT_PITCH_ACTION);
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getPelvisHeightActions().size());
         pelvisHeightActionState.toMessage(treeStateMessage.getPelvisHeightActions().add());
      }
      else if (nodeState instanceof WaitDurationActionState waitDurationActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.WAIT_DURATION_ACTION);
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getWaitDurationActions().size());
         waitDurationActionState.toMessage(treeStateMessage.getWaitDurationActions().add());
      }
      else if (nodeState instanceof WalkActionState walkActionState)
      {
         treeStateMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.WALK_ACTION);
         treeStateMessage.getBehaviorTreeIndices().add(treeStateMessage.getWalkActions().size());
         walkActionState.toMessage(treeStateMessage.getWalkActions().add());
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

   public static void fromMessage(ROS2BehaviorTreeSubscriptionNode subscriptionNode, BehaviorTreeNodeState<?> nodeState)
   {
      if (nodeState instanceof ActionSequenceState actionSequenceState)
      {
         actionSequenceState.fromMessage(subscriptionNode.getActionSequenceStateMessage());
      }
      else if (nodeState instanceof ArmJointAnglesActionState armJointAnglesActionState)
      {
         armJointAnglesActionState.fromMessage(subscriptionNode.getArmJointAnglesActionStateMessage());
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
      else if (nodeState instanceof PelvisHeightPitchActionState pelvisHeightActionState)
      {
         pelvisHeightActionState.fromMessage(subscriptionNode.getPelvisHeightPitchActionStateMessage());
      }
      else if (nodeState instanceof WaitDurationActionState waitDurationActionState)
      {
         waitDurationActionState.fromMessage(subscriptionNode.getWaitDurationActionStateMessage());
      }
      else if (nodeState instanceof WalkActionState walkActionState)
      {
         walkActionState.fromMessage(subscriptionNode.getWalkActionStateMessage());
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
      switch (nodeType)
      {
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
         case BehaviorTreeStateMessage.ARM_JOINT_ANGLES_ACTION ->
         {
            ArmJointAnglesActionStateMessage armJointAnglesActionStateMessage = treeStateMessage.getArmJointAnglesActions().get(indexInTypesList);
            subscriptionNode.setArmJointAnglesActionStateMessage(armJointAnglesActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(armJointAnglesActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(armJointAnglesActionStateMessage.getDefinition().getDefinition().getDefinition());
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
         case BehaviorTreeStateMessage.PELVIS_HEIGHT_PITCH_ACTION ->
         {
            PelvisHeightPitchActionStateMessage pelvisHeightPitchActionStateMessage = treeStateMessage.getPelvisHeightActions().get(indexInTypesList);
            subscriptionNode.setPelvisHeightPitchActionStateMessage(pelvisHeightPitchActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(pelvisHeightPitchActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(pelvisHeightPitchActionStateMessage.getDefinition().getDefinition().getDefinition());
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
         case BehaviorTreeStateMessage.WALK_ACTION ->
         {
            WalkActionStateMessage walkActionStateMessage = treeStateMessage.getWalkActions().get(indexInTypesList);
            subscriptionNode.setWalkActionStateMessage(walkActionStateMessage);
            subscriptionNode.setBehaviorTreeNodeStateMessage(walkActionStateMessage.getState().getState());
            subscriptionNode.setBehaviorTreeNodeDefinitionMessage(walkActionStateMessage.getDefinition().getDefinition().getDefinition());
         }
      }
   }
}
