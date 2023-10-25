package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionSequenceUpdateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.sequence.actions.*;

import java.util.List;

public class BehaviorActionSequenceTools
{
   public static <T extends ActionNodeExecutor> void packActionSequenceUpdateMessage(List<T> actionSequence,
                                                                                     ActionSequenceUpdateMessage actionSequenceUpdateMessage)
   {
      actionSequenceUpdateMessage.setSequenceSize(actionSequence.size());
      actionSequenceUpdateMessage.getArmJointAnglesActions().clear();
      actionSequenceUpdateMessage.getChestOrientationActions().clear();
      actionSequenceUpdateMessage.getFootstepPlanActions().clear();
      actionSequenceUpdateMessage.getSakeHandCommandActions().clear();
      actionSequenceUpdateMessage.getHandPoseActions().clear();
      actionSequenceUpdateMessage.getHandWrenchActions().clear();
      actionSequenceUpdateMessage.getPelvisHeightActions().clear();
      actionSequenceUpdateMessage.getWaitDurationActions().clear();
      actionSequenceUpdateMessage.getWalkActions().clear();

      for (T action : actionSequence)
      {
         BehaviorTreeNodeState state = action.getState(); // TODO Why doesn't this return BehaviorActionState?
         if (state instanceof ActionNodeState<?> actionState)
         {
            if (actionState instanceof ArmJointAnglesActionState armJointAnglesActionState)
            {
               armJointAnglesActionState.toMessage(actionSequenceUpdateMessage.getArmJointAnglesActions().add());
            }
            else if (actionState instanceof ChestOrientationActionState chestOrientationActionState)
            {
               chestOrientationActionState.toMessage(actionSequenceUpdateMessage.getChestOrientationActions().add());
            }
            else if (actionState instanceof FootstepPlanActionState footstepPlanActionState)
            {
               footstepPlanActionState.toMessage(actionSequenceUpdateMessage.getFootstepPlanActions().add());
            }
            else if (actionState instanceof SakeHandCommandActionState sakeHandCommandActionState)
            {
               sakeHandCommandActionState.toMessage(actionSequenceUpdateMessage.getSakeHandCommandActions().add());
            }
            else if (actionState instanceof HandPoseActionState handPoseActionState)
            {
               handPoseActionState.toMessage(actionSequenceUpdateMessage.getHandPoseActions().add());
            }
            else if (actionState instanceof HandWrenchActionState handWrenchActionState)
            {
               handWrenchActionState.toMessage(actionSequenceUpdateMessage.getHandWrenchActions().add());
            }
            else if (actionState instanceof PelvisHeightPitchActionState pelvisHeightActionState)
            {
               pelvisHeightActionState.toMessage(actionSequenceUpdateMessage.getPelvisHeightActions().add());
            }
            else if (actionState instanceof WaitDurationActionState waitDurationActionState)
            {
               waitDurationActionState.toMessage(actionSequenceUpdateMessage.getWaitDurationActions().add());
            }
            else if (actionState instanceof WalkActionState walkActionState)
            {
               walkActionState.toMessage(actionSequenceUpdateMessage.getWalkActions().add());
            }
         }
      }
   }
}
