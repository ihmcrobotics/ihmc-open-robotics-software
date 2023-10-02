package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.*;
import us.ihmc.behaviors.sequence.actions.*;

import java.util.List;

public class BehaviorActionSequenceTools
{
   public static <T extends BehaviorActionStateSupplier> void packActionSequenceUpdateMessage(List<T> actionSequence,
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

      for (int i = 0; i < actionSequence.size(); i++)
      {
         BehaviorActionDefinition action = actionSequence.get(i).getState().getDefinition();
         if (action instanceof ArmJointAnglesActionDefinition armJointAnglesActionDefinition)
         {
            ArmJointAnglesActionDefinitionMessage armJointAnglesActionDefinitionMessage = actionSequenceUpdateMessage.getArmJointAnglesActions().add();
            armJointAnglesActionDefinitionMessage.getActionInformation().setActionIndex(i);
            armJointAnglesActionDefinition.toMessage(armJointAnglesActionDefinitionMessage);
         }
         else if (action instanceof ChestOrientationActionDefinition chestOrientationActionDefinition)
         {
            BodyPartPoseActionDefinitionMessage chestOrientationActionMessage = actionSequenceUpdateMessage.getChestOrientationActions().add();
            chestOrientationActionMessage.getActionInformation().setActionIndex(i);
            chestOrientationActionDefinition.toMessage(chestOrientationActionMessage);
         }
         else if (action instanceof FootstepPlanActionDefinition footstepPlanActionDefinition)
         {
            FootstepPlanActionDefinitionMessage footstepPlanActionDefinitionMessage = actionSequenceUpdateMessage.getFootstepPlanActions().add();
            footstepPlanActionDefinitionMessage.getActionInformation().setActionIndex(i);
            footstepPlanActionDefinition.toMessage(footstepPlanActionDefinitionMessage);
         }
         else if (action instanceof SakeHandCommandActionDefinition sakeHandCommandActionDefinition)
         {
            SakeHandCommandActionDefinitionMessage sakehandCommandMessage = actionSequenceUpdateMessage.getSakeHandCommandActions().add();
            sakehandCommandMessage.getActionInformation().setActionIndex(i);
            sakeHandCommandActionDefinition.toMessage(sakehandCommandMessage);
         }
         else if (action instanceof HandPoseActionDefinition handPoseActionDefinition)
         {
            SidedBodyPartPoseActionDefinitionMessage handPoseActionMessage = actionSequenceUpdateMessage.getHandPoseActions().add();
            handPoseActionMessage.getActionInformation().setActionIndex(i);
            handPoseActionDefinition.toMessage(handPoseActionMessage);
         }
         else if (action instanceof HandWrenchActionDefinition handWrenchActionDefinition)
         {
            HandWrenchActionDefinitionMessage handWrenchActionDefinitionMessage = actionSequenceUpdateMessage.getHandWrenchActions().add();
            handWrenchActionDefinitionMessage.getActionInformation().setActionIndex(i);
            handWrenchActionDefinition.toMessage(handWrenchActionDefinitionMessage);
         }
         else if (action instanceof PelvisHeightPitchActionDefinition pelvisHeightActionDefinition)
         {
            BodyPartPoseActionDefinitionMessage pelvisHeightActionMessage = actionSequenceUpdateMessage.getPelvisHeightActions().add();
            pelvisHeightActionMessage.getActionInformation().setActionIndex(i);
            pelvisHeightActionDefinition.toMessage(pelvisHeightActionMessage);
         }
         else if (action instanceof WaitDurationActionDefinition waitDurationActionDefinition)
         {
            WaitDurationActionDefinitionMessage waitDurationActionDefinitionMessage = actionSequenceUpdateMessage.getWaitDurationActions().add();
            waitDurationActionDefinitionMessage.getActionInformation().setActionIndex(i);
            waitDurationActionDefinition.toMessage(waitDurationActionDefinitionMessage);
         }
         else if (action instanceof WalkActionDefinition walkActionDefinition)
         {
            WalkActionDefinitionMessage walkActionDefinitionMessage = actionSequenceUpdateMessage.getWalkActions().add();
            walkActionDefinitionMessage.getActionInformation().setActionIndex(i);
            walkActionDefinition.toMessage(walkActionDefinitionMessage);
         }
      }
   }
}
