package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.*;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.List;

public class BehaviorActionSequenceTools
{
   public static <T extends BehaviorActionData> void packActionSequenceUpdateMessage(List<T> actionSequence,
                                                                                     ActionSequenceUpdateMessage actionSequenceUpdateMessage)
   {
      actionSequenceUpdateMessage.setSequenceSize(actionSequence.size());
      actionSequenceUpdateMessage.getArmJointAnglesActions().clear();
      actionSequenceUpdateMessage.getChestOrientationActions().clear();
      actionSequenceUpdateMessage.getFootstepPlanActions().clear();
      actionSequenceUpdateMessage.getHandConfigurationActions().clear();
      actionSequenceUpdateMessage.getHandPoseActions().clear();
      actionSequenceUpdateMessage.getHandWrenchActions().clear();
      actionSequenceUpdateMessage.getPelvisHeightActions().clear();
      actionSequenceUpdateMessage.getWaitDurationActions().clear();
      actionSequenceUpdateMessage.getWalkActions().clear();

      for (int i = 0; i < actionSequence.size(); i++)
      {
         BehaviorActionData action = actionSequence.get(i);
         if (action instanceof ArmJointAnglesActionData armJointAnglesActionData)
         {
            ArmJointAnglesActionDescriptionMessage armJointAnglesActionDescriptionMessage = actionSequenceUpdateMessage.getArmJointAnglesActions().add();
            armJointAnglesActionDescriptionMessage.getActionInformation().setActionIndex(i);
            armJointAnglesActionData.toMessage(armJointAnglesActionDescriptionMessage);
         }
         else if (action instanceof ChestOrientationActionData chestOrientationActionData)
         {
            BodyPartPoseActionDescriptionMessage chestOrientationActionMessage = actionSequenceUpdateMessage.getChestOrientationActions().add();
            chestOrientationActionMessage.getActionInformation().setActionIndex(i);
            chestOrientationActionData.toMessage(chestOrientationActionMessage);
         }
         else if (action instanceof FootstepPlanActionData footstepPlanActionData)
         {
            FootstepPlanActionDescriptionMessage footstepPlanActionDescriptionMessage = actionSequenceUpdateMessage.getFootstepPlanActions().add();
            footstepPlanActionDescriptionMessage.getActionInformation().setActionIndex(i);
            footstepPlanActionData.toMessage(footstepPlanActionDescriptionMessage);
         }
         else if (action instanceof HandConfigurationActionData handConfigurationActionData)
         {
            HandConfigurationActionDescriptionMessage handConfigurationActionDescriptionMessage = actionSequenceUpdateMessage.getHandConfigurationActions().add();
            handConfigurationActionDescriptionMessage.getActionInformation().setActionIndex(i);
            handConfigurationActionData.toMessage(handConfigurationActionDescriptionMessage);
         }
         else if (action instanceof HandPoseActionData handPoseActionData)
         {
            SidedBodyPartPoseActionDescriptionMessage handPoseActionMessage = actionSequenceUpdateMessage.getHandPoseActions().add();
            handPoseActionMessage.getActionInformation().setActionIndex(i);
            handPoseActionData.toMessage(handPoseActionMessage);
         }
         else if (action instanceof HandWrenchActionData handWrenchActionData)
         {
            HandWrenchActionDescriptionMessage handWrenchActionDescriptionMessage = actionSequenceUpdateMessage.getHandWrenchActions().add();
            handWrenchActionDescriptionMessage.getActionInformation().setActionIndex(i);
            handWrenchActionData.toMessage(handWrenchActionDescriptionMessage);
         }
         else if (action instanceof PelvisHeightPitchActionData pelvisHeightActionData)
         {
            BodyPartPoseActionDescriptionMessage pelvisHeightActionMessage = actionSequenceUpdateMessage.getPelvisHeightActions().add();
            pelvisHeightActionMessage.getActionInformation().setActionIndex(i);
            pelvisHeightActionData.toMessage(pelvisHeightActionMessage);
         }
         else if (action instanceof WaitDurationActionData waitDurationActionData)
         {
            WaitDurationActionDescriptionMessage waitDurationActionDescriptionMessage = actionSequenceUpdateMessage.getWaitDurationActions().add();
            waitDurationActionDescriptionMessage.getActionInformation().setActionIndex(i);
            waitDurationActionData.toMessage(waitDurationActionDescriptionMessage);
         }
         else if (action instanceof WalkActionData walkActionData)
         {
            WalkActionDescriptionMessage walkActionDescriptionMessage = actionSequenceUpdateMessage.getWalkActions().add();
            walkActionDescriptionMessage.getActionInformation().setActionIndex(i);
            walkActionData.toMessage(walkActionDescriptionMessage);
         }
      }
   }

   /**
    * ReferenceFrames don't have mutable parents, so they get recreated. This accomodates for that.
    */
   public static void accomodateFrameReplacement(ModifiableReferenceFrame frameToUpdate, ReferenceFrameLibrary referenceFrameLibrary)
   {
      ReferenceFrame previousParentFrame = frameToUpdate.getReferenceFrame().getParent();
      ReferenceFrame nextParentFrame = referenceFrameLibrary.findFrameByNameOrWorld(previousParentFrame.getName());
      if (previousParentFrame != nextParentFrame)
      {
         frameToUpdate.changeParentFrame(nextParentFrame);
      }
   }
}
