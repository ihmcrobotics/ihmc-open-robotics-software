package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.*;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.List;

public class BehaviorActionSequenceTools
{
   public static <T extends BehaviorActionDescription> void packActionSequenceUpdateMessage(List<T> actionSequence,
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
         BehaviorActionDescription action = actionSequence.get(i);
         if (action instanceof ArmJointAnglesActionDescription armJointAnglesActionDescription)
         {
            ArmJointAnglesActionDescriptionMessage armJointAnglesActionDescriptionMessage = actionSequenceUpdateMessage.getArmJointAnglesActions().add();
            armJointAnglesActionDescriptionMessage.getActionInformation().setActionIndex(i);
            armJointAnglesActionDescription.toMessage(armJointAnglesActionDescriptionMessage);
         }
         else if (action instanceof ChestOrientationActionDescription chestOrientationActionDescription)
         {
            BodyPartPoseActionDescriptionMessage chestOrientationActionMessage = actionSequenceUpdateMessage.getChestOrientationActions().add();
            chestOrientationActionMessage.getActionInformation().setActionIndex(i);
            chestOrientationActionDescription.toMessage(chestOrientationActionMessage);
         }
         else if (action instanceof FootstepPlanActionDescription footstepPlanActionDescription)
         {
            FootstepPlanActionDescriptionMessage footstepPlanActionDescriptionMessage = actionSequenceUpdateMessage.getFootstepPlanActions().add();
            footstepPlanActionDescriptionMessage.getActionInformation().setActionIndex(i);
            footstepPlanActionDescription.toMessage(footstepPlanActionDescriptionMessage);
         }
         else if (action instanceof HandConfigurationActionDescription handConfigurationActionDescription)
         {
            HandConfigurationActionDescriptionMessage handConfigurationActionDescriptionMessage = actionSequenceUpdateMessage.getHandConfigurationActions().add();
            handConfigurationActionDescriptionMessage.getActionInformation().setActionIndex(i);
            handConfigurationActionDescription.toMessage(handConfigurationActionDescriptionMessage);
         }
         else if (action instanceof HandPoseActionDescription handPoseActionDescription)
         {
            SidedBodyPartPoseActionDescriptionMessage handPoseActionMessage = actionSequenceUpdateMessage.getHandPoseActions().add();
            handPoseActionMessage.getActionInformation().setActionIndex(i);
            handPoseActionDescription.toMessage(handPoseActionMessage);
         }
         else if (action instanceof HandWrenchActionDescription handWrenchActionDescription)
         {
            HandWrenchActionDescriptionMessage handWrenchActionDescriptionMessage = actionSequenceUpdateMessage.getHandWrenchActions().add();
            handWrenchActionDescriptionMessage.getActionInformation().setActionIndex(i);
            handWrenchActionDescription.toMessage(handWrenchActionDescriptionMessage);
         }
         else if (action instanceof PelvisHeightPitchActionDescription pelvisHeightActionDescription)
         {
            BodyPartPoseActionDescriptionMessage pelvisHeightActionMessage = actionSequenceUpdateMessage.getPelvisHeightActions().add();
            pelvisHeightActionMessage.getActionInformation().setActionIndex(i);
            pelvisHeightActionDescription.toMessage(pelvisHeightActionMessage);
         }
         else if (action instanceof WaitDurationActionDescription waitDurationActionDescription)
         {
            WaitDurationActionDescriptionMessage waitDurationActionDescriptionMessage = actionSequenceUpdateMessage.getWaitDurationActions().add();
            waitDurationActionDescriptionMessage.getActionInformation().setActionIndex(i);
            waitDurationActionDescription.toMessage(waitDurationActionDescriptionMessage);
         }
         else if (action instanceof WalkActionDescription walkActionDescription)
         {
            WalkActionDescriptionMessage walkActionDescriptionMessage = actionSequenceUpdateMessage.getWalkActions().add();
            walkActionDescriptionMessage.getActionInformation().setActionIndex(i);
            walkActionDescription.toMessage(walkActionDescriptionMessage);
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
