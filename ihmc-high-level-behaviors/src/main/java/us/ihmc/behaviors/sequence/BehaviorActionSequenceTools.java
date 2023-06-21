package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.*;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.Throttler;

import java.util.List;

public class BehaviorActionSequenceTools
{
   public static <T extends BehaviorActionData> void packActionSequenceUpdateMessage(List<T> actionSequence,
                                                                                     ActionSequenceUpdateMessage actionSequenceUpdateMessage)
   {
      actionSequenceUpdateMessage.setSequenceSize(actionSequence.size());
      actionSequenceUpdateMessage.getArmJointAnglesActions().clear();
      actionSequenceUpdateMessage.getChestOrientationActions().clear();
      actionSequenceUpdateMessage.getFootstepActions().clear();
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
            ArmJointAnglesActionMessage armJointAnglesActionMessage = actionSequenceUpdateMessage.getArmJointAnglesActions().add();
            armJointAnglesActionMessage.getActionInformation().setActionIndex(i);
            armJointAnglesActionData.toMessage(armJointAnglesActionMessage);
         }
         else if (action instanceof ChestOrientationActionData chestOrientationActionData)
         {
            ChestOrientationActionMessage chestOrientationActionMessage = actionSequenceUpdateMessage.getChestOrientationActions().add();
            chestOrientationActionMessage.getActionInformation().setActionIndex(i);
            chestOrientationActionData.toMessage(chestOrientationActionMessage);
         }
         else if (action instanceof FootstepActionData footstepActionData)
         {
            FootstepActionMessage footstepActionMessage = actionSequenceUpdateMessage.getFootstepActions().add();
            footstepActionMessage.getActionInformation().setActionIndex(i);
            footstepActionData.toMessage(footstepActionMessage);
         }
         else if (action instanceof HandConfigurationActionData handConfigurationActionData)
         {
            HandConfigurationActionMessage handConfigurationActionMessage = actionSequenceUpdateMessage.getHandConfigurationActions().add();
            handConfigurationActionMessage.getActionInformation().setActionIndex(i);
            handConfigurationActionData.toMessage(handConfigurationActionMessage);
         }
         else if (action instanceof HandPoseActionData handPoseActionData)
         {
            HandPoseActionMessage handPoseActionMessage = actionSequenceUpdateMessage.getHandPoseActions().add();
            handPoseActionMessage.getActionInformation().setActionIndex(i);
            handPoseActionData.toMessage(handPoseActionMessage);
         }
         else if (action instanceof HandWrenchActionData handWrenchActionData)
         {
            HandWrenchActionMessage handWrenchActionMessage = actionSequenceUpdateMessage.getHandWrenchActions().add();
            handWrenchActionMessage.getActionInformation().setActionIndex(i);
            handWrenchActionData.toMessage(handWrenchActionMessage);
         }
         else if (action instanceof PelvisHeightActionData pelvisHeightActionData)
         {
            PelvisHeightActionMessage pelvisHeightActionMessage = actionSequenceUpdateMessage.getPelvisHeightActions().add();
            pelvisHeightActionMessage.getActionInformation().setActionIndex(i);
            pelvisHeightActionData.toMessage(pelvisHeightActionMessage);
         }
         else if (action instanceof WaitDurationActionData waitDurationActionData)
         {
            WaitDurationActionMessage waitDurationActionMessage = actionSequenceUpdateMessage.getWaitDurationActions().add();
            waitDurationActionMessage.getActionInformation().setActionIndex(i);
            waitDurationActionData.toMessage(waitDurationActionMessage);
         }
         else if (action instanceof WalkActionData walkActionData)
         {
            WalkActionMessage walkActionMessage = actionSequenceUpdateMessage.getWalkActions().add();
            walkActionMessage.getActionInformation().setActionIndex(i);
            walkActionData.toMessage(walkActionMessage);
         }
      }
   }

   public static boolean isExecuting(RigidBodyTransform desired,
                                     RigidBodyTransform actual,
                                     double translationTolerance,
                                     double rotationTolerance,
                                     double actionNominalDuration,
                                     Timer executionTimer,
                                     Throttler warningThrottler)
   {
      boolean trajectoryTimerRunning = executionTimer.isRunning(actionNominalDuration);

      boolean shouldBeAchieved = !trajectoryTimerRunning;

      // If the timer was ever set, we check if the timer is up and add 100 ms of lee-way
      // before we start printing out warnings.
      if (executionTimer.hasBeenSet())
         shouldBeAchieved &= (actionNominalDuration + 0.1 - executionTimer.getElapsedTime()) <= 0.0;

      boolean desiredPoseAchieved = BehaviorActionSequenceTools.isDesiredPoseAchieved(desired,
                                                                                      actual,
                                                                                      translationTolerance,
                                                                                      rotationTolerance,
                                                                                      shouldBeAchieved,
                                                                                      warningThrottler);

      return trajectoryTimerRunning | !desiredPoseAchieved;
   }

   public static boolean isDesiredPoseAchieved(RigidBodyTransform desired,
                                               RigidBodyTransform actual,
                                               double translationTolerance,
                                               double rotationTolerance,
                                               boolean shouldBeAchieved,
                                               Throttler warningThrottler)
   {
      double translationError = actual.getTranslation().differenceNorm(desired.getTranslation());
      boolean desiredTranslationAcheived = translationError <= translationTolerance;

      double rotationError = actual.getRotation().distance(desired.getRotation(), true);
      boolean desiredRotationAcheived = rotationError <= rotationTolerance;

      boolean desiredAchieved = desiredTranslationAcheived && desiredRotationAcheived;

      if (!desiredAchieved && shouldBeAchieved && warningThrottler.run())
      {
         LogTools.warn("""
                       Desired not achieved.
                          Desired translation acheived: %b
                          Translation error: %.5f
                          Desired rotation achieved: %b
                          Rotation error: %.5f%s
                       """.formatted(desiredTranslationAcheived,
                                     translationError,
                                     desiredRotationAcheived,
                                     Math.toDegrees(rotationError),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL));
      }

      return desiredAchieved;
   }
}
