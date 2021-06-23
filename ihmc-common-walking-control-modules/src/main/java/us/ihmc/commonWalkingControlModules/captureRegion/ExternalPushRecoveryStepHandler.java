package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PushRecoveryResultCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ExternalPushRecoveryStepHandler
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final RecyclingArrayList<Footstep> recoveryFootsteps = new RecyclingArrayList<>(Footstep::new);
   private final RecyclingArrayList<FootstepTiming> recoveryFootstepTimings = new RecyclingArrayList<>(FootstepTiming::new);

   private final YoBoolean isRecoveryImpossible = new YoBoolean("isExternalRecoveryImpossible", registry);
   private final PushRecoveryControllerParameters pushRecoveryControllerParameters;

   public ExternalPushRecoveryStepHandler(PushRecoveryControllerParameters pushRecoveryControllerParameters, YoRegistry parentRegistry)
   {
      this.pushRecoveryControllerParameters = pushRecoveryControllerParameters;

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      isRecoveryImpossible.set(false);
   }

   public void consumePushRecoveryResultCommand(PushRecoveryResultCommand command)
   {
      recoveryFootsteps.clear();
      recoveryFootstepTimings.clear();

      isRecoveryImpossible.set(!command.isStepRecoverable());

      FootstepDataListCommand recoverySteps = command.getRecoverySteps();

      double defaultSwingDuration = Double.isNaN(recoverySteps.getDefaultSwingDuration()) ?
            pushRecoveryControllerParameters.getPreferredRecoverySwingDuration() :
            recoverySteps.getDefaultSwingDuration();
      double defaultTransferDuration = Double.isNaN(recoverySteps.getDefaultSwingDuration()) ?
            pushRecoveryControllerParameters.getTransferDurationAfterRecovery() :
            recoverySteps.getDefaultTransferDuration();

      for (int i = 0; i < recoverySteps.getNumberOfFootsteps(); i++)
      {
         FootstepDataCommand footstepCommand = recoverySteps.getFootstep(i);
         Footstep footstep = recoveryFootsteps.add();
         FootstepTiming footstepTiming = recoveryFootstepTimings.add();

         footstep.set(footstepCommand, recoverySteps.isTrustHeightOfFootsteps(), recoverySteps.areFootstepsAdjustable());

         double swingDuration = Double.isNaN(footstepCommand.getSwingDuration()) ? defaultSwingDuration : footstepCommand.getSwingDuration();
         double transferDuration = Double.isNaN(footstepCommand.getTransferDuration()) ? defaultTransferDuration : footstepCommand.getTransferDuration();
         footstepTiming.setTimings(swingDuration, transferDuration);
      }
   }

   public RobotSide getSwingSideForRecovery()
   {
      if (recoveryFootsteps.isEmpty())
         return null;
      else
         return recoveryFootsteps.get(0).getRobotSide();
   }

   public int getNumberOfRecoverySteps()
   {
      return recoveryFootsteps.size();
   }

   public Footstep pollRecoveryStep()
   {
      Footstep footstep = recoveryFootsteps.get(0);
      recoveryFootsteps.remove(0);

      return footstep;
   }

   public FootstepTiming pollRecoveryStepTiming()
   {
      FootstepTiming timing = recoveryFootstepTimings.get(0);
      recoveryFootstepTimings.remove(0);

      return timing;
   }

   public Footstep getRecoveryStep(int stepIndex)
   {
      return recoveryFootsteps.get(stepIndex);
   }

   public FootstepTiming getRecoveryStepTiming(int stepIndex)
   {
      return recoveryFootstepTimings.get(stepIndex);
   }

   public boolean isRecoveryImpossible()
   {
      return isRecoveryImpossible.getBooleanValue();
   }
}
