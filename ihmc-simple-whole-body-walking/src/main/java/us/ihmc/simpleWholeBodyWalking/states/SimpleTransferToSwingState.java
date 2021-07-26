package us.ihmc.simpleWholeBodyWalking.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleTransferToSwingState extends SimpleTransferState
{
   private static final int numberOfFootstepsToConsider = 3;
   private final Footstep[] footsteps = Footstep.createFootsteps(numberOfFootstepsToConsider);
   private final FootstepTiming[] footstepTimings = FootstepTiming.createTimings(numberOfFootstepsToConsider);
   private final FootstepShiftFractions[] footstepShiftFractions = FootstepShiftFractions.createShiftFractions(numberOfFootstepsToConsider);

   private final YoDouble currentTransferDuration = new YoDouble("CurrentTransferDuration", registry);

   public SimpleTransferToSwingState(SimpleWalkingStateEnum stateEnum,
                                     WalkingMessageHandler walkingMessageHandler,
                                     HighLevelHumanoidControllerToolbox controllerToolbox,
                                     SimpleControlManagerFactory managerFactory,
                                     WalkingFailureDetectionControlModule failureDetectionControlModule,
                                     YoRegistry parentRegistry)
   {
      super(stateEnum, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, parentRegistry);
   }

   @Override
   protected void updateICPPlan()
   {
      super.updateICPPlan();

      // This needs to check `TO_STANDING` as well as messages could be received on the very first controller tick at which point
      // the robot is not in the standing state but not yet walking either.
      if (getPreviousWalkingStateEnum() == SimpleWalkingStateEnum.STANDING || getPreviousWalkingStateEnum() == SimpleWalkingStateEnum.TO_STANDING)
      {
         walkingMessageHandler.reportWalkingStarted();
      }

      if (isInitialTransfer())
      {
         pelvisOrientationManager.moveToAverageInSupportFoot(transferToSide);
      }
      else
      {
         // In middle of walking or leaving foot pose, pelvis is good leave it like that.
         pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);
      }

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      walkingMessageHandler.requestPlanarRegions();
      balanceManager.setFinalTransferTime(finalTransferTime);

      int stepsToAdd = Math.min(numberOfFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
      if (stepsToAdd < 1)
      {
         throw new RuntimeException("Can not go to walking single support if there are no upcoming footsteps.");
      }
      for (int i = 0; i < stepsToAdd; i++)
      {
         Footstep footstep = footsteps[i];
         FootstepTiming timing = footstepTimings[i];
         walkingMessageHandler.peekFootstep(i, footstep);
         walkingMessageHandler.peekTiming(i, timing);

         balanceManager.addFootstepToPlan(footstep, timing);
      }

      balanceManager.setICPPlanTransferToSide(transferToSide);
      FootstepTiming firstTiming = footstepTimings[0];
      currentTransferDuration.set(firstTiming.getTransferTime());
      balanceManager.initializeICPPlanForTransfer(finalTransferTime);

      pelvisOrientationManager.setUpcomingFootstep(footsteps[0]);
      pelvisOrientationManager.initializeTransfer(transferToSide, firstTiming.getTransferTime(), firstTiming.getSwingTime());
   }
}