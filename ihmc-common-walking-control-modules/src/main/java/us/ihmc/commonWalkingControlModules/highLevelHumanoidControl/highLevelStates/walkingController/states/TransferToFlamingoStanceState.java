package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TransferToFlamingoStanceState extends TransferState
{
   private final FootstepTiming footstepTiming = new FootstepTiming();

   public TransferToFlamingoStanceState(WalkingStateEnum stateEnum, WalkingMessageHandler walkingMessageHandler,
                                        HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
                                        WalkingFailureDetectionControlModule failureDetectionControlModule,
                                        DoubleProvider unloadFraction, DoubleProvider rhoMin, YoRegistry parentRegistry)
   {
      super(stateEnum, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, unloadFraction,
            rhoMin, parentRegistry);
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeFlamingoStateICPPlan();

      switchToToeOffIfPossible();
      super.doAction(timeInState);
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      double extraToeOffHeight = 0.0;
      if (feetManager.canDoDoubleSupportToeOff(transferToSide))
         extraToeOffHeight = feetManager.getExtraCoMMaxHeightWithToes();

      TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(transferToSide);
      comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);

      double initialTransferTime = walkingMessageHandler.getInitialTransferTime();

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      footstepTiming.setTimings(Double.POSITIVE_INFINITY, initialTransferTime);


      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide.getOppositeSide()), footstepTiming);
      balanceManager.initializeICPPlanForTransfer();
   }
}