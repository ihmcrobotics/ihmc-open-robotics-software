package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
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
      if (feetManager.canDoDoubleSupportToeOff(null, transferToSide))
         extraToeOffHeight = feetManager.getToeOffManager().getExtraCoMMaxHeightWithToes();

      NewTransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(transferToSide);
      comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);

      double initialTransferTime = walkingMessageHandler.getInitialTransferTime();

      // Transferring to execute a foot pose, hold current desired in upcoming support foot in case it slips
      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      footstepTiming.setTimings(Double.POSITIVE_INFINITY, initialTransferTime);


      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide.getOppositeSide()), footstepTiming);
      balanceManager.initializeICPPlanForTransfer();
   }
}