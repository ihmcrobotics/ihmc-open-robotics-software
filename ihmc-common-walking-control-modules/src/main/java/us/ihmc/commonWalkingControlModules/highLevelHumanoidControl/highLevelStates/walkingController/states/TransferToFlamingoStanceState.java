package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
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
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TransferToFlamingoStanceState extends TransferState
{
   private final FootstepTiming footstepTiming = new FootstepTiming();
   private final FootstepShiftFractions footstepShiftFractions = new FootstepShiftFractions();

   public TransferToFlamingoStanceState(WalkingStateEnum stateEnum, WalkingControllerParameters walkingControllerParameters,
                                        WalkingMessageHandler walkingMessageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
                                        HighLevelControlManagerFactory managerFactory, WalkingFailureDetectionControlModule failureDetectionControlModule,
                                        DoubleProvider unloadFraction, DoubleProvider rhoMin, YoVariableRegistry parentRegistry)
   {
      super(stateEnum, walkingControllerParameters, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, unloadFraction,
            rhoMin, parentRegistry);
   }

   @Override
   public void doAction(double timeInState)
   {
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

      if (!comHeightManager.hasBeenInitializedWithNextStep())
      {
         TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(transferToSide);
         comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);
      }

      double initialTransferTime = walkingMessageHandler.getInitialTransferTime();
      Footstep footstep = walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide);
      FixedFramePoint3DBasics transferFootPosition = footstep.getFootstepPose().getPosition();
      RobotSide swingSide = transferToSide.getOppositeSide();
      comHeightManager.transfer(transferFootPosition, initialTransferTime, swingSide, extraToeOffHeight);

      // Transferring to execute a foot pose, hold current desired in upcoming support foot in case it slips
      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

      double swingTime = Double.POSITIVE_INFINITY;
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      double finalTransferSplitFraction = walkingMessageHandler.getFinalTransferSplitFraction();
      footstepTiming.setTimings(Double.POSITIVE_INFINITY, initialTransferTime);

      double swingDurationShiftFraction = walkingMessageHandler.getDefaultSwingDurationShiftFraction();
      double swingSplitFraction = walkingMessageHandler.getDefaultSwingSplitFraction();
      double transferSplitFraction = walkingMessageHandler.getDefaultTransferSplitFraction();
      footstepShiftFractions.setShiftFractions(swingDurationShiftFraction, swingSplitFraction, transferSplitFraction);

      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.setFinalTransferSplitFraction(finalTransferSplitFraction);
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide.getOppositeSide()), footstepTiming, footstepShiftFractions);
      balanceManager.setICPPlanTransferToSide(transferToSide);
      balanceManager.initializeICPPlanForTransfer(swingTime, initialTransferTime, finalTransferTime);
   }
}