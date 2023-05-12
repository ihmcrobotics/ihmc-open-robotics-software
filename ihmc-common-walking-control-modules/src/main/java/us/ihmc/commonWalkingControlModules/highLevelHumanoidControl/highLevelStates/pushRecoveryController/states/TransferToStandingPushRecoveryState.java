package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryBalanceManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TransferToStandingPushRecoveryState extends PushRecoveryState
{
   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PushRecoveryBalanceManager balanceManager;
   private final FeetManager feetManager;

   private final PushRecoveryControllerParameters pushRecoveryParameters;

   private final FramePoint2D capturePoint = new FramePoint2D();

   private final MultiStepPushRecoveryControlModule pushRecoveryCalculator;

   public TransferToStandingPushRecoveryState(WalkingMessageHandler walkingMessageHandler,
                                              HighLevelHumanoidControllerToolbox controllerToolbox,
                                              PushRecoveryControllerParameters pushRecoveryParameters,
                                              MultiStepPushRecoveryControlModule pushRecoveryControlModule,
                                              PushRecoveryControlManagerFactory managerFactory,
                                              WalkingFailureDetectionControlModule failureDetectionControlModule,
                                              YoRegistry parentRegistry)
   {
      super(PushRecoveryStateEnum.TO_STANDING, parentRegistry);

      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.balanceManager = managerFactory.getOrCreateBalanceManager();
      this.pushRecoveryCalculator = pushRecoveryControlModule;
      this.pushRecoveryParameters = pushRecoveryParameters;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeICPPlan();

      switchToPointToeOffIfAlreadyInLine();

      capturePoint.set(controllerToolbox.getCapturePoint());
      pushRecoveryCalculator.updateForDoubleSupport(capturePoint, controllerToolbox.getOmega0());

      RobotSide supportingSide = getSideCarryingMostWeight();
      balanceManager.computeNormalizedEllipticICPError(supportingSide);

      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      comHeightManager.setSupportLeg(RobotSide.LEFT);
   }

   private final FramePoint2D desiredCoP = new FramePoint2D();

   public void switchToPointToeOffIfAlreadyInLine()
   {
      RobotSide sideOnToes = getSideThatCouldBeOnToes();

      if (sideOnToes == null)
         return;

      // switch to point toe off from line toe off
      if (feetManager.getCurrentConstraintType(sideOnToes) == FootControlModule.ConstraintType.TOES && !feetManager.isUsingPointContactInToeOff(sideOnToes) && !feetManager.useToeLineContactInTransfer())
      {
         FramePoint3DReadOnly trailingFootExitCMP = balanceManager.getFirstExitCMPForToeOff(true);
         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(sideOnToes), desiredCoP);
         feetManager.requestPointToeOff(sideOnToes, trailingFootExitCMP, desiredCoP);
      }
   }

   private RobotSide getSideThatCouldBeOnToes()
   {
      PushRecoveryStateEnum previousWalkingState = getPreviousWalkingStateEnum();
      if (previousWalkingState == null)
         return null;

      RobotSide sideOnToes = null;
      if (previousWalkingState.isSingleSupport())
         sideOnToes = previousWalkingState.getSupportSide();
      else if (previousWalkingState.getTransferToSide() != null)
         sideOnToes = previousWalkingState.getTransferToSide().getOppositeSide();

      return sideOnToes;
   }

   private final FramePoint3D leftFootPosition = new FramePoint3D();
   private final FramePoint3D rightFootPosition = new FramePoint3D();

   private RobotSide getSideCarryingMostWeight()
   {
      PushRecoveryStateEnum previousWalkingState = getPreviousWalkingStateEnum();
      if (previousWalkingState == null)
         return RobotSide.RIGHT;

      leftFootPosition.setFromReferenceFrame(controllerToolbox.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      rightFootPosition.setFromReferenceFrame(controllerToolbox.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));

      boolean leftStepLower = leftFootPosition.getZ() <= rightFootPosition.getZ();
      boolean rightStepLower = leftFootPosition.getZ() > rightFootPosition.getZ();

      RobotSide mostSupportingSide;
      if (previousWalkingState.isSingleSupport() && leftStepLower)
         mostSupportingSide = RobotSide.LEFT;
      else if(previousWalkingState.isSingleSupport() && rightStepLower)
         mostSupportingSide = RobotSide.RIGHT;
      else if (previousWalkingState.getTransferToSide() != null)
         mostSupportingSide = previousWalkingState.getTransferToSide().getOppositeSide();
      else
         mostSupportingSide = RobotSide.RIGHT;

      return mostSupportingSide;
   }


   @Override
   public boolean isDone(double timeInState)
   {
      if (!balanceManager.isICPPlanDone())
         return false;

      if (pushRecoveryCalculator.isRobotFallingFromDoubleSupport() != null)
         return false;

      return balanceManager.getNormalizedEllipticICPError() < 1.0;
   }

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();

      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      walkingMessageHandler.clearFootsteps();
      walkingMessageHandler.setFinalTransferTime(pushRecoveryParameters.getTransferDurationAfterRecovery());

      capturePoint.setIncludingFrame(controllerToolbox.getCapturePoint());
      pushRecoveryCalculator.updateForDoubleSupport(capturePoint, controllerToolbox.getOmega0());

      failureDetectionControlModule.setNextFootstep(null);

      RobotSide supportingSide = getSideCarryingMostWeight();

      double extraToeOffHeight = 0.0;
      if (feetManager.getCurrentConstraintType(supportingSide.getOppositeSide()) == FootControlModule.ConstraintType.TOES)
         extraToeOffHeight = feetManager.getExtraCoMMaxHeightWithToes();

      TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler
            .createTransferToAndNextFootstepDataForDoubleSupport(supportingSide);
      comHeightManager.setSupportLeg(supportingSide);
      comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);

      double finalTransferTime = pushRecoveryParameters.getFinalTransferDurationForRecovery();

      // Just standing in double support, do nothing
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.initializeICPPlanForTransferToStanding();
   }

   @Override
   public void onExit(double timeInState)
   {
   }
}