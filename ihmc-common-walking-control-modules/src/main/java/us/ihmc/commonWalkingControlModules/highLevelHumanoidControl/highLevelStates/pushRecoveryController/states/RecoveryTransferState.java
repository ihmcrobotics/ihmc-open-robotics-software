package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryBalanceManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RecoveryTransferState extends PushRecoveryState
{
   private final YoDouble currentTransferDuration = new YoDouble("CurrentTransferDuration", registry);

   private final BooleanProvider minimizeAngularMomentumRateZDuringTransfer;

   protected final RobotSide transferToSide;

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final HighLevelHumanoidControllerToolbox controllerToolbox;
   protected final WalkingFailureDetectionControlModule failureDetectionControlModule;

   protected final CenterOfMassHeightManager comHeightManager;
   protected final PushRecoveryBalanceManager balanceManager;
   protected final PelvisOrientationManager pelvisOrientationManager;
   protected final FeetManager feetManager;

   private final FramePoint2D capturePoint2d = new FramePoint2D();

   private final FramePoint2D filteredDesiredCoP = new FramePoint2D();
   private final FramePoint2D desiredCoP = new FramePoint2D();

   private final FramePoint2D capturePoint = new FramePoint2D();

   private final MultiStepPushRecoveryControlModule pushRecoveryControlModule;

   private final FootstepTiming stepTiming = new FootstepTiming();

   private final Footstep nextFootstep = new Footstep();

   public RecoveryTransferState(PushRecoveryStateEnum stateEnum,
                                WalkingMessageHandler walkingMessageHandler,
                                HighLevelHumanoidControllerToolbox controllerToolbox,
                                PushRecoveryControlManagerFactory managerFactory,
                                PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                MultiStepPushRecoveryControlModule pushRecoveryControlModule,
                                WalkingFailureDetectionControlModule failureDetectionControlModule,
                                YoRegistry parentRegistry)
   {
      super(stateEnum, parentRegistry);

      this.transferToSide = stateEnum.getTransferToSide();
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.controllerToolbox = controllerToolbox;
      this.balanceManager = managerFactory.getOrCreateBalanceManager();
      this.pushRecoveryControlModule = pushRecoveryControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      minimizeAngularMomentumRateZDuringTransfer = new BooleanParameter("minimizeAngularMomentumRateZDuringTransfer", registry,
              pushRecoveryControllerParameters.minimizeAngularMomentumRateZDuringTransfer());
   }


   @Override
   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   public boolean isInitialTransfer()
   {
      return false;  //TODO check
   }

   private void updateICPPlan()
   {
//      balanceManager.clearICPPlan();
      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      failureDetectionControlModule.setNextFootstep(null);

      pelvisOrientationManager.setTrajectoryTime(stepTiming.getTransferTime());

      // In middle of walking or leaving foot pose, pelvis is good leave it like that.
      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      walkingMessageHandler.requestPlanarRegions();
      balanceManager.setFinalTransferTime(finalTransferTime);

      currentTransferDuration.set(stepTiming.getTransferTime());
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.initializeICPPlanForTransfer();

//      pelvisOrientationManager.setUpcomingFootstep(footsteps);
      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);
      pelvisOrientationManager.initializeTransfer(transferToSide, stepTiming.getTransferTime(), stepTiming.getSwingTime());
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeICPPlan();

      switchToToeOffIfPossible();

      feetManager.updateContactStatesInDoubleSupport(transferToSide);

      capturePoint.setIncludingFrame(controllerToolbox.getCapturePoint());
      pushRecoveryControlModule.updateForDoubleSupport(capturePoint, controllerToolbox.getOmega0());

      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      //      comHeightManager.setSupportLeg(transferToSide.getOppositeSide());

   }

   @Override
   public void onEntry()
   {
      stepTiming.set(pushRecoveryControlModule.pollRecoveryStepTiming(transferToSide));
      nextFootstep.set(pushRecoveryControlModule.pollRecoveryStep(transferToSide));
      updateICPPlan();

      double extraToeOffHeight = 0.0;
      if (feetManager.canDoDoubleSupportToeOff(nextFootstep.getFootstepPose().getPosition(), transferToSide)) // FIXME should this be swing side?
         extraToeOffHeight = feetManager.getToeOffManager().getExtraCoMMaxHeightWithToes();

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(transferToSide);
      transferToAndNextFootstepsData.setComAtEndOfState(balanceManager.getFinalDesiredCoMPosition());
      comHeightManager.setSupportLeg(transferToSide);
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);

      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringTransfer.getValue());
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return balanceManager.isICPPlanDone();
      /*
      if (balanceManager.isICPPlanDone())
      {
         capturePoint2d.setIncludingFrame(balanceManager.getCapturePoint());
         FrameConvexPolygon2DReadOnly supportPolygonInWorld = controllerToolbox.getBipedSupportPolygons().getSupportPolygonInWorld();
         FrameConvexPolygon2DReadOnly nextPolygonInWorld = failureDetectionControlModule.getCombinedFootPolygonWithNextFootstep();

         double distanceToSupport = supportPolygonInWorld.distance(capturePoint2d);
         boolean isICPInsideNextSupportPolygon = nextPolygonInWorld.isPointInside(capturePoint2d);

         if (distanceToSupport > balanceManager.getICPDistanceOutsideSupportForStep() || (distanceToSupport > 0.0 && isICPInsideNextSupportPolygon))
            return true;
//         else if (balanceManager.getNormalizedEllipticICPError() < 1.0)
//            return true;
      }

      return feetManager.isFootToeingOffSlipping(transferToSide.getOppositeSide());

       */
   }

   @Override
   public void onExit()
   {
      feetManager.reset();

      balanceManager.minimizeAngularMomentumRateZ(false);
   }

   /**
    * @return whether or not it switched
    */
   public boolean switchToToeOffIfPossible()
   {
      RobotSide trailingLeg = transferToSide.getOppositeSide();

      if (feetManager.getCurrentConstraintType(trailingLeg) != FootControlModule.ConstraintType.TOES)
      {
         capturePoint2d.setIncludingFrame(balanceManager.getCapturePoint());

         controllerToolbox.getFilteredDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), filteredDesiredCoP);
         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), desiredCoP);

         FramePoint3DReadOnly trailingFootExitCMP = balanceManager.getFirstExitCMPForToeOff(true);
         feetManager.updateToeOffStatusDoubleSupport(trailingLeg,
                                                     nextFootstep,
                                                     trailingFootExitCMP,
                                                     balanceManager.getDesiredCMP(),
                                                     desiredCoP,
                                                     balanceManager.getDesiredICP(),
                                                     capturePoint2d,
                                                     balanceManager.getFinalDesiredICP());

         if (feetManager.okForPointToeOff())
         {
            feetManager.requestPointToeOff(trailingLeg, trailingFootExitCMP, filteredDesiredCoP);
            return true;
         }
         else if (feetManager.okForLineToeOff())
         {
            feetManager.requestLineToeOff(trailingLeg, trailingFootExitCMP, filteredDesiredCoP);
            return true;
         }
      }
      // switch to point toe off from line toe off
      else if (!feetManager.isUsingPointContactInToeOff(trailingLeg) && !feetManager.useToeLineContactInTransfer())
      {
         FramePoint3DReadOnly trailingFootExitCMP = balanceManager.getFirstExitCMPForToeOff(true);
         controllerToolbox.getFilteredDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), filteredDesiredCoP);
         feetManager.requestPointToeOff(trailingLeg, trailingFootExitCMP, filteredDesiredCoP);
         return true;
      }
      return false;
   }

}