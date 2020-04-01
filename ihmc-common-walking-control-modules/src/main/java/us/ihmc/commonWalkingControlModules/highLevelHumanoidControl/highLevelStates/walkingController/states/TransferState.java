package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import boofcv.struct.flow.ImageFlow;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class TransferState extends WalkingState
{
   protected final RobotSide transferToSide;

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final HighLevelHumanoidControllerToolbox controllerToolbox;
   protected final WalkingFailureDetectionControlModule failureDetectionControlModule;

   protected final CenterOfMassHeightManager comHeightManager;
   protected final BalanceManager balanceManager;
   protected final PelvisOrientationManager pelvisOrientationManager;
   protected final FeetManager feetManager;

   private final FramePoint2D desiredICPLocal = new FramePoint2D();
   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCMP = new FramePoint2D();

   private final FramePoint2D filteredDesiredCoP = new FramePoint2D();
   private final FramePoint2D desiredCoP = new FramePoint2D();
   private final FramePoint3D nextExitCMP = new FramePoint3D();

   private final FootstepTiming stepTiming = new FootstepTiming();

   private final Footstep nextFootstep = new Footstep();

   private final YoBoolean isUnloading;
   private final DoubleProvider unloadFraction;
   private final DoubleProvider rhoMin;

   public TransferState(WalkingStateEnum transferStateEnum, WalkingMessageHandler walkingMessageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
                        HighLevelControlManagerFactory managerFactory, WalkingFailureDetectionControlModule failureDetectionControlModule,
                        DoubleProvider unloadFraction, DoubleProvider rhoMin, YoVariableRegistry parentRegistry)
   {
      super(transferStateEnum, parentRegistry);
      this.transferToSide = transferStateEnum.getTransferToSide();
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.controllerToolbox = controllerToolbox;
      this.unloadFraction = unloadFraction;
      this.rhoMin = rhoMin;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      if (unloadFraction != null)
      {
         isUnloading = new YoBoolean(transferToSide.getOppositeSide().getLowerCaseName() + "FootIsUnloading", registry);
      }
      else
      {
         isUnloading = null;
      }
   }

   @Override
   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   @Override
   public void doAction(double timeInState)
   {
      feetManager.updateContactStatesInDoubleSupport(transferToSide);

      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      comHeightManager.setSupportLeg(transferToSide);

      balanceManager.computeNormalizedEllipticICPError(transferToSide);

      if (isUnloading != null && unloadFraction.getValue() > 0.0)
      {
         double percentInTransfer = MathTools.clamp(timeInState / stepTiming.getTransferTime(), 0.0, 1.0);

         if (!isUnloading.getValue())
         {
            isUnloading.set(percentInTransfer > unloadFraction.getValue());
         }

         if (isUnloading.getValue())
         {
            double nominalPercentInUnloading = (percentInTransfer - unloadFraction.getValue()) / (1.0 - unloadFraction.getValue());
            double icpBasedPercentInUnloading = 1.0 - MathTools.clamp(balanceManager.getNormalizedEllipticICPError() - 1.0, 0.0, 1.0);
            double percentInUnloading = Math.min(nominalPercentInUnloading, icpBasedPercentInUnloading);
            feetManager.unload(transferToSide.getOppositeSide(), percentInUnloading, rhoMin.getValue());
         }
      }

      if (balanceManager.getNormalizedEllipticICPError() > balanceManager.getEllipticICPErrorForMomentumRecovery())
         balanceManager.setUseMomentumRecoveryModeForBalance(true);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      //If we're using a precomputed icp trajectory we can't rely on the icp planner's state to dictate when to exit transfer.
      boolean transferTimeElapsedUnderPrecomputedICPPlan = false;
      if (balanceManager.isPrecomputedICPPlannerActive())
      {
         transferTimeElapsedUnderPrecomputedICPPlan = timeInState > walkingMessageHandler.getNextTransferTime();
      }

      if (balanceManager.isICPPlanDone() || transferTimeElapsedUnderPrecomputedICPPlan)
      {
         balanceManager.getCapturePoint(capturePoint2d);
         FrameConvexPolygon2DReadOnly supportPolygonInWorld = controllerToolbox.getBipedSupportPolygons().getSupportPolygonInWorld();
         FrameConvexPolygon2DReadOnly nextPolygonInWorld = failureDetectionControlModule.getCombinedFootPolygonWithNextFootstep();

         double distanceToSupport = supportPolygonInWorld.distance(capturePoint2d);
         boolean isICPInsideNextSupportPolygon = nextPolygonInWorld.isPointInside(capturePoint2d);

         if (distanceToSupport > balanceManager.getICPDistanceOutsideSupportForStep() || (distanceToSupport > 0.0 && isICPInsideNextSupportPolygon))
            return true;
         else if (balanceManager.getNormalizedEllipticICPError() < 1.0)
            return true;
         else
            balanceManager.setUseMomentumRecoveryModeForBalance(true);
      }

      return false;
   }

   public void switchToToeOffIfPossible()
   {
      RobotSide trailingLeg = transferToSide.getOppositeSide();

      boolean shouldComputeToeLineContact = feetManager.shouldComputeToeLineContact();
      boolean shouldComputeToePointContact = feetManager.shouldComputeToePointContact();

      if (shouldComputeToeLineContact || shouldComputeToePointContact)
      {
         balanceManager.getDesiredCMP(desiredCMP);
         balanceManager.getDesiredICP(desiredICPLocal);
         balanceManager.getCapturePoint(capturePoint2d);
         balanceManager.getNextExitCMP(nextExitCMP);

         controllerToolbox.getFilteredDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), filteredDesiredCoP);
         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), desiredCoP);

         feetManager.updateToeOffStatusDoubleSupport(trailingLeg, nextFootstep, nextExitCMP, desiredCMP, desiredCoP, desiredICPLocal, capturePoint2d);

         if (feetManager.okForPointToeOff() && shouldComputeToePointContact)
            feetManager.requestPointToeOff(trailingLeg, nextExitCMP, filteredDesiredCoP);
         else if (feetManager.okForLineToeOff() && shouldComputeToeLineContact)
            feetManager.requestLineToeOff(trailingLeg, nextExitCMP, filteredDesiredCoP);
      }
   }

   @Override
   public void onEntry()
   {
      updateICPPlan();

      if (walkingMessageHandler.hasUpcomingFootsteps())
      {
         walkingMessageHandler.peekTiming(0, stepTiming);
      }
      else
      {
         stepTiming.setTimings(Double.NaN, Double.NaN);
      }

      double extraToeOffHeight = 0.0;
      RobotSide swingSide = transferToSide.getOppositeSide();
      if (feetManager.canDoDoubleSupportToeOff(nextFootstep, swingSide))
         extraToeOffHeight = feetManager.getToeOffManager().getExtraCoMMaxHeightWithToes();

      Footstep footstep = walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide);
      FixedFramePoint3DBasics transferFootPosition = footstep.getFootstepPose().getPosition();
      double transferTime = walkingMessageHandler.getNextTransferTime();
      comHeightManager.transfer(transferFootPosition, transferTime, swingSide, extraToeOffHeight);
   }

   protected void updateICPPlan()
   {
      balanceManager.clearICPPlan();
      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      if (walkingMessageHandler.hasUpcomingFootsteps())
      {
         walkingMessageHandler.peekFootstep(0, nextFootstep);
         failureDetectionControlModule.setNextFootstep(nextFootstep);
         balanceManager.setUpcomingFootstep(nextFootstep);
      }
      else
      {
         failureDetectionControlModule.setNextFootstep(null);
         balanceManager.setUpcomingFootstep(null);
      }

      balanceManager.resetPushRecovery();

      double transferTime = walkingMessageHandler.getNextTransferTime();
      pelvisOrientationManager.setTrajectoryTime(transferTime);
   }

   public boolean isInitialTransfer()
   {
      return getPreviousWalkingStateEnum() == WalkingStateEnum.STANDING;
   }

   @Override
   public void onExit()
   {
      if (isUnloading != null)
      {
         isUnloading.set(false);
         feetManager.resetLoadConstraints(transferToSide.getOppositeSide());
      }
      feetManager.reset();
      balanceManager.setUseMomentumRecoveryModeForBalance(false);
   }
}