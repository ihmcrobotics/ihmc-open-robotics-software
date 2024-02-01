package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.referenceFrames.WalkingTrajectoryPath;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class TransferState extends WalkingState
{
   protected final RobotSide transferToSide;

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final WalkingTrajectoryPath walkingTrajectoryPath;
   protected final WalkingFailureDetectionControlModule failureDetectionControlModule;

   protected final CenterOfMassHeightManager comHeightManager;
   protected final PelvisOrientationManager pelvisOrientationManager;
   protected final FeetManager feetManager;

   private final FramePoint2D capturePoint2d = new FramePoint2D();

   private final FramePoint2D desiredCoP = new FramePoint2D();

   private final FootstepTiming stepTiming = new FootstepTiming();

   private final Footstep nextFootstep = new Footstep();

   private final YoBoolean isUnloading;
   private final DoubleProvider unloadFraction;
   private final DoubleProvider rhoMin;

   public TransferState(WalkingStateEnum transferStateEnum,
                        WalkingMessageHandler walkingMessageHandler,
                        HighLevelHumanoidControllerToolbox controllerToolbox,
                        HighLevelControlManagerFactory managerFactory,
                        WalkingFailureDetectionControlModule failureDetectionControlModule,
                        DoubleProvider unloadFraction,
                        DoubleProvider rhoMin,
                        YoRegistry parentRegistry)
   {
      super(transferStateEnum, managerFactory, controllerToolbox, parentRegistry);

      this.transferToSide = transferStateEnum.getTransferToSide();
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.unloadFraction = unloadFraction;
      this.rhoMin = rhoMin;

      walkingTrajectoryPath = controllerToolbox.getWalkingTrajectoryPath();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
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
      //      comHeightManager.setSupportLeg(transferToSide.getOppositeSide());

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

      capturePoint2d.setIncludingFrame(balanceManager.getCapturePoint());
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = controllerToolbox.getBipedSupportPolygons().getSupportPolygonInWorld();

      double distanceToSupport = supportPolygonInWorld.distance(capturePoint2d);

      if (balanceManager.isICPPlanDone() || transferTimeElapsedUnderPrecomputedICPPlan)
      {
         FrameConvexPolygon2DReadOnly nextPolygonInWorld = failureDetectionControlModule.getCombinedFootPolygonWithNextFootstep();
         boolean isICPInsideNextSupportPolygon = nextPolygonInWorld.isPointInside(capturePoint2d);

         // if you're outside at all, but taking the next step will recover, go ahead and do it.
         if (distanceToSupport > 0.0 && (isICPInsideNextSupportPolygon || nextFootstep.getIsAdjustable()))
            return true;
         // if you're done, and your error is low, trigger the next step
         else if (balanceManager.getNormalizedEllipticICPError() < 1.0)
            return true;
         // if you're done, and your error is too high, start using a higher momentum recovery
         else
            balanceManager.setUseMomentumRecoveryModeForBalance(true);
      }

      // if you're too far outside, go ahead and trigger the next step
      return distanceToSupport > balanceManager.getICPDistanceOutsideSupportForStep();
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

         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), desiredCoP);

         FramePoint3DReadOnly trailingFootExitCMP = balanceManager.getFirstExitCMPForToeOff(true);
         feetManager.updateToeOffStatusDoubleSupport(trailingLeg,
                                                     trailingFootExitCMP,
                                                     balanceManager.getDesiredCMP(),
                                                     balanceManager.getDesiredICP(),
                                                     capturePoint2d);

         if (feetManager.okForPointToeOff(false))
         {
            feetManager.requestPointToeOff(trailingLeg, trailingFootExitCMP, desiredCoP);
            return true;
         }
         else if (feetManager.okForLineToeOff(false))
         {
            feetManager.requestLineToeOff(trailingLeg, trailingFootExitCMP, desiredCoP);
            return true;
         }
      }
      // switch to point toe off from line toe off
      else if (!feetManager.isUsingPointContactInToeOff(trailingLeg) && !feetManager.useToeLineContactInTransfer())
      {
         FramePoint3DReadOnly trailingFootExitCMP = balanceManager.getFirstExitCMPForToeOff(true);
         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), desiredCoP);
         feetManager.requestPointToeOff(trailingLeg, trailingFootExitCMP, desiredCoP);
         return true;
      }
      return false;
   }

   @Override
   public void onEntry()
   {
      initializeWalkingTrajectoryPath();
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

      if (feetManager.canDoDoubleSupportToeOff(transferToSide)) // FIXME should this be swing side?
         extraToeOffHeight = feetManager.getExtraCoMMaxHeightWithToes();

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(transferToSide);
      transferToAndNextFootstepsData.setComAtEndOfState(balanceManager.getFinalDesiredCoMPosition());
      comHeightManager.setSupportLeg(transferToSide);
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   protected void initializeWalkingTrajectoryPath()
   {
      walkingTrajectoryPath.clearFootsteps();
      walkingTrajectoryPath.addFootsteps(walkingMessageHandler);
      walkingTrajectoryPath.initializeDoubleSupport();
   }

   protected void updateICPPlan()
   {
      balanceManager.clearICPPlan();
      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states
      balanceManager.setHoldSplitFractions(false);

      if (walkingMessageHandler.hasUpcomingFootsteps())
      {
         walkingMessageHandler.peekFootstep(0, nextFootstep);
         failureDetectionControlModule.setNextFootstep(nextFootstep);
      }
      else
      {
         failureDetectionControlModule.setNextFootstep(null);
      }

      double transferTime = walkingMessageHandler.getNextTransferTime();
      pelvisOrientationManager.setTrajectoryTime(transferTime);
   }

   public boolean isInitialTransfer()
   {
      return getPreviousWalkingStateEnum() == WalkingStateEnum.STANDING;
   }

   @Override
   public void onExit(double timeInState)
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