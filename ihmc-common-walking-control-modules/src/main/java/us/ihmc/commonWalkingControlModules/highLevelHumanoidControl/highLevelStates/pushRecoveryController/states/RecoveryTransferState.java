package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import org.apache.commons.math3.util.Precision;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryBalanceManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class RecoveryTransferState extends PushRecoveryState
{
   private final Footstep footsteps;
   private final FootstepTiming footstepTimings;

   private final DoubleProvider minimumTransferTime;
   private final DoubleProvider minimumSwingTime;

   private final YoDouble currentTransferDuration = new YoDouble("CurrentTransferDuration", registry);

   private final YoDouble originalTransferTime = new YoDouble("OriginalTransferTime", registry);
   private final BooleanProvider minimizeAngularMomentumRateZDuringTransfer;

   private final FrameVector3D tempAngularVelocity = new FrameVector3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();

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

   private final FootstepTiming stepTiming = new FootstepTiming();

   private final Footstep nextFootstep = new Footstep();

   private final YoBoolean isUnloading;
   private final DoubleProvider unloadFraction;
   private final DoubleProvider rhoMin;

   public RecoveryTransferState(PushRecoveryStateEnum stateEnum,
                                WalkingMessageHandler walkingMessageHandler,
                                HighLevelHumanoidControllerToolbox controllerToolbox,
                                PushRecoveryControlManagerFactory managerFactory,
                                PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                WalkingFailureDetectionControlModule failureDetectionControlModule,
                                DoubleProvider minimumTransferTime,
                                DoubleProvider minimumSwingTime,
                                DoubleProvider unloadFraction,
                                DoubleProvider rhoMin,
                                YoRegistry parentRegistry)
   {
      super(stateEnum, parentRegistry);

      this.transferToSide = stateEnum.getTransferToSide();
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.controllerToolbox = controllerToolbox;
      this.unloadFraction = unloadFraction;
      this.rhoMin = rhoMin;
      this.balanceManager = managerFactory.getOrCreateBalanceManager();

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

      this.minimumTransferTime = minimumTransferTime;
      this.minimumSwingTime = minimumSwingTime;

      minimizeAngularMomentumRateZDuringTransfer = new BooleanParameter("minimizeAngularMomentumRateZDuringTransfer", registry,
              pushRecoveryControllerParameters.minimizeAngularMomentumRateZDuringTransfer());

      footsteps = new Footstep();
      footstepTimings = new FootstepTiming();
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
      balanceManager.clearICPPlan();
      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states
      balanceManager.setHoldSplitFractions(false);

      failureDetectionControlModule.setNextFootstep(null);

      balanceManager.resetPushRecovery();

      double transferTime = walkingMessageHandler.getNextTransferTime();
      pelvisOrientationManager.setTrajectoryTime(transferTime);

      // In middle of walking or leaving foot pose, pelvis is good leave it like that.
      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      walkingMessageHandler.requestPlanarRegions();
      balanceManager.setFinalTransferTime(finalTransferTime);

      currentTransferDuration.set(minimumTransferTime.getValue());
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.initializeICPPlanForTransferToRecovery();

//      pelvisOrientationManager.setUpcomingFootstep(footsteps);
      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);
      pelvisOrientationManager.initializeTransfer(transferToSide, minimumTransferTime.getValue(), minimumSwingTime.getValue());
   }

   @Override
   public void doAction(double timeInState)
   {
      RobotSide swingSide = transferToSide.getOppositeSide();
      feetManager.updateSwingTrajectoryPreview(swingSide);
      balanceManager.computeICPPlan();

      if (!doManualLiftOff())
      {
         if (switchToToeOffIfPossible())
            feetManager.initializeSwingTrajectoryPreview(swingSide, footsteps, footstepTimings.getSwingTime());
      }

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

      double transferDuration = currentTransferDuration.getDoubleValue();

      double toeOffDuration = footstepTimings.getLiftoffDuration();
      if (doManualLiftOff() && transferDuration - timeInState < toeOffDuration)
      {
         Footstep upcomingFootstep = footsteps;
         FrameSE3TrajectoryPoint firstWaypoint = upcomingFootstep.getSwingTrajectory().get(0);
         MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(transferToSide.getOppositeSide());
         tempOrientation.setIncludingFrame(firstWaypoint.getOrientation());
         tempOrientation.changeFrame(soleZUpFrame);
         tempAngularVelocity.setIncludingFrame(firstWaypoint.getAngularVelocity());
         tempAngularVelocity.changeFrame(soleZUpFrame); // The y component is equivalent to the pitch rate since the yaw and roll rate are 0.0
         feetManager.liftOff(transferToSide.getOppositeSide(), tempOrientation.getPitch(), tempAngularVelocity.getY(), toeOffDuration);
      }
   }

   private boolean doManualLiftOff()
   {
      Footstep upcomingFootstep = footsteps;
      return upcomingFootstep.getTrajectoryType() == TrajectoryType.WAYPOINTS && Precision.equals(upcomingFootstep.getSwingTrajectory().get(0).getTime(), 0.0);
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
      if (feetManager.canDoDoubleSupportToeOff(nextFootstep.getFootstepPose().getPosition(), transferToSide)) // FIXME should this be swing side?
         extraToeOffHeight = feetManager.getToeOffManager().getExtraCoMMaxHeightWithToes();

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(transferToSide);
      transferToAndNextFootstepsData.setComAtEndOfState(balanceManager.getFinalDesiredCoMPosition());
      comHeightManager.setSupportLeg(transferToSide);
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);

      feetManager.initializeSwingTrajectoryPreview(transferToSide.getOppositeSide(), footsteps, footstepTimings.getSwingTime());
      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringTransfer.getValue());
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (balanceManager.isICPPlanDone())
      {
         capturePoint2d.setIncludingFrame(balanceManager.getCapturePoint());
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

      return feetManager.isFootToeingOffSlipping(transferToSide.getOppositeSide());
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

      balanceManager.minimizeAngularMomentumRateZ(false);
   }

   /**
    * This method checks if the upcoming step has a desired absolute start time. If that is the case
    * the transfer time is adjusted such that the swing starts at the correct time.
    */
   private void adjustTiming(FootstepTiming stepTiming)
   {
      if (!stepTiming.hasAbsoluteTime())
      {
         originalTransferTime.setToNaN();
         return;
      }

      double originalSwingTime = stepTiming.getSwingTime();
      double originalTransferTime = stepTiming.getTransferTime();
      this.originalTransferTime.set(originalTransferTime);

      double currentTime = controllerToolbox.getYoTime().getDoubleValue();
      double timeInFootstepPlan = currentTime - stepTiming.getExecutionStartTime();
      double adjustedTransferTime = stepTiming.getSwingStartTime() - timeInFootstepPlan;

      // make sure transfer does not get too short
      adjustedTransferTime = Math.max(adjustedTransferTime, minimumTransferTime.getValue());

      // GW TODO - possible improvement:
      // If the adjustment is capped by the minimum transfer time adjust also the upcoming transfer times here. That
      // would make the ICP plan for the upcoming steps more accurate. However, if the given original transfer times
      // are correctly set this might be a minimal improvement that makes step timing more complicated and difficult
      // to debug. If we have big adjustments a lot we should revisit this.

      // keep swing times and only adjust transfers for now
      stepTiming.setTimings(originalSwingTime, adjustedTransferTime);
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