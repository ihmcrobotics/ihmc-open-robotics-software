package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import javafx.geometry.Side;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryCalculator;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryBalanceManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;

public class TransferToStandingPushRecoveryState extends PushRecoveryState
{
   private final YoDouble maxICPErrorToSwitchToStanding = new YoDouble("maxICPErrorToSwitchToStanding", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PushRecoveryBalanceManager balanceManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;

   private final Point3D midFootPosition = new Point3D();
   private final FramePoint2D capturePoint = new FramePoint2D();

   private final MultiStepPushRecoveryControlModule pushRecoveryCalculator;

   public TransferToStandingPushRecoveryState(WalkingMessageHandler walkingMessageHandler,
                                              HighLevelHumanoidControllerToolbox controllerToolbox,
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

      maxICPErrorToSwitchToStanding.set(0.025);

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeICPPlan();

      switchToPointToeOffIfAlreadyInLine();

      pushRecoveryCalculator.updateForDoubleSupport(capturePoint, controllerToolbox.getOmega0());

      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      comHeightManager.setSupportLeg(RobotSide.LEFT);
   }

   private final FramePoint2D filteredDesiredCoP = new FramePoint2D();

   public void switchToPointToeOffIfAlreadyInLine()
   {
      RobotSide sideOnToes = getSideThatCouldBeOnToes();

      if (sideOnToes == null)
         return;

      // switch to point toe off from line toe off
      if (feetManager.getCurrentConstraintType(sideOnToes) == FootControlModule.ConstraintType.TOES && !feetManager.isUsingPointContactInToeOff(sideOnToes) && !feetManager.useToeLineContactInTransfer())
        {
           FramePoint3DReadOnly trailingFootExitCMP = balanceManager.getFirstExitCMPForToeOff(true);
            controllerToolbox.getFilteredDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(sideOnToes), filteredDesiredCoP);
            feetManager.requestPointToeOff(sideOnToes, trailingFootExitCMP, filteredDesiredCoP);
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

   private RobotSide getSideCarryingMostWeight(Footstep leftFootstep, Footstep rightFootstep)
   {
      PushRecoveryStateEnum previousWalkingState = getPreviousWalkingStateEnum();
      if (previousWalkingState == null)
         return null;

      RobotSide mostSupportingSide = null;
      boolean leftStepLower = leftFootstep.getZ() <= rightFootstep.getZ();
      boolean rightStepLower = leftFootstep.getZ() > rightFootstep.getZ();
      if(previousWalkingState.isSingleSupport() && leftStepLower)
         mostSupportingSide = RobotSide.LEFT;
      else if(previousWalkingState.isSingleSupport() && rightStepLower)
         mostSupportingSide = RobotSide.RIGHT;
      else if (previousWalkingState.getTransferToSide() != null)
         mostSupportingSide = previousWalkingState.getTransferToSide().getOppositeSide();

      return mostSupportingSide;
   }


   @Override
   public boolean isDone(double timeInState)
   {
      if (!balanceManager.isICPPlanDone())
         return false;

      if (pushRecoveryCalculator.isRobotFallingFromDoubleSupport() != null)
         return false;

      // FIXME likely remove this
      return balanceManager.getICPErrorMagnitude() < maxICPErrorToSwitchToStanding.getDoubleValue();
   }

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();

      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      walkingMessageHandler.clearFootsteps();

      capturePoint.setIncludingFrame(controllerToolbox.getCapturePoint());
      pushRecoveryCalculator.updateForDoubleSupport(capturePoint, controllerToolbox.getOmega0());

      failureDetectionControlModule.setNextFootstep(null);

      Footstep footstepLeft = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.LEFT);
      Footstep footstepRight = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.RIGHT);
      RobotSide supportingSide = getSideCarryingMostWeight(footstepLeft, footstepRight);
      supportingSide = supportingSide == null ? RobotSide.RIGHT : supportingSide;

      double extraToeOffHeight = 0.0;
      if (feetManager.getCurrentConstraintType(supportingSide.getOppositeSide()) == FootControlModule.ConstraintType.TOES)
         extraToeOffHeight = feetManager.getToeOffManager().getExtraCoMMaxHeightWithToes();

      TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler
            .createTransferToAndNextFootstepDataForDoubleSupport(supportingSide);
      comHeightManager.setSupportLeg(supportingSide);
      comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      midFootPosition.interpolate(footstepLeft.getFootstepPose().getPosition(), footstepRight.getFootstepPose().getPosition(), 0.5);

      // Just standing in double support, do nothing
      pelvisOrientationManager.centerInMidFeetZUpFrame(finalTransferTime);
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.initializeICPPlanForTransferToStanding();
   }



   @Override
   public void onExit()
   {
   }
}