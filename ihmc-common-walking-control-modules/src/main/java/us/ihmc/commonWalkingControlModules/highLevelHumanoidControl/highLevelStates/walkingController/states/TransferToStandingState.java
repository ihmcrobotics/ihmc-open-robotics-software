package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.referenceFrames.WalkingTrajectoryPath;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TransferToStandingState extends WalkingState
{
   private final YoDouble maxICPErrorToSwitchToStanding = new YoDouble("maxICPErrorToSwitchToStanding", registry);

   private final YoBoolean doFootExplorationInTransferToStanding = new YoBoolean("doFootExplorationInTransferToStanding", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final TouchdownErrorCompensator touchdownErrorCompensator;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;

   private final Point3D midFootPosition = new Point3D();

   public TransferToStandingState(WalkingMessageHandler walkingMessageHandler,
                                  TouchdownErrorCompensator touchdownErrorCompensator,
                                  HighLevelHumanoidControllerToolbox controllerToolbox,
                                  HighLevelControlManagerFactory managerFactory,
                                  WalkingFailureDetectionControlModule failureDetectionControlModule,
                                  YoRegistry parentRegistry)
   {
      super(WalkingStateEnum.TO_STANDING, managerFactory, controllerToolbox, parentRegistry);
      maxICPErrorToSwitchToStanding.set(0.025);

      this.walkingMessageHandler = walkingMessageHandler;
      this.touchdownErrorCompensator = touchdownErrorCompensator;
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      doFootExplorationInTransferToStanding.set(false);
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeICPPlan();
      controllerToolbox.getWalkingTrajectoryPath()
                       .updateTrajectory(feetManager.getCurrentConstraintType(RobotSide.LEFT), feetManager.getCurrentConstraintType(RobotSide.RIGHT));

      switchToPointToeOffIfAlreadyInLine();

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
      if (feetManager.getCurrentConstraintType(sideOnToes) == FootControlModule.ConstraintType.TOES && !feetManager.isUsingPointContactInToeOff(sideOnToes)
          && !feetManager.useToeLineContactInTransfer())
      {
         FramePoint3DReadOnly trailingFootExitCMP = balanceManager.getFirstExitCMPForToeOff(true);
         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(sideOnToes), desiredCoP);
         feetManager.requestPointToeOff(sideOnToes, trailingFootExitCMP, desiredCoP);
      }
   }

   private RobotSide getSideThatCouldBeOnToes()
   {
      WalkingStateEnum previousWalkingState = getPreviousWalkingStateEnum();
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
      WalkingStateEnum previousWalkingState = getPreviousWalkingStateEnum();
      if (previousWalkingState == null)
         return null;

      RobotSide mostSupportingSide = null;
      boolean leftStepLower = leftFootstep.getZ() <= rightFootstep.getZ();
      boolean rightStepLower = leftFootstep.getZ() > rightFootstep.getZ();
      if (previousWalkingState.isSingleSupport() && leftStepLower)
         mostSupportingSide = RobotSide.LEFT;
      else if (previousWalkingState.isSingleSupport() && rightStepLower)
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

      return balanceManager.getICPErrorMagnitude() < maxICPErrorToSwitchToStanding.getDoubleValue();
   }

   @Override
   public void onEntry()
   {
      initializeWalkingTrajectoryPath();
      balanceManager.clearICPPlan();
      balanceManager.clearSwingFootTrajectory();

      WalkingStateEnum previousStateEnum = getPreviousWalkingStateEnum();

      if (previousStateEnum != null && previousStateEnum.isSingleSupport())
         balanceManager.setHoldSplitFractions(true);

      // This can happen if walking is paused or aborted while the robot is on its toes already. In that case
      // restore the full foot contact.
      // TODO don't restore the foot to full contact necessarily. need to figure out how to detect that it's ok
      if (previousStateEnum != null && previousStateEnum.isDoubleSupport())
         feetManager.initializeContactStatesForDoubleSupport(null);

      RobotSide previousSupportSide = null;
      if (previousStateEnum != null)
      {
         if (previousStateEnum.getSupportSide() != null)
            previousSupportSide = previousStateEnum.getSupportSide();
         else if (previousStateEnum.getTransferToSide() != null)
            previousSupportSide = previousStateEnum.getTransferToSide();
      }

      if (doFootExplorationInTransferToStanding.getBooleanValue())
      {
         if (previousSupportSide != null)
         {
            feetManager.initializeFootExploration(previousSupportSide.getOppositeSide());
         }
      }

      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      failureDetectionControlModule.setNextFootstep(null);

      Footstep footstepLeft = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.LEFT);
      Footstep footstepRight = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.RIGHT);
      RobotSide supportingSide = getSideCarryingMostWeight(footstepLeft, footstepRight);
      supportingSide = supportingSide == null ? RobotSide.RIGHT : supportingSide;

      double extraToeOffHeight = 0.0;
      if (feetManager.getCurrentConstraintType(supportingSide.getOppositeSide()) == FootControlModule.ConstraintType.TOES)
         extraToeOffHeight = feetManager.getExtraCoMMaxHeightWithToes();

      TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(supportingSide);
      comHeightManager.setSupportLeg(supportingSide);
      comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      midFootPosition.interpolate(footstepLeft.getFootstepPose().getPosition(), footstepRight.getFootstepPose().getPosition(), 0.5);

      // Just standing in double support, do nothing
      pelvisOrientationManager.centerInMidFeetZUpFrame(finalTransferTime);
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.initializeICPPlanForTransferToStanding();

      touchdownErrorCompensator.clear();
   }

   private void initializeWalkingTrajectoryPath()
   {
      WalkingTrajectoryPath walkingTrajectoryPath = controllerToolbox.getWalkingTrajectoryPath();
      walkingTrajectoryPath.clearFootsteps();
      walkingTrajectoryPath.initializeDoubleSupport();
   }

   @Override
   public void onExit(double timeInState)
   {
   }
}