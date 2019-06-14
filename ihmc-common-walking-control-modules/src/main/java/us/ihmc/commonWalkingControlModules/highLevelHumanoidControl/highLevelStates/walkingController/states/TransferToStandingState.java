package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TransferToStandingState extends WalkingState
{
   private final YoDouble maxICPErrorToSwitchToStanding = new YoDouble("maxICPErrorToSwitchToStanding", registry);

   private final YoBoolean doFootExplorationInTransferToStanding = new YoBoolean("doFootExplorationInTransferToStanding", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final BalanceManager balanceManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final LegConfigurationManager legConfigurationManager;

   private final Point3D midFootPosition = new Point3D();

   public TransferToStandingState(WalkingMessageHandler walkingMessageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
         HighLevelControlManagerFactory managerFactory, WalkingFailureDetectionControlModule failureDetectionControlModule, YoVariableRegistry parentRegistry)
   {
      super(WalkingStateEnum.TO_STANDING, parentRegistry);
      maxICPErrorToSwitchToStanding.set(0.025);

      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      doFootExplorationInTransferToStanding.set(false);
   }

   @Override
   public void doAction(double timeInState)
   {
      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      comHeightManager.setSupportLeg(RobotSide.LEFT);
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
      balanceManager.clearICPPlan();
      balanceManager.resetPushRecovery();

      WalkingStateEnum previousStateEnum = getPreviousWalkingStateEnum();

      // This can happen if walking is paused or aborted while the robot is on its toes already. In that case
      // restore the full foot contact.
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

      TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler
            .createTransferToAndNextFootstepDataForDoubleSupport(RobotSide.LEFT);
      double extraToeOffHeight = 0.0;
      comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      Footstep footstepLeft = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.LEFT);
      Footstep footstepRight = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.LEFT);
      midFootPosition.interpolate(footstepLeft.getFootstepPose().getPosition(), footstepRight.getFootstepPose().getPosition(), 0.5);
      comHeightManager.transfer(midFootPosition, finalTransferTime);

      // Just standing in double support, do nothing
      pelvisOrientationManager.centerInMidFeetZUpFrame(finalTransferTime);
      balanceManager.setICPPlanTransferFromSide(previousSupportSide);
      balanceManager.initializeICPPlanForTransferToStanding(finalTransferTime);

      if (previousSupportSide != null)
      {
         RobotSide previousSwingSide = previousSupportSide.getOppositeSide();
         legConfigurationManager.setFullyExtendLeg(previousSwingSide, false);
         legConfigurationManager.beginStraightening(previousSwingSide);
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            legConfigurationManager.setFullyExtendLeg(robotSide, false);
            legConfigurationManager.setStraight(robotSide);
         }
      }
   }

   @Override
   public void onExit()
   {
   }
}