package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StandingState extends WalkingState
{
   private static final boolean holdDesiredHeightConstantWhenStanding = false;

   private final CommandInputManager commandInputManager;
   private final WalkingMessageHandler walkingMessageHandler;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;
   private final TouchdownErrorCompensator touchdownErrorCompensator;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final RigidBodyControlManager chestManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();
   private final FeetManager feetManager;

   private RobotSide supportingSide;

   public StandingState(CommandInputManager commandInputManager,
                        WalkingMessageHandler walkingMessageHandler,
                        TouchdownErrorCompensator touchdownErrorCompensator,
                        HighLevelHumanoidControllerToolbox controllerToolbox,
                        HighLevelControlManagerFactory managerFactory,
                        WalkingFailureDetectionControlModule failureDetectionControlModule,
                        WalkingControllerParameters walkingControllerParameters,
                        YoRegistry parentRegistry)
   {
      super(WalkingStateEnum.STANDING, managerFactory, controllerToolbox, parentRegistry);

      this.commandInputManager = commandInputManager;
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.touchdownErrorCompensator = touchdownErrorCompensator;

      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         if (hand != null)
            handManagers.put(robotSide, managerFactory.getRigidBodyManager(hand));
      }

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();

      RigidBodyBasics chest = fullRobotModel.getChest();
      if (chest != null)
         chestManager = managerFactory.getRigidBodyManager(chest);
      else
         chestManager = null;

      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      if (!holdDesiredHeightConstantWhenStanding)
         comHeightManager.setSupportLeg(supportingSide);
      balanceManager.computeICPPlan();
      controllerToolbox.getWalkingTrajectoryPath().updateTrajectory(feetManager.getCurrentConstraintType(RobotSide.LEFT),
                                                                    feetManager.getCurrentConstraintType(RobotSide.RIGHT),
                                                                    true);
   }

   @Override
   public void onEntry()
   {
      commandInputManager.clearAllCommands();
      feetManager.reset();

      touchdownErrorCompensator.clear();

      // need to always update biped support polygons after a change to the contact states
      controllerToolbox.updateBipedSupportPolygons();

      balanceManager.enablePelvisXYControl();
      balanceManager.initializeICPPlanForStanding();
      balanceManager.setHoldSplitFractions(false);

      if (holdDesiredHeightConstantWhenStanding)
      {
         comHeightManager.initializeToNominalDesiredHeight();
         supportingSide = RobotSide.LEFT;
      }
      else
      {
         Footstep footstepLeft = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.LEFT);
         Footstep footstepRight = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.RIGHT);

         supportingSide = getSideCarryingMostWeight(footstepLeft, footstepRight);
         supportingSide = supportingSide == null ? RobotSide.LEFT : supportingSide;

         TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(supportingSide.getOppositeSide());
         comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, 0.0);
      }

      walkingMessageHandler.reportWalkingComplete();

      failureDetectionControlModule.setNextFootstep(null);
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.STANDING);
   }

   @Override
   public void onExit(double timeInState)
   {
      feetManager.saveCurrentPositionsAsLastFootstepPositions();
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handManagers.get(robotSide) != null)
            handManagers.get(robotSide).prepareForLocomotion();
      }

      if (chestManager != null)
         chestManager.prepareForLocomotion();

      if (pelvisOrientationManager != null)
         pelvisOrientationManager.prepareForLocomotion(walkingMessageHandler.getNextStepTime());
      if (comHeightManager != null)
         comHeightManager.prepareForLocomotion();

      balanceManager.disablePelvisXYControl();
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.IN_MOTION);
   }

   @Override
   public boolean isStateSafeToConsumePelvisTrajectoryCommand()
   {
      return true;
   }

   @Override
   public boolean isStateSafeToConsumeManipulationCommands()
   {
      return true;
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return true;
   }

   private RobotSide getSideCarryingMostWeight(Footstep leftFootstep, Footstep rightFootstep)
   {
      WalkingStateEnum previousWalkingState = getPreviousWalkingStateEnum();
      if (previousWalkingState == null)
         return null;

      RobotSide mostSupportingSide = null;
      boolean leftStepLower = leftFootstep.getZ() <= rightFootstep.getZ();
      boolean rightStepLower = leftFootstep.getZ() > rightFootstep.getZ();
      if (leftStepLower)
         mostSupportingSide = RobotSide.LEFT;
      else if (rightStepLower)
         mostSupportingSide = RobotSide.RIGHT;
      else if (previousWalkingState.getTransferToSide() != null)
         mostSupportingSide = previousWalkingState.getTransferToSide().getOppositeSide();

      return mostSupportingSide;
   }
}