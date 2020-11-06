package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StandingState extends WalkingState
{
   private final CommandInputManager commandInputManager;
   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;
   private final TouchdownErrorCompensator touchdownErrorCompensator;

   private final CenterOfMassHeightManager comHeightManager;
   private final BalanceManager balanceManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final LegConfigurationManager legConfigurationManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   public StandingState(CommandInputManager commandInputManager, WalkingMessageHandler walkingMessageHandler, TouchdownErrorCompensator touchdownErrorCompensator,
                        HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
                        WalkingFailureDetectionControlModule failureDetectionControlModule, WalkingControllerParameters walkingControllerParameters,
                        YoRegistry parentRegistry)
   {
      super(WalkingStateEnum.STANDING, parentRegistry);

      this.commandInputManager = commandInputManager;
      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.touchdownErrorCompensator = touchdownErrorCompensator;

      RigidBodyBasics chest = controllerToolbox.getFullRobotModel().getChest();
      if (chest != null)
      {
         ReferenceFrame chestBodyFrame = chest.getBodyFixedFrame();

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hand = controllerToolbox.getFullRobotModel().getHand(robotSide);
            if (hand != null)
            {
               ReferenceFrame handControlFrame = controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide);
               RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame);
               handManagers.put(robotSide, handManager);
            }
         }
      }

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      comHeightManager.setSupportLeg(RobotSide.LEFT);
      balanceManager.computeICPPlan();
   }

   @Override
   public void onEntry()
   {
      commandInputManager.clearAllCommands();

      touchdownErrorCompensator.clear();

      // need to always update biped support polygons after a change to the contact states
      controllerToolbox.updateBipedSupportPolygons();

      balanceManager.resetPushRecovery();
      balanceManager.enablePelvisXYControl();
      balanceManager.initializeICPPlanForStanding();

      NewTransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler
            .createTransferToAndNextFootstepDataForDoubleSupport(RobotSide.RIGHT);
      comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, 0.0);

      walkingMessageHandler.reportWalkingComplete();

      if (pelvisOrientationManager != null)
         pelvisOrientationManager.initializeStanding();

      failureDetectionControlModule.setNextFootstep(null);
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.STANDING);

      for (RobotSide robotSide : RobotSide.values)
      {
         legConfigurationManager.setFullyExtendLeg(robotSide, false);
         legConfigurationManager.setStraight(robotSide);
      }
   }

   @Override
   public void onExit()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handManagers.get(robotSide) != null)
            handManagers.get(robotSide).prepareForLocomotion();
      }

      if (pelvisOrientationManager != null)
      {
         pelvisOrientationManager.prepareForLocomotion(walkingMessageHandler.getNextStepTime());
         comHeightManager.prepareForLocomotion();
      }

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
}