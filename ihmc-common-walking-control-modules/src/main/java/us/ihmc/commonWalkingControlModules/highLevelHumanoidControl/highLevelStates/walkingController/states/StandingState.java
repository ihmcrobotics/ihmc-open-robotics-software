package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PrepareForLocomotionCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class StandingState extends WalkingState
{
   private final CommandInputManager commandInputManager;
   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final BalanceManager balanceManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final LegConfigurationManager legConfigurationManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   private final YoBoolean doPrepareManipulationForLocomotion = new YoBoolean("doPrepareManipulationForLocomotion", registry);
   private final YoBoolean doPreparePelvisForLocomotion = new YoBoolean("doPreparePelvisForLocomotion", registry);

   public StandingState(CommandInputManager commandInputManager, WalkingMessageHandler walkingMessageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
         HighLevelControlManagerFactory managerFactory, WalkingFailureDetectionControlModule failureDetectionControlModule,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      super(WalkingStateEnum.STANDING, parentRegistry);

      this.commandInputManager = commandInputManager;
      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      
      RigidBody chest = controllerToolbox.getFullRobotModel().getChest();
      if(chest != null)
      {
         ReferenceFrame chestBodyFrame = chest.getBodyFixedFrame();
         
         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBody hand = controllerToolbox.getFullRobotModel().getHand(robotSide);
            if(hand != null)
            {
               ReferenceFrame handControlFrame = controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide);
               RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame, trajectoryFrames);
               handManagers.put(robotSide, handManager);
            }
         }
      }

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      doPrepareManipulationForLocomotion.set(walkingControllerParameters.doPrepareManipulationForLocomotion());
      doPreparePelvisForLocomotion.set(walkingControllerParameters.doPreparePelvisForLocomotion());
   }

   @Override
   public void doAction()
   {
      comHeightManager.setSupportLeg(RobotSide.LEFT);
      consumePrepareForLocomotion();
   }

   @Override
   public void doTransitionIntoAction()
   {
      consumePrepareForLocomotion();
      commandInputManager.flushAllCommands();

      balanceManager.clearICPPlan();
      balanceManager.resetPushRecovery();
      balanceManager.enablePelvisXYControl();

      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

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
   public void doTransitionOutOfAction()
   {
      if (doPrepareManipulationForLocomotion.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (handManagers.get(robotSide) != null)
               handManagers.get(robotSide).holdInJointspace();
         }
      }

      if (pelvisOrientationManager != null && doPreparePelvisForLocomotion.getBooleanValue())
      {
         pelvisOrientationManager.prepareForLocomotion();
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

   private void consumePrepareForLocomotion()
   {
      if (commandInputManager.isNewCommandAvailable(PrepareForLocomotionCommand.class))
      {
         PrepareForLocomotionCommand command = commandInputManager.pollNewestCommand(PrepareForLocomotionCommand.class);
         doPrepareManipulationForLocomotion.set(command.isPrepareManipulation());
         doPreparePelvisForLocomotion.set(command.isPreparePelvis());
      }
   }

   @Override
   public boolean isDone()
   {
      return true;
   }
}