package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingState;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbortWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AdjustFootstepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AutomaticManipulationAbortCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EndEffectorLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandComplianceControlParametersCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeadTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineDesiredAccelerationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class WalkingCommandConsumer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable isAutomaticManipulationAbortEnabled = new BooleanYoVariable("isAutomaticManipulationAbortEnabled", registry);
   private final BooleanYoVariable hasManipulationBeenAborted = new BooleanYoVariable("hasManipulationBeenAborted", registry);
   private final DoubleYoVariable icpErrorThresholdToAbortManipulation = new DoubleYoVariable("icpErrorThresholdToAbortManipulation", registry);
   private final DoubleYoVariable minimumDurationBetweenTwoManipulationAborts = new DoubleYoVariable("minimumDurationBetweenTwoManipulationAborts", registry);
   private final DoubleYoVariable timeOfLastManipulationAbortRequest = new DoubleYoVariable("timeOfLastManipulationAbortRequest", registry);
   private final DoubleYoVariable manipulationIgnoreInputsDurationAfterAbort = new DoubleYoVariable("manipulationIgnoreInputsDurationAfterAbort", registry);

   private final DoubleYoVariable yoTime;
   private final WalkingMessageHandler walkingMessageHandler;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final BalanceManager balanceManager;
   private final CenterOfMassHeightManager comHeightManager;

   private final RigidBodyControlManager chestManager;
   private final RigidBodyControlManager headManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   public WalkingCommandConsumer(CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager, HighLevelHumanoidControllerToolbox controllerToolbox, WalkingMessageHandler walkingMessageHandler, HighLevelControlManagerFactory managerFactory,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      this.walkingMessageHandler = walkingMessageHandler;
      yoTime = controllerToolbox.getYoTime();

      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;

      RigidBody head = controllerToolbox.getFullRobotModel().getHead();
      RigidBody chest = controllerToolbox.getFullRobotModel().getChest();
      RigidBody pelvis = controllerToolbox.getFullRobotModel().getPelvis();

      ReferenceFrame pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();
      ReferenceFrame chestBodyFrame = chest.getBodyFixedFrame();
      ReferenceFrame headBodyFrame = head.getBodyFixedFrame();

      this.chestManager = managerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestBodyFrame, pelvisZUpFrame);
      this.headManager = managerFactory.getOrCreateRigidBodyManager(head, chest, headBodyFrame, chestBodyFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = controllerToolbox.getFullRobotModel().getHand(robotSide);
         ReferenceFrame handControlFrame = controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide);
         RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame);
         handManagers.put(robotSide, handManager);
      }

      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();

      isAutomaticManipulationAbortEnabled.set(walkingControllerParameters.allowAutomaticManipulationAbort());
      icpErrorThresholdToAbortManipulation.set(0.04);
      minimumDurationBetweenTwoManipulationAborts.set(5.0);
      manipulationIgnoreInputsDurationAfterAbort.set(2.0);
      timeOfLastManipulationAbortRequest.set(Double.NEGATIVE_INFINITY);

      parentRegistry.addChild(registry);
   }

   public void consumeHeadCommands()
   {
      if (commandInputManager.isNewCommandAvailable(HeadTrajectoryCommand.class))
         headManager.handleTaskspaceTrajectoryCommand(commandInputManager.pollNewestCommand(HeadTrajectoryCommand.class));
      if (commandInputManager.isNewCommandAvailable(NeckTrajectoryCommand.class))
         headManager.handleJointspaceTrajectoryCommand(commandInputManager.pollNewestCommand(NeckTrajectoryCommand.class));
      if (commandInputManager.isNewCommandAvailable(NeckDesiredAccelerationsCommand.class))
         headManager.handleDesiredAccelerationsCommand(commandInputManager.pollNewestCommand(NeckDesiredAccelerationsCommand.class));
   }

   public void consumeChestCommands()
   {
      if (commandInputManager.isNewCommandAvailable(ChestTrajectoryCommand.class))
         chestManager.handleTaskspaceTrajectoryCommand(commandInputManager.pollNewestCommand(ChestTrajectoryCommand.class));
      if (commandInputManager.isNewCommandAvailable(SpineTrajectoryCommand.class))
         chestManager.handleJointspaceTrajectoryCommand(commandInputManager.pollNewestCommand(SpineTrajectoryCommand.class));
      if (commandInputManager.isNewCommandAvailable(SpineDesiredAccelerationCommand.class))
         chestManager.handleDesiredAccelerationsCommand(commandInputManager.pollNewestCommand(SpineDesiredAccelerationCommand.class));
   }

   public void consumePelvisHeightCommands()
   {
      if (commandInputManager.isNewCommandAvailable(PelvisHeightTrajectoryCommand.class))
         comHeightManager.handlePelvisHeightTrajectoryCommand(commandInputManager.pollNewestCommand(PelvisHeightTrajectoryCommand.class));
   }

   public void consumeGoHomeMessages()
   {
      if (!commandInputManager.isNewCommandAvailable(GoHomeCommand.class))
         return;

      GoHomeCommand command = commandInputManager.pollAndCompileCommands(GoHomeCommand.class);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (command.getRequest(robotSide, BodyPart.ARM))
            handManagers.get(robotSide).goHome(command.getTrajectoryTime());
      }

      if (command.getRequest(BodyPart.PELVIS))
      {
         pelvisOrientationManager.goToHomeFromCurrentDesired(command.getTrajectoryTime());
         balanceManager.goHome();
         comHeightManager.goHome(command.getTrajectoryTime());
      }

      if (command.getRequest(BodyPart.CHEST))
      {
         chestManager.goHome(command.getTrajectoryTime());
      }
   }

   public void consumePelvisCommands(WalkingState currentState, boolean allowMotionRegardlessOfState)
   {
      if (commandInputManager.isNewCommandAvailable(PelvisOrientationTrajectoryCommand.class))
      {
         PelvisOrientationTrajectoryCommand newestCommand = commandInputManager.pollNewestCommand(PelvisOrientationTrajectoryCommand.class);
         if (allowMotionRegardlessOfState || currentState.isStateSafeToConsumePelvisTrajectoryCommand())
            pelvisOrientationManager.handlePelvisOrientationTrajectoryCommands(newestCommand);
      }

      if (commandInputManager.isNewCommandAvailable(PelvisTrajectoryCommand.class))
      {
         PelvisTrajectoryCommand command = commandInputManager.pollNewestCommand(PelvisTrajectoryCommand.class);
         if (allowMotionRegardlessOfState || currentState.isStateSafeToConsumePelvisTrajectoryCommand())
         {
            pelvisOrientationManager.handlePelvisTrajectoryCommand(command);
            balanceManager.handlePelvisTrajectoryCommand(command);
            comHeightManager.handlePelvisTrajectoryCommand(command);
         }
      }
   }

   public void consumeManipulationCommands(WalkingState currentState, boolean allowMotionRegardlessOfState)
   {
      if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < manipulationIgnoreInputsDurationAfterAbort.getDoubleValue())
      {
         commandInputManager.flushCommands(HandTrajectoryCommand.class);
         commandInputManager.flushCommands(ArmTrajectoryCommand.class);
         commandInputManager.flushCommands(ArmDesiredAccelerationsCommand.class);
         commandInputManager.flushCommands(HandComplianceControlParametersCommand.class);
         return;
      }

      List<HandTrajectoryCommand> handTrajectoryCommands = commandInputManager.pollNewCommands(HandTrajectoryCommand.class);
      List<ArmTrajectoryCommand> armTrajectoryCommands = commandInputManager.pollNewCommands(ArmTrajectoryCommand.class);
      List<ArmDesiredAccelerationsCommand> armDesiredAccelerationCommands = commandInputManager.pollNewCommands(ArmDesiredAccelerationsCommand.class);
      List<HandComplianceControlParametersCommand> handComplianceCommands = commandInputManager.pollNewCommands(HandComplianceControlParametersCommand.class);

      if (allowMotionRegardlessOfState || currentState.isStateSafeToConsumeManipulationCommands())
      {
         for (int i = 0; i < handTrajectoryCommands.size(); i++)
         {
            HandTrajectoryCommand command = handTrajectoryCommands.get(i);
            RobotSide robotSide = command.getRobotSide();
            if (handManagers.get(robotSide) != null)
               handManagers.get(robotSide).handleTaskspaceTrajectoryCommand(command);
         }

         for (int i = 0; i < armTrajectoryCommands.size(); i++)
         {
            ArmTrajectoryCommand command = armTrajectoryCommands.get(i);
            RobotSide robotSide = command.getRobotSide();
            if (handManagers.get(robotSide) != null)
               handManagers.get(robotSide).handleJointspaceTrajectoryCommand(command);
         }

         for (int i = 0; i < armDesiredAccelerationCommands.size(); i++)
         {
            ArmDesiredAccelerationsCommand command = armDesiredAccelerationCommands.get(i);
            RobotSide robotSide = command.getRobotSide();
            if (handManagers.get(robotSide) != null)
               handManagers.get(robotSide).handleDesiredAccelerationsCommand(command);
         }

         for (int i = 0; i < handComplianceCommands.size(); i++)
            PrintTools.info(HandComplianceControlParametersCommand.class.getSimpleName() + " not implemented.");
      }
   }

   public void handleAutomaticManipulationAbortOnICPError(WalkingState currentState)
   {
      if (!currentState.isStateSafeToConsumeManipulationCommands())
         return;

      if (commandInputManager.isNewCommandAvailable(AutomaticManipulationAbortCommand.class))
      {
         AutomaticManipulationAbortCommand message = commandInputManager.pollNewestCommand(AutomaticManipulationAbortCommand.class);
         isAutomaticManipulationAbortEnabled.set(message.isEnable());
      }

      if (!isAutomaticManipulationAbortEnabled.getBooleanValue())
         return;

      if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < minimumDurationBetweenTwoManipulationAborts.getDoubleValue())
         return;

      if (balanceManager.getICPErrorMagnitude() > icpErrorThresholdToAbortManipulation.getDoubleValue())
      {
         hasManipulationBeenAborted.set(true);

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyControlManager handManager = handManagers.get(robotSide);
            handManager.holdInJointspace();
            handManager.resetJointIntegrators();
         }

         timeOfLastManipulationAbortRequest.set(yoTime.getDoubleValue());

         statusMessageOutputManager.reportStatusMessage(new ManipulationAbortedStatus());
      }
      else
      {
         hasManipulationBeenAborted.set(false);
      }
   }

   public void consumeEndEffectorLoadBearingCommands(WalkingState currentState)
   {
      if (!commandInputManager.isNewCommandAvailable(EndEffectorLoadBearingCommand.class))
         return;

      EndEffectorLoadBearingCommand command = commandInputManager.pollAndCompileCommands(EndEffectorLoadBearingCommand.class);

//      for (RobotSide robotSide : RobotSide.values)
//      {
//         if (handManagers.get(robotSide) == null)
//            continue;
//
//         LoadBearingRequest request = command.getRequest(robotSide, EndEffector.HAND);
//         if (request == LoadBearingRequest.LOAD)
//            handManagers.get(robotSide).requestLoadBearing();
//         else if (request == LoadBearingRequest.UNLOAD)
//            handManagers.get(robotSide).holdInJointspace();
//      }

      currentState.handleEndEffectorLoadBearingCommand(command);
   }

   public void consumeLoadBearingCommands()
   {
      List<HandLoadBearingCommand> handLoadBearingCommands = commandInputManager.pollNewCommands(HandLoadBearingCommand.class);

      for (int i = 0; i < handLoadBearingCommands.size(); i++)
      {
         HandLoadBearingCommand command = handLoadBearingCommands.get(i);
         RobotSide robotSide = command.getRobotSide();
         if (handManagers.get(robotSide) != null)
            handManagers.get(robotSide).handleLoadBearingCommand(command);
      }
   }

   public void consumeStopAllTrajectoryCommands()
   {
      if (!commandInputManager.isNewCommandAvailable(StopAllTrajectoryCommand.class))
         return;

      StopAllTrajectoryCommand command = commandInputManager.pollNewestCommand(StopAllTrajectoryCommand.class);
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handManagers.get(robotSide) != null)
            handManagers.get(robotSide).handleStopAllTrajectoryCommand(command);
      }
      chestManager.handleStopAllTrajectoryCommand(command);
      feetManager.handleStopAllTrajectoryCommand(command);
      comHeightManager.handleStopAllTrajectoryCommand(command);
      balanceManager.handleStopAllTrajectoryCommand(command);
      pelvisOrientationManager.handleStopAllTrajectoryCommand(command);
   }

   public void consumeFootCommands()
   {
      if (commandInputManager.isNewCommandAvailable(FootTrajectoryCommand.class))
         walkingMessageHandler.handleFootTrajectoryCommand(commandInputManager.pollNewCommands(FootTrajectoryCommand.class));

      if (commandInputManager.isNewCommandAvailable(PauseWalkingCommand.class))
         walkingMessageHandler.handlePauseWalkingCommand(commandInputManager.pollNewestCommand(PauseWalkingCommand.class));

      if (commandInputManager.isNewCommandAvailable(FootstepDataListCommand.class))
      {
         walkingMessageHandler.handleFootstepDataListCommand(commandInputManager.pollNewestCommand(FootstepDataListCommand.class));
      }

      if (commandInputManager.isNewCommandAvailable(AdjustFootstepCommand.class))
         walkingMessageHandler.handleAdjustFootstepCommand(commandInputManager.pollNewestCommand(AdjustFootstepCommand.class));
   }

   public void consumeAbortWalkingCommands(BooleanYoVariable abortWalkingRequested)
   {
      if (!commandInputManager.isNewCommandAvailable(AbortWalkingCommand.class))
         return;
      abortWalkingRequested.set(commandInputManager.pollNewestCommand(AbortWalkingCommand.class).isAbortWalkingRequested());
   }
}
