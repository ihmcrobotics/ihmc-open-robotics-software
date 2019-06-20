package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import java.util.List;

import controller_msgs.msg.dds.ManipulationAbortedStatus;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingState;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbortWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AdjustFootstepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AutomaticManipulationAbortCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestHybridJointspaceTaskspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.DesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandHybridJointspaceTaskspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeadHybridJointspaceTaskspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeadTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.LoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.MomentumTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WrenchTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingCommandConsumer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean isAutomaticManipulationAbortEnabled = new YoBoolean("isAutomaticManipulationAbortEnabled", registry);
   private final YoBoolean hasManipulationBeenAborted = new YoBoolean("hasManipulationBeenAborted", registry);
   private final YoDouble icpErrorThresholdToAbortManipulation = new YoDouble("icpErrorThresholdToAbortManipulation", registry);
   private final YoDouble minimumDurationBetweenTwoManipulationAborts = new YoDouble("minimumDurationBetweenTwoManipulationAborts", registry);
   private final YoDouble timeOfLastManipulationAbortRequest = new YoDouble("timeOfLastManipulationAbortRequest", registry);
   private final YoDouble manipulationIgnoreInputsDurationAfterAbort = new YoDouble("manipulationIgnoreInputsDurationAfterAbort", registry);
   private final YoDouble allowManipulationAbortAfterThisTime = new YoDouble("allowManipulationAbortAfterThisTime", registry);

   private final YoDouble yoTime;
   private final WalkingMessageHandler walkingMessageHandler;

   private final CommandConsumerWithDelayBuffers commandConsumerWithDelayBuffers;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final BalanceManager balanceManager;
   private final CenterOfMassHeightManager comHeightManager;

   private final RigidBodyControlManager chestManager;
   private final RigidBodyControlManager headManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   private final ManipulationAbortedStatus manipulationAbortedStatus = new ManipulationAbortedStatus();

   public WalkingCommandConsumer(CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager, HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      this.walkingMessageHandler = controllerToolbox.getWalkingMessageHandler();
      yoTime = controllerToolbox.getYoTime();

      this.commandConsumerWithDelayBuffers = new CommandConsumerWithDelayBuffers(commandInputManager, controllerToolbox.getYoTime());
      this.statusMessageOutputManager = statusMessageOutputManager;

      RigidBodyBasics head = controllerToolbox.getFullRobotModel().getHead();
      RigidBodyBasics chest = controllerToolbox.getFullRobotModel().getChest();
      RigidBodyBasics pelvis = controllerToolbox.getFullRobotModel().getPelvis();

      ReferenceFrame pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();

      ReferenceFrame chestBodyFrame = null;
      if(chest != null)
      {
         chestBodyFrame = chest.getBodyFixedFrame();
         this.chestManager = managerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestBodyFrame, pelvisZUpFrame);
      }
      else
      {
         chestManager = null;
      }

      if (head != null)
      {
         ReferenceFrame headBodyFrame = head.getBodyFixedFrame();
         this.headManager = managerFactory.getOrCreateRigidBodyManager(head, chest, headBodyFrame, chestBodyFrame);
      }
      else
      {
         this.headManager = null;
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = controllerToolbox.getFullRobotModel().getHand(robotSide);
         if(hand != null)
         {
            ReferenceFrame handControlFrame = controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide);
            RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame);
            handManagers.put(robotSide, handManager);
         }
      }

      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();

      isAutomaticManipulationAbortEnabled.set(walkingControllerParameters.allowAutomaticManipulationAbort());
      icpErrorThresholdToAbortManipulation.set(walkingControllerParameters.getICPErrorThresholdForManipulationAbort());
      minimumDurationBetweenTwoManipulationAborts.set(5.0);
      manipulationIgnoreInputsDurationAfterAbort.set(2.0);
      timeOfLastManipulationAbortRequest.set(Double.NEGATIVE_INFINITY);
      allowManipulationAbortAfterThisTime.set(Double.NEGATIVE_INFINITY);

      parentRegistry.addChild(registry);
   }

   public void avoidManipulationAbortForDuration(double durationToAvoidAbort)
   {
      allowManipulationAbortAfterThisTime.set(yoTime.getDoubleValue() + durationToAvoidAbort);
   }

   public void update()
   {
      commandConsumerWithDelayBuffers.update();
   }

   public void consumeHeadCommands()
   {
      if (headManager == null)
      {
         return;
      }

      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(HeadTrajectoryCommand.class))
      {
         HeadTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(HeadTrajectoryCommand.class);
         SO3TrajectoryControllerCommand so3Trajectory = command.getSO3Trajectory();
         so3Trajectory.setSequenceId(command.getSequenceId());
         headManager.handleTaskspaceTrajectoryCommand(so3Trajectory);
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(NeckTrajectoryCommand.class))
      {
         NeckTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(NeckTrajectoryCommand.class);
         JointspaceTrajectoryCommand jointspaceTrajectory = command.getJointspaceTrajectory();
         jointspaceTrajectory.setSequenceId(command.getSequenceId());
         headManager.handleJointspaceTrajectoryCommand(jointspaceTrajectory);
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(NeckDesiredAccelerationsCommand.class))
      {
         NeckDesiredAccelerationsCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(NeckDesiredAccelerationsCommand.class);
         DesiredAccelerationsCommand desiredAccelerations = command.getDesiredAccelerations();
         desiredAccelerations.setSequenceId(command.getSequenceId());
         headManager.handleDesiredAccelerationsCommand(desiredAccelerations);
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(HeadHybridJointspaceTaskspaceTrajectoryCommand.class))
      {
         HeadHybridJointspaceTaskspaceTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(HeadHybridJointspaceTaskspaceTrajectoryCommand.class);
         SO3TrajectoryControllerCommand taskspaceTrajectoryCommand = command.getTaskspaceTrajectoryCommand();
         JointspaceTrajectoryCommand jointspaceTrajectoryCommand = command.getJointspaceTrajectoryCommand();
         taskspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
         jointspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
         headManager.handleHybridTrajectoryCommand(taskspaceTrajectoryCommand, jointspaceTrajectoryCommand);
      }
   }

   public void consumeChestCommands()
   {
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(ChestTrajectoryCommand.class))
      {
         ChestTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(ChestTrajectoryCommand.class);
         SO3TrajectoryControllerCommand so3Trajectory = command.getSO3Trajectory();
         so3Trajectory.setSequenceId(command.getSequenceId());
         chestManager.handleTaskspaceTrajectoryCommand(so3Trajectory);
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(SpineTrajectoryCommand.class))
      {
         SpineTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(SpineTrajectoryCommand.class);
         JointspaceTrajectoryCommand jointspaceTrajectory = command.getJointspaceTrajectory();
         jointspaceTrajectory.setSequenceId(command.getSequenceId());
         chestManager.handleJointspaceTrajectoryCommand(jointspaceTrajectory);
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(SpineDesiredAccelerationsCommand.class))
      {
         SpineDesiredAccelerationsCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(SpineDesiredAccelerationsCommand.class);
         DesiredAccelerationsCommand desiredAccelerations = command.getDesiredAccelerations();
         desiredAccelerations.setSequenceId(command.getSequenceId());
         chestManager.handleDesiredAccelerationsCommand(desiredAccelerations);
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(ChestHybridJointspaceTaskspaceTrajectoryCommand.class))
      {
         ChestHybridJointspaceTaskspaceTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(ChestHybridJointspaceTaskspaceTrajectoryCommand.class);
         SO3TrajectoryControllerCommand taskspaceTrajectoryCommand = command.getTaskspaceTrajectoryCommand();
         JointspaceTrajectoryCommand jointspaceTrajectoryCommand = command.getJointspaceTrajectoryCommand();
         taskspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
         jointspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
         chestManager.handleHybridTrajectoryCommand(taskspaceTrajectoryCommand, jointspaceTrajectoryCommand);
      }
   }

   public void consumePelvisHeightCommands()
   {
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(PelvisHeightTrajectoryCommand.class))
      {
         PelvisHeightTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(PelvisHeightTrajectoryCommand.class);
         comHeightManager.handlePelvisHeightTrajectoryCommand(command);
      }
   }

   public void consumeGoHomeMessages()
   {
      if (!commandConsumerWithDelayBuffers.isNewCommandAvailable(GoHomeCommand.class))
         return;

      List<GoHomeCommand> commands = commandConsumerWithDelayBuffers.pollNewCommands(GoHomeCommand.class);
      for(int i = 0; i < commands.size(); i++)
      {
         GoHomeCommand command = commands.get(i);

         for (RobotSide robotSide : RobotSide.values)
         {
            if (command.getRequest(robotSide, HumanoidBodyPart.ARM))
            {
               RigidBodyControlManager handManager = handManagers.get(robotSide);
               if(handManager != null)
               {
                  handManager.goHome(command.getTrajectoryTime());
               }
            }
         }

         if (command.getRequest(HumanoidBodyPart.PELVIS))
         {
            pelvisOrientationManager.goToHomeFromCurrentDesired(command.getTrajectoryTime());
            balanceManager.goHome();
            comHeightManager.goHome(command.getTrajectoryTime());
         }

         if (command.getRequest(HumanoidBodyPart.CHEST))
         {
            chestManager.goHome(command.getTrajectoryTime());
         }
      }
   }

   public void consumePelvisCommands(WalkingState currentState, boolean allowMotionRegardlessOfState)
   {
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(PelvisOrientationTrajectoryCommand.class))
      {
         PelvisOrientationTrajectoryCommand newestCommand = commandConsumerWithDelayBuffers.pollNewestCommand(PelvisOrientationTrajectoryCommand.class);
         if (allowMotionRegardlessOfState || currentState.isStateSafeToConsumePelvisTrajectoryCommand())
            pelvisOrientationManager.handlePelvisOrientationTrajectoryCommands(newestCommand);
      }

      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(PelvisTrajectoryCommand.class))
      {
         PelvisTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(PelvisTrajectoryCommand.class);
         if (allowMotionRegardlessOfState || currentState.isStateSafeToConsumePelvisTrajectoryCommand())
         {
            if (!pelvisOrientationManager.handlePelvisTrajectoryCommand(command))
               return;
            balanceManager.handlePelvisTrajectoryCommand(command);
            comHeightManager.handlePelvisTrajectoryCommand(command);
         }
      }
   }

   public void consumeManipulationCommands(WalkingState currentState, boolean allowMotionRegardlessOfState)
   {
      if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < manipulationIgnoreInputsDurationAfterAbort.getDoubleValue())
      {
         commandConsumerWithDelayBuffers.clearCommands(HandTrajectoryCommand.class);
         commandConsumerWithDelayBuffers.clearCommands(ArmTrajectoryCommand.class);
         commandConsumerWithDelayBuffers.clearCommands(ArmDesiredAccelerationsCommand.class);
         commandConsumerWithDelayBuffers.clearCommands(HandHybridJointspaceTaskspaceTrajectoryCommand.class);
         return;
      }

      List<HandTrajectoryCommand> handTrajectoryCommands = commandConsumerWithDelayBuffers.pollNewCommands(HandTrajectoryCommand.class);
      List<ArmTrajectoryCommand> armTrajectoryCommands = commandConsumerWithDelayBuffers.pollNewCommands(ArmTrajectoryCommand.class);
      List<ArmDesiredAccelerationsCommand> armDesiredAccelerationCommands = commandConsumerWithDelayBuffers.pollNewCommands(ArmDesiredAccelerationsCommand.class);
      List<HandHybridJointspaceTaskspaceTrajectoryCommand> handHybridCommands = commandConsumerWithDelayBuffers.pollNewCommands(HandHybridJointspaceTaskspaceTrajectoryCommand.class);

      if (allowMotionRegardlessOfState || currentState.isStateSafeToConsumeManipulationCommands())
      {
         for (int i = 0; i < handTrajectoryCommands.size(); i++)
         {
            HandTrajectoryCommand command = handTrajectoryCommands.get(i);
            RobotSide robotSide = command.getRobotSide();
            if (handManagers.get(robotSide) != null)
            {
               SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
               WrenchTrajectoryControllerCommand wrenchTrajectory = command.getWrenchTrajectory();
               se3Trajectory.setSequenceId(command.getSequenceId());
               wrenchTrajectory.setSequenceId(command.getSequenceId());
               handManagers.get(robotSide).handleTaskspaceTrajectoryCommand(se3Trajectory);
               handManagers.get(robotSide).handleWrenchTrajectoryCommand(wrenchTrajectory);
            }
         }

         for (int i = 0; i < armTrajectoryCommands.size(); i++)
         {
            ArmTrajectoryCommand command = armTrajectoryCommands.get(i);
            RobotSide robotSide = command.getRobotSide();
            if (handManagers.get(robotSide) != null)
            {
               JointspaceTrajectoryCommand jointspaceTrajectory = command.getJointspaceTrajectory();
               jointspaceTrajectory.setSequenceId(command.getSequenceId());
               handManagers.get(robotSide).handleJointspaceTrajectoryCommand(jointspaceTrajectory);
            }
         }

         for (int i = 0; i < handHybridCommands.size(); i++)
         {
            HandHybridJointspaceTaskspaceTrajectoryCommand command = handHybridCommands.get(i);
            RobotSide robotSide = command.getRobotSide();
            if (handManagers.get(robotSide) != null)
            {
               SE3TrajectoryControllerCommand taskspaceTrajectoryCommand = command.getTaskspaceTrajectoryCommand();
               JointspaceTrajectoryCommand jointspaceTrajectoryCommand = command.getJointspaceTrajectoryCommand();
               taskspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
               jointspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
               handManagers.get(robotSide).handleHybridTrajectoryCommand(taskspaceTrajectoryCommand, jointspaceTrajectoryCommand);
            }
         }

         for (int i = 0; i < armDesiredAccelerationCommands.size(); i++)
         {
            ArmDesiredAccelerationsCommand command = armDesiredAccelerationCommands.get(i);
            RobotSide robotSide = command.getRobotSide();
            if (handManagers.get(robotSide) != null)
            {
               DesiredAccelerationsCommand desiredAccelerations = command.getDesiredAccelerations();
               desiredAccelerations.setSequenceId(command.getSequenceId());
               handManagers.get(robotSide).handleDesiredAccelerationsCommand(desiredAccelerations);
            }
         }
      }
   }

   public void handleAutomaticManipulationAbortOnICPError(WalkingState currentState)
   {
      if (!currentState.isStateSafeToConsumeManipulationCommands())
         return;

      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(AutomaticManipulationAbortCommand.class))
      {
         AutomaticManipulationAbortCommand message = commandConsumerWithDelayBuffers.pollNewestCommand(AutomaticManipulationAbortCommand.class);
         isAutomaticManipulationAbortEnabled.set(message.isEnable());
      }

      if (!isAutomaticManipulationAbortEnabled.getBooleanValue())
         return;

      if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < minimumDurationBetweenTwoManipulationAborts.getDoubleValue())
         return;

      if (yoTime.getDoubleValue() < allowManipulationAbortAfterThisTime.getDoubleValue())
         return;

      if (balanceManager.getICPErrorMagnitude() > icpErrorThresholdToAbortManipulation.getDoubleValue())
      {
         hasManipulationBeenAborted.set(true);

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyControlManager handManager = handManagers.get(robotSide);
            if (handManager != null && !handManager.isLoadBearing())
            {
               handManager.holdInJointspace();
               handManager.resetJointIntegrators();
            }
         }

         timeOfLastManipulationAbortRequest.set(yoTime.getDoubleValue());

         statusMessageOutputManager.reportStatusMessage(manipulationAbortedStatus);
      }
      else
      {
         hasManipulationBeenAborted.set(false);
      }
   }

   public void consumeFootLoadBearingCommands(WalkingState currentState)
   {
      if (!commandConsumerWithDelayBuffers.isNewCommandAvailable(FootLoadBearingCommand.class))
         return;

      List<FootLoadBearingCommand> footLoadBearingCommands = commandConsumerWithDelayBuffers.pollNewCommands(FootLoadBearingCommand.class);
      for (int i = 0; i < footLoadBearingCommands.size(); i++)
      {
         FootLoadBearingCommand command = footLoadBearingCommands.get(i);
         currentState.handleFootLoadBearingCommand(command);
      }
   }

   public void consumeLoadBearingCommands()
   {
      List<HandLoadBearingCommand> handLoadBearingCommands = commandConsumerWithDelayBuffers.pollNewCommands(HandLoadBearingCommand.class);

      for (int i = 0; i < handLoadBearingCommands.size(); i++)
      {
         HandLoadBearingCommand command = handLoadBearingCommands.get(i);
         RobotSide robotSide = command.getRobotSide();
         if (handManagers.get(robotSide) != null)
         {
            JointspaceTrajectoryCommand jointspaceTrajectory = null;
            if (command.isUseJointspaceCommand())
            {
               jointspaceTrajectory = command.getJointspaceTrajectory();
               jointspaceTrajectory.setSequenceId(command.getSequenceId());
            }

            LoadBearingCommand loadBearingCommand = command.getLoadBearingCommand();
            loadBearingCommand.setSequenceId(command.getSequenceId());
            handManagers.get(robotSide).handleLoadBearingCommand(loadBearingCommand, jointspaceTrajectory);
         }
      }
   }

   public void consumeStopAllTrajectoryCommands()
   {
      if (!commandConsumerWithDelayBuffers.isNewCommandAvailable(StopAllTrajectoryCommand.class))
         return;

      StopAllTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(StopAllTrajectoryCommand.class);
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handManagers.get(robotSide) != null)
            handManagers.get(robotSide).handleStopAllTrajectoryCommand(command);
      }

      if(chestManager != null)
      {
         chestManager.handleStopAllTrajectoryCommand(command);
      }

      feetManager.handleStopAllTrajectoryCommand(command);
      comHeightManager.handleStopAllTrajectoryCommand(command);
      balanceManager.handleStopAllTrajectoryCommand(command);
      pelvisOrientationManager.handleStopAllTrajectoryCommand(command);
   }

   public void consumeFootCommands()
   {
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(FootTrajectoryCommand.class))
      {
         walkingMessageHandler.handleFootTrajectoryCommand(commandConsumerWithDelayBuffers.pollNewCommands(FootTrajectoryCommand.class));
      }

      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(PauseWalkingCommand.class))
      {
         walkingMessageHandler.handlePauseWalkingCommand(commandConsumerWithDelayBuffers.pollNewestCommand(PauseWalkingCommand.class));
      }

      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(FootstepDataListCommand.class))
      {
         walkingMessageHandler.handleFootstepDataListCommand(commandConsumerWithDelayBuffers.pollNewestCommand(FootstepDataListCommand.class));
      }

      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(AdjustFootstepCommand.class))
      {
         walkingMessageHandler.handleAdjustFootstepCommand(commandConsumerWithDelayBuffers.pollNewestCommand(AdjustFootstepCommand.class));
      }

      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(MomentumTrajectoryCommand.class))
      {
         walkingMessageHandler.handleMomentumTrajectoryCommand(commandConsumerWithDelayBuffers.pollNewestCommand(MomentumTrajectoryCommand.class));
      }

      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(CenterOfMassTrajectoryCommand.class))
      {
         walkingMessageHandler.handleComTrajectoryCommand(commandConsumerWithDelayBuffers.pollNewestCommand(CenterOfMassTrajectoryCommand.class));
      }

   }

   public void consumeAbortWalkingCommands(YoBoolean abortWalkingRequested)
   {
      if (!commandConsumerWithDelayBuffers.isNewCommandAvailable(AbortWalkingCommand.class))
         return;
      abortWalkingRequested.set(commandConsumerWithDelayBuffers.pollNewestCommand(AbortWalkingCommand.class).isAbortWalkingRequested());
   }

   public void consumePlanarRegionsListCommand()
   {
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(PlanarRegionsListCommand.class))
      {
         walkingMessageHandler.handlePlanarRegionsListCommand(commandConsumerWithDelayBuffers.pollNewestCommand(PlanarRegionsListCommand.class));
      }
   }
}
