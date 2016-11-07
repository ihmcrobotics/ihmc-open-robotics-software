package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.ToolboxController;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.ToolboxModule;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WholeBodyTrajectoryCommand;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class KinematicsToolboxModule extends ToolboxModule
{
   private final KinematicsToolboxController kinematicsToolBoxController;

   public KinematicsToolboxModule(FullHumanoidRobotModel desiredFullRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer) throws IOException
   {
      super(desiredFullRobotModel, modelProvider, startYoVariableServer);
      kinematicsToolBoxController = new KinematicsToolboxController(commandInputManager, statusOutputManager, desiredFullRobotModel, yoGraphicsListRegistry, registry);
      packetCommunicator.attachListener(RobotConfigurationData.class, kinematicsToolBoxController.createRobotConfigurationDataConsumer());
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(ArmTrajectoryCommand.class);
      commands.add(HandTrajectoryCommand.class);
      commands.add(FootTrajectoryCommand.class);
      commands.add(ChestTrajectoryCommand.class);
      commands.add(PelvisTrajectoryCommand.class);
      commands.add(WholeBodyTrajectoryCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends StatusPacket<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends StatusPacket<?>>> status = new ArrayList<>();
      status.add(KinematicsToolboxOutputStatus.class);
      return status;
   }

   @Override
   public ToolboxController<KinematicsToolboxOutputStatus> getToolboxController()
   {
      return kinematicsToolBoxController;
   }
}
