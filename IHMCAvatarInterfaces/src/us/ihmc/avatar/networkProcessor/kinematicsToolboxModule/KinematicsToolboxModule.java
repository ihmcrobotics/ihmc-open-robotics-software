package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TrackingWeightsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WholeBodyTrajectoryCommand;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class KinematicsToolboxModule extends ToolboxModule
{
   private static final PacketDestination PACKET_DESTINATION = PacketDestination.KINEMATICS_TOOLBOX_MODULE;
   private static final NetworkPorts NETWORK_PORT = NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT;

   private final KinematicsToolboxController kinematicsToolBoxController;

   public KinematicsToolboxModule(FullHumanoidRobotModel desiredFullRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer) throws IOException
   {
      super(desiredFullRobotModel, modelProvider, startYoVariableServer, PACKET_DESTINATION, NETWORK_PORT);
      kinematicsToolBoxController = new KinematicsToolboxController(commandInputManager, statusOutputManager, desiredFullRobotModel, yoGraphicsListRegistry, registry);
      packetCommunicator.attachListener(RobotConfigurationData.class, kinematicsToolBoxController.createRobotConfigurationDataConsumer());
      startYoVariableServer();
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(HandTrajectoryCommand.class);
      commands.add(ChestTrajectoryCommand.class);
      commands.add(PelvisOrientationTrajectoryCommand.class);
      commands.add(WholeBodyTrajectoryCommand.class);
      commands.add(TrackingWeightsCommand.class);
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
   public ToolboxController getToolboxController()
   {
      return kinematicsToolBoxController;
   }
}
