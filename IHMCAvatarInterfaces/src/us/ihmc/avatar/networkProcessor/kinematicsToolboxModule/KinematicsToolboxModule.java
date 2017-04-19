package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class KinematicsToolboxModule extends ToolboxModule
{
   private static final PacketDestination PACKET_DESTINATION = PacketDestination.KINEMATICS_TOOLBOX_MODULE;
   private static final NetworkPorts NETWORK_PORT = NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT;

   private final KinematicsToolboxController kinematicsToolBoxController;

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer) throws IOException
   {
      super(robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, PACKET_DESTINATION, NETWORK_PORT);
      kinematicsToolBoxController = new KinematicsToolboxController(commandInputManager, statusOutputManager, fullRobotModel, yoGraphicsListRegistry, registry);
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(fullRobotModel.getElevator()));
      packetCommunicator.attachListener(RobotConfigurationData.class, kinematicsToolBoxController::updateRobotConfigurationData);
      packetCommunicator.attachListener(CapturabilityBasedStatus.class, kinematicsToolBoxController::updateCapturabilityBasedStatus);
      startYoVariableServer();
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(KinematicsToolboxCenterOfMassCommand.class);
      commands.add(KinematicsToolboxRigidBodyCommand.class);
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
