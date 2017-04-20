package us.ihmc.avatar.networkProcessor.poseValidityToolboxModule;

import java.io.IOException;
import java.util.List;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class PoseValidityToolboxModule extends ToolboxModule
{
   private static final PacketDestination PACKET_DESTINATION = PacketDestination.POSEVALIDITY_TOOLBOX_MODULE;
   private static final NetworkPorts NETWORK_PORT = NetworkPorts.POSEVALIDITY_TOOLBOX_MODULE_PORT;

   private final PoseValidityToolboxController toolboxController;
   
   public PoseValidityToolboxModule(FullHumanoidRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
                                    PacketDestination toolboxDestination, NetworkPorts toolboxPort)
         throws IOException
   {
      super(fullRobotModelToLog, modelProvider, startYoVariableServer, toolboxDestination, toolboxPort);
      
      toolboxController = new PoseValidityToolboxController(statusOutputManager, registry);
      // packetCommunicator.attachListener(RobotConfigurationData.class, toolboxController.createRobotConfigurationDataConsumer());
      startYoVariableServer();
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return toolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public List<Class<? extends StatusPacket<?>>> createListOfSupportedStatus()
   {
      // TODO Auto-generated method stub
      return null;
   }

}
