package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.RRTPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.RRTPlanningToolboxOutputStatus;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class RRTPlanningToolboxModule extends ToolboxModule
{
   private static final PacketDestination PACKET_DESTINATION = PacketDestination.RRTPLANNING_TOOLBOX_MODULE;
   private static final NetworkPorts NETWORK_PORT = NetworkPorts.RRTPLANNING_TOOLBOX_MODULE_PORT;

   private final RRTPlanningToolboxController rrtToolboxController;
   
   public RRTPlanningToolboxModule(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullHumanoidRobotModel, LogModelProvider modelProvider,
                           boolean startYoVariableServer)
         throws IOException
   {
      super(drcRobotModel.createFullRobotModel(), drcRobotModel.getLogModelProvider(), startYoVariableServer, PACKET_DESTINATION, NETWORK_PORT);
      
      PrintTools.info("bb");
      
      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);
      
      PrintTools.info("bb");
      rrtToolboxController = new RRTPlanningToolboxController(statusOutputManager, registry);
      PrintTools.info("bb");
      
      packetCommunicator.attachListener(RRTPlanningRequestPacket.class, rrtToolboxController.createRequestConsumer());
      PrintTools.info("bb");
      startYoVariableServer();
      PrintTools.info("bb");
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return rrtToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      return commands;
   }

   @Override
   public List<Class<? extends StatusPacket<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends StatusPacket<?>>> status = new ArrayList<>();
      status.add(RRTPlanningToolboxOutputStatus.class);
      return status;
   }

}
