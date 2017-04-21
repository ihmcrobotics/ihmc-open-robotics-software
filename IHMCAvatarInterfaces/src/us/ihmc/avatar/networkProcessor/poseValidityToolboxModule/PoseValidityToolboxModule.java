package us.ihmc.avatar.networkProcessor.poseValidityToolboxModule;

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
import us.ihmc.humanoidRobotics.communication.packets.behaviors.PoseValidityRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;

public class PoseValidityToolboxModule extends ToolboxModule
{
   private static final PacketDestination PACKET_DESTINATION = PacketDestination.POSEVALIDITY_TOOLBOX_MODULE;
   private static final NetworkPorts NETWORK_PORT = NetworkPorts.POSEVALIDITY_TOOLBOX_MODULE_PORT;

   private final PoseValidityToolboxController toolboxController;
   
   public PoseValidityToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer)
         throws IOException
   {      
      super(robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, PACKET_DESTINATION, NETWORK_PORT);
            
      toolboxController = new PoseValidityToolboxController(statusOutputManager, registry);
      
      packetCommunicator.attachListener(PoseValidityRequestPacket.class, toolboxController.createRequestConsumer());
      
      startYoVariableServer();
      
      PrintTools.info("PoseValdityToolboxModule Initiated");
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return toolboxController;
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
      status.add(FootstepPlanningToolboxOutputStatus.class);
      return status;
   }

}
