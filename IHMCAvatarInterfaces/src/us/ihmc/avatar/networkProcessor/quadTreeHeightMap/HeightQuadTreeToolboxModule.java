package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.HeightQuadTreeToolboxRequestCommand;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.LidarScanCommand;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class HeightQuadTreeToolboxModule extends ToolboxModule
{
   private static final PacketDestination PACKET_DESTINATION = PacketDestination.HEIGHT_QUADTREE_TOOLBOX_MODULE;
   private static final NetworkPorts NETWORK_PORT = NetworkPorts.HEIGHT_QUADTREE_TOOLBOX_MODULE_PORT;

   private final HeightQuadTreeToolboxController controller;

   public HeightQuadTreeToolboxModule(FullHumanoidRobotModel desiredFullRobotModel, LogModelProvider modelProvider) throws IOException
   {
      super(desiredFullRobotModel, modelProvider, false, PACKET_DESTINATION, NETWORK_PORT, 50);

      controller = new HeightQuadTreeToolboxController(fullRobotModel, packetCommunicator, commandInputManager, statusOutputManager, registry);
      setTimeWithoutInputsBeforeGoingToSleep(3.0);
      packetCommunicator.attachListener(RobotConfigurationData.class, controller.robotConfigurationDataConsumer());
      packetCommunicator.attachListener(CapturabilityBasedStatus.class, controller.capturabilityBasedStatusConsumer());
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return controller;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(HeightQuadTreeToolboxRequestCommand.class);
      commands.add(LidarScanCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends StatusPacket<?>>> createListOfSupportedStatus()
   {
      return Collections.singletonList(HeightQuadTreeMessage.class);
   }

   @Override
   public Set<Class<? extends Command<?, ?>>> silentCommands()
   {
      return Collections.singleton(LidarScanCommand.class);
   }

   @Override
   public Set<Class<? extends Packet<?>>> filterExceptions()
   {
      return Collections.singleton(LidarScanMessage.class);
   }
}
