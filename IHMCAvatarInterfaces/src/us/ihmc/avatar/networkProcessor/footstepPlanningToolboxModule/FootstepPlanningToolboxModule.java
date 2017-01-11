package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.TimeTools;

public class FootstepPlanningToolboxModule extends ToolboxModule
{
   private static final PacketDestination PACKET_DESTINATION = PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE;
   private static final NetworkPorts NETWORK_PORT = NetworkPorts.FOOTSTEP_PLANNING_TOOLBOX_MODULE_PORT;

   private final FootstepPlanningToolboxController footstepPlanningToolboxController;

   public FootstepPlanningToolboxModule(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullHumanoidRobotModel, LogModelProvider modelProvider,
                                        boolean startYoVariableServer)
         throws IOException
   {
      super(fullHumanoidRobotModel, modelProvider, startYoVariableServer, PACKET_DESTINATION, NETWORK_PORT);
      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);
      footstepPlanningToolboxController = new FootstepPlanningToolboxController(drcRobotModel, fullHumanoidRobotModel, statusOutputManager, packetCommunicator,
                                                                                registry, TimeTools.milliSecondsToSeconds(DEFAULT_UPDATE_PERIOD_MILLISECONDS));
      packetCommunicator.attachListener(FootstepPlanningRequestPacket.class, footstepPlanningToolboxController.createRequestConsumer());
      startYoVariableServer();
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return footstepPlanningToolboxController;
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
