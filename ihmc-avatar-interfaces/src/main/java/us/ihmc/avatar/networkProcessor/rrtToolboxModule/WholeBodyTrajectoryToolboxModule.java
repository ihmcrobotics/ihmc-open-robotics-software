package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class WholeBodyTrajectoryToolboxModule extends ToolboxModule
{
   private static final PacketDestination PACKET_DESTINATION = PacketDestination.WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE;
   private static final NetworkPorts NETWORK_PORT = NetworkPorts.WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_PORT;

   private final WholeBodyTrajectoryToolboxController wholeBodyTrajectoryToolboxController;

   public WholeBodyTrajectoryToolboxModule(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullHumanoidRobotModel, LogModelProvider modelProvider,
                                                    boolean startYoVariableServer)
         throws IOException
   {
      super(drcRobotModel.createFullRobotModel(), drcRobotModel.getLogModelProvider(), startYoVariableServer, PACKET_DESTINATION, NETWORK_PORT);

      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);

      wholeBodyTrajectoryToolboxController = new WholeBodyTrajectoryToolboxController(drcRobotModel, fullRobotModel, commandInputManager,
                                                                                                        statusOutputManager, registry, yoGraphicsListRegistry,
                                                                                                        startYoVariableServer);
      commandInputManager.registerConversionHelper(new WholeBodyTrajectoryToolboxCommandConverter(fullRobotModel));
      startYoVariableServer();
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return wholeBodyTrajectoryToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(WholeBodyTrajectoryToolboxCommand.class);
      commands.add(WaypointBasedTrajectoryCommand.class);
      commands.add(WholeBodyTrajectoryToolboxConfigurationCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends SettablePacket<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends SettablePacket<?>>> status = new ArrayList<>();
      status.add(WholeBodyTrajectoryToolboxOutputStatus.class);
      return status;
   }

   @Override
   public Set<Class<? extends Command<?, ?>>> silentCommands()
   {
      Set<Class<? extends Command<?, ?>>> commands = new HashSet<>();
      commands.add(WaypointBasedTrajectoryCommand.class);
      commands.add(WholeBodyTrajectoryToolboxConfigurationCommand.class);
      return commands;
   }
}