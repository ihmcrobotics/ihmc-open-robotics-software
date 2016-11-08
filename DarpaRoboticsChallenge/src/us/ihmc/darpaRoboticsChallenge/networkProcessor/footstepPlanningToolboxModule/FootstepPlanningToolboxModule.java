package us.ihmc.darpaRoboticsChallenge.networkProcessor.footstepPlanningToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.FootstepPlanningToolboxOutputStatus;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.ToolboxController;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.ToolboxModule;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class FootstepPlanningToolboxModule extends ToolboxModule
{
   private final FootstepPlanningToolboxController footstepPlanningToolboxController;

   public FootstepPlanningToolboxModule(FullHumanoidRobotModel desiredFullRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer)
         throws IOException
   {
      super(desiredFullRobotModel, modelProvider, startYoVariableServer);
      footstepPlanningToolboxController = new FootstepPlanningToolboxController(statusOutputManager);
      packetCommunicator.attachListener(FootstepPlanningRequestPacket.class, footstepPlanningToolboxController.createRequestConsumer());
   }

   @Override
   public ToolboxController<? extends StatusPacket<?>> getToolboxController()
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
