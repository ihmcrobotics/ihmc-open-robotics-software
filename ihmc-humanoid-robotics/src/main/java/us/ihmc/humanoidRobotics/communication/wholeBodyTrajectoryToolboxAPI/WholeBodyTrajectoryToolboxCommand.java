package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.controllerAPI.command.MultipleCommandHolder;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class WholeBodyTrajectoryToolboxCommand implements MultipleCommandHolder<WholeBodyTrajectoryToolboxCommand, WholeBodyTrajectoryToolboxMessage>
{
   private boolean hasConfiguration = false;
   private final WholeBodyTrajectoryToolboxConfigurationCommand configuration = new WholeBodyTrajectoryToolboxConfigurationCommand();
   private final RecyclingArrayList<WaypointBasedTrajectoryCommand> endEffectorTrajectories = new RecyclingArrayList<>(WaypointBasedTrajectoryCommand.class);
   private final List<Command<?, ?>> commands = new ArrayList<>();

   @Override
   public void clear()
   {
      hasConfiguration = false;
      configuration.clear();
      endEffectorTrajectories.clear();
      commands.clear();
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxCommand other)
   {
      clear();
      hasConfiguration = other.hasConfiguration;
      if (hasConfiguration)
      {
         configuration.set(other.configuration);
         commands.add(configuration);
      }

      for (int i = 0; i < other.endEffectorTrajectories.size(); i++)
      {
         WaypointBasedTrajectoryCommand endEffectorTrajectory = endEffectorTrajectories.add();
         endEffectorTrajectory.set(other.endEffectorTrajectories.get(i));
         commands.add(endEffectorTrajectory);
      }
      
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxMessage message)
   {
      clear();

      hasConfiguration = message.getConfiguration() != null;
      if (hasConfiguration)
      {
         configuration.set(message.getConfiguration());
         commands.add(configuration);
      }

      for (int i = 0; i < message.getEndEffectorTrajectories().size(); i++)
      {
         WaypointBasedTrajectoryCommand endEffectorTrajectory = endEffectorTrajectories.add();
         endEffectorTrajectory.set(message.getEndEffectorTrajectories().get(i));
         commands.add(endEffectorTrajectory);
      }
   }

   @Override
   public void parseCommandIdToChildren()
   {
   }

   @Override
   public List<Command<?, ?>> getControllerCommands()
   {
      return commands;
   }

   @Override
   public Class<WholeBodyTrajectoryToolboxMessage> getMessageClass()
   {
      return WholeBodyTrajectoryToolboxMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return endEffectorTrajectories.size() > 0;
   }
}
