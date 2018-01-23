package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.controllerAPI.command.MultipleCommandHolder;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class WholeBodyTrajectoryToolboxCommand implements MultipleCommandHolder<WholeBodyTrajectoryToolboxCommand, WholeBodyTrajectoryToolboxMessage>, WholeBodyTrajectoryToolboxAPI<WholeBodyTrajectoryToolboxMessage>
{
   private boolean hasConfiguration = false;
   private final WholeBodyTrajectoryToolboxConfigurationCommand configuration = new WholeBodyTrajectoryToolboxConfigurationCommand();
   private final RecyclingArrayList<WaypointBasedTrajectoryCommand> endEffectorTrajectories = new RecyclingArrayList<>(WaypointBasedTrajectoryCommand.class);
   private final RecyclingArrayList<RigidBodyExplorationConfigurationCommand> rigidBodyExplorationConfigurations = new RecyclingArrayList<>(RigidBodyExplorationConfigurationCommand.class);
   private final List<Command<?, ?>> commands = new ArrayList<>();
   
   @Override
   public void clear()
   {
      hasConfiguration = false;
      configuration.clear();
      endEffectorTrajectories.clear();
      rigidBodyExplorationConfigurations.clear();
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
      
      for (int i = 0; i < other.rigidBodyExplorationConfigurations.size(); i++)
      {
         RigidBodyExplorationConfigurationCommand rigidBodyExplorationConfiguration = rigidBodyExplorationConfigurations.add();
         rigidBodyExplorationConfiguration.set(other.rigidBodyExplorationConfigurations.get(i));
         commands.add(rigidBodyExplorationConfiguration);
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
      
      for (int i = 0; i < message.getExplorationConfigurations().size(); i++)
      {
         RigidBodyExplorationConfigurationCommand rigidBodyExplorationConfiguration = rigidBodyExplorationConfigurations.add();
         rigidBodyExplorationConfiguration.set(message.getExplorationConfigurations().get(i));
         commands.add(rigidBodyExplorationConfiguration);
      }
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxMessage message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
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
         endEffectorTrajectory.set(message.getEndEffectorTrajectories().get(i), rigidBodyNamedBasedHashMap, referenceFrameResolver);
         commands.add(endEffectorTrajectory);
      }
      
      for (int i = 0; i < message.getExplorationConfigurations().size(); i++)
      {
         RigidBodyExplorationConfigurationCommand rigidBodyExplorationConfiguration = rigidBodyExplorationConfigurations.add();
         rigidBodyExplorationConfiguration.set(message.getExplorationConfigurations().get(i), rigidBodyNamedBasedHashMap, referenceFrameResolver);
         commands.add(rigidBodyExplorationConfiguration);
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

   public boolean hasConfiguration()
   {
      return hasConfiguration;
   }
   
   public WholeBodyTrajectoryToolboxConfigurationCommand getConfigurationCommand()
   {
      return configuration;
   }
   
   public RecyclingArrayList<WaypointBasedTrajectoryCommand> getEndEffectorTrajectories()
   {
      return endEffectorTrajectories;
   }
   
   public RecyclingArrayList<RigidBodyExplorationConfigurationCommand> getRigidBodyExplorationConfigurations()
   {
      return rigidBodyExplorationConfigurations;
   }

}
