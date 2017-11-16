package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.MultiplePacketHolder;
import us.ihmc.communication.packets.Packet;

public class WholeBodyTrajectoryToolboxMessage extends Packet<WholeBodyTrajectoryToolboxMessage> implements MultiplePacketHolder
{
   public WholeBodyTrajectoryToolboxConfigurationMessage configuration;
   public List<WaypointBasedTrajectoryMessage> endEffectorTrajectories;
   public List<RigidBodyExplorationConfigurationMessage> explorationConfigurations;

   public WholeBodyTrajectoryToolboxMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public WholeBodyTrajectoryToolboxMessage(List<WaypointBasedTrajectoryMessage> endEffectorTrajectories)
   {
      this(null, endEffectorTrajectories, null);
   }

   public WholeBodyTrajectoryToolboxMessage(WholeBodyTrajectoryToolboxConfigurationMessage configuration,
                                            List<WaypointBasedTrajectoryMessage> endEffectorTrajectories)
   {
      this(configuration, endEffectorTrajectories, null);
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public WholeBodyTrajectoryToolboxMessage(WholeBodyTrajectoryToolboxConfigurationMessage configuration,
                                            List<WaypointBasedTrajectoryMessage> endEffectorTrajectories,
                                            List<RigidBodyExplorationConfigurationMessage> explorationConfigurations)
   {
      this.configuration = configuration;
      this.endEffectorTrajectories = endEffectorTrajectories;
      this.explorationConfigurations = explorationConfigurations;
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public void setConfiguration(WholeBodyTrajectoryToolboxConfigurationMessage configuration)
   {
      this.configuration = configuration;
   }

   public void addEndEffectorTrajectory(WaypointBasedTrajectoryMessage endEffectorTrajectory)
   {
      addEndEffectorTrajectories(endEffectorTrajectory);
   }

   public void addEndEffectorTrajectories(WaypointBasedTrajectoryMessage... endEffectorTrajectories)
   {
      if (this.endEffectorTrajectories == null)
         this.endEffectorTrajectories = new ArrayList<>();
      for (WaypointBasedTrajectoryMessage endEffectorTrajectory : endEffectorTrajectories)
      {
         this.endEffectorTrajectories.add(endEffectorTrajectory);
      }
   }

   public void addRigidBodyExplorationConfiguration(RigidBodyExplorationConfigurationMessage explorationConfiguration)
   {
      addRigidBodyExplorationConfigurations(explorationConfiguration);
   }

   public void addRigidBodyExplorationConfigurations(RigidBodyExplorationConfigurationMessage... explorationConfigurations)
   {
      if (this.explorationConfigurations == null)
         this.explorationConfigurations = new ArrayList<>();
      for (RigidBodyExplorationConfigurationMessage explorationConfiguration : explorationConfigurations)
      {
         this.explorationConfigurations.add(explorationConfiguration);
      }
   }

   public void setEndEffectorTrajectories(List<WaypointBasedTrajectoryMessage> endEffectorTrajectories)
   {
      this.endEffectorTrajectories = endEffectorTrajectories;
   }

   public WholeBodyTrajectoryToolboxConfigurationMessage getConfiguration()
   {
      return configuration;
   }

   public List<WaypointBasedTrajectoryMessage> getEndEffectorTrajectories()
   {
      return endEffectorTrajectories;
   }

   public List<RigidBodyExplorationConfigurationMessage> getExplorationConfigurations()
   {
      return explorationConfigurations;
   }

   @Override
   public List<Packet<?>> getPackets()
   {
      List<Packet<?>> allPackets = new ArrayList<>();

      allPackets.addAll(endEffectorTrajectories);
      allPackets.addAll(explorationConfigurations);
      allPackets.add(configuration);

      return allPackets;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxMessage other, double epsilon)
   {
      if (configuration == null ^ other.configuration == null)
         return false;
      if (configuration != null && !configuration.epsilonEquals(other.configuration, epsilon))
         return false;
      if (endEffectorTrajectories.size() != other.endEffectorTrajectories.size())
         return false;
      for (int i = 0; i < endEffectorTrajectories.size(); i++)
      {
         if (!endEffectorTrajectories.get(i).epsilonEquals(other.endEffectorTrajectories.get(i), epsilon))
            return false;
      }
      if (explorationConfigurations.size() != other.explorationConfigurations.size())
         return false;
      for (int i = 0; i < explorationConfigurations.size(); i++)
      {
         if (!explorationConfigurations.get(i).epsilonEquals(other.explorationConfigurations.get(i), epsilon))
            return false;
      }
      return true;
   }
}
