package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.List;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.PreallocatedList;

public class WholeBodyTrajectoryToolboxMessage extends Packet<WholeBodyTrajectoryToolboxMessage>
{
   public WholeBodyTrajectoryToolboxConfigurationMessage configuration;
   public PreallocatedList<WaypointBasedTrajectoryMessage> endEffectorTrajectories = new PreallocatedList<>(WaypointBasedTrajectoryMessage.class, WaypointBasedTrajectoryMessage::new, 10);
   public PreallocatedList<RigidBodyExplorationConfigurationMessage> explorationConfigurations = new PreallocatedList<>(RigidBodyExplorationConfigurationMessage.class, RigidBodyExplorationConfigurationMessage::new, 10);
   public PreallocatedList<ReachingManifoldMessage> reachingManifolds = new PreallocatedList<>(ReachingManifoldMessage.class, ReachingManifoldMessage::new, 10);

   public WholeBodyTrajectoryToolboxMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxMessage other)
   {
      configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.set(other.configuration);
      MessageTools.copyData(other.endEffectorTrajectories, endEffectorTrajectories);
      MessageTools.copyData(other.explorationConfigurations, explorationConfigurations);
      MessageTools.copyData(other.reachingManifolds, reachingManifolds);
      setPacketInformation(other);
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
      MessageTools.copyData(endEffectorTrajectories, this.endEffectorTrajectories);
   }

   public void addRigidBodyExplorationConfiguration(RigidBodyExplorationConfigurationMessage explorationConfiguration)
   {
      addRigidBodyExplorationConfigurations(explorationConfiguration);
   }

   public void addRigidBodyExplorationConfigurations(RigidBodyExplorationConfigurationMessage... explorationConfigurations)
   {
      MessageTools.copyData(explorationConfigurations, this.explorationConfigurations);
   }

   public void addReachingManifold(ReachingManifoldMessage reachingManifold)
   {
      addReachingManifolds(reachingManifold);
   }

   public void addReachingManifolds(ReachingManifoldMessage... reachingManifolds)
   {
      MessageTools.copyData(reachingManifolds, this.reachingManifolds);
   }

   public void setEndEffectorTrajectories(List<WaypointBasedTrajectoryMessage> endEffectorTrajectories)
   {
      MessageTools.copyData(endEffectorTrajectories, this.endEffectorTrajectories);
   }

   public WholeBodyTrajectoryToolboxConfigurationMessage getConfiguration()
   {
      return configuration;
   }

   public PreallocatedList<WaypointBasedTrajectoryMessage> getEndEffectorTrajectories()
   {
      return endEffectorTrajectories;
   }

   public PreallocatedList<RigidBodyExplorationConfigurationMessage> getExplorationConfigurations()
   {
      return explorationConfigurations;
   }

   public PreallocatedList<ReachingManifoldMessage> getReachingManifolds()
   {
      return reachingManifolds;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxMessage other, double epsilon)
   {
      if (configuration == null ^ other.configuration == null)
         return false;
      if (configuration != null && !configuration.epsilonEquals(other.configuration, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(endEffectorTrajectories, other.endEffectorTrajectories, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(explorationConfigurations, other.explorationConfigurations, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(reachingManifolds, other.reachingManifolds, epsilon))
         return false;
      if (explorationConfigurations.size() != other.explorationConfigurations.size())
         return false;
      return true;
   }
}
