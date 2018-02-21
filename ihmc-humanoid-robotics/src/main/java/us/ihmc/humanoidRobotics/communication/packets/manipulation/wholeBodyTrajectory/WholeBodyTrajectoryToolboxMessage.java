package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.RecyclingArrayListPubSub;

public class WholeBodyTrajectoryToolboxMessage extends Packet<WholeBodyTrajectoryToolboxMessage>
{
   public WholeBodyTrajectoryToolboxConfigurationMessage configuration;
   public RecyclingArrayListPubSub<WaypointBasedTrajectoryMessage> endEffectorTrajectories = new RecyclingArrayListPubSub<>(WaypointBasedTrajectoryMessage.class, WaypointBasedTrajectoryMessage::new, 10);
   public RecyclingArrayListPubSub<RigidBodyExplorationConfigurationMessage> explorationConfigurations = new RecyclingArrayListPubSub<>(RigidBodyExplorationConfigurationMessage.class, RigidBodyExplorationConfigurationMessage::new, 10);
   public RecyclingArrayListPubSub<ReachingManifoldMessage> reachingManifolds = new RecyclingArrayListPubSub<>(ReachingManifoldMessage.class, ReachingManifoldMessage::new, 10);

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

   public WholeBodyTrajectoryToolboxConfigurationMessage getConfiguration()
   {
      return configuration;
   }

   public RecyclingArrayListPubSub<WaypointBasedTrajectoryMessage> getEndEffectorTrajectories()
   {
      return endEffectorTrajectories;
   }

   public RecyclingArrayListPubSub<RigidBodyExplorationConfigurationMessage> getExplorationConfigurations()
   {
      return explorationConfigurations;
   }

   public RecyclingArrayListPubSub<ReachingManifoldMessage> getReachingManifolds()
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
