package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.packets.Packet;

public class WholeBodyTrajectoryToolboxMessage extends Packet<WholeBodyTrajectoryToolboxMessage>
{
   public WholeBodyTrajectoryToolboxConfigurationMessage configuration;
   public List<WaypointBasedTrajectoryMessage> endEffectorTrajectories;
   public List<RigidBodyExplorationConfigurationMessage> explorationConfigurations;
   public List<ReachingManifoldMessage> reachingManifolds;

   public WholeBodyTrajectoryToolboxMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public WholeBodyTrajectoryToolboxMessage(WholeBodyTrajectoryToolboxConfigurationMessage configuration,
                                            List<WaypointBasedTrajectoryMessage> endEffectorTrajectories, List<ReachingManifoldMessage> reachingManifolds,
                                            List<RigidBodyExplorationConfigurationMessage> explorationConfigurations)
   {
      this.configuration = configuration;
      this.endEffectorTrajectories = endEffectorTrajectories;
      this.reachingManifolds = reachingManifolds;
      this.explorationConfigurations = explorationConfigurations;
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxMessage other)
   {
      configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.set(other.configuration);
      endEffectorTrajectories = new ArrayList<>();
      for (WaypointBasedTrajectoryMessage otherTrajectory : other.endEffectorTrajectories)
      {
         WaypointBasedTrajectoryMessage trajectory = new WaypointBasedTrajectoryMessage();
         trajectory.set(otherTrajectory);
         endEffectorTrajectories.add(trajectory);
      }

      explorationConfigurations = new ArrayList<>();
      for (RigidBodyExplorationConfigurationMessage otherConfiguration : other.explorationConfigurations)
      {
         RigidBodyExplorationConfigurationMessage configuration = new RigidBodyExplorationConfigurationMessage();
         configuration.set(otherConfiguration);
         explorationConfigurations.add(configuration);
      }
      
      reachingManifolds = new ArrayList<>();
      for (ReachingManifoldMessage otherManifold : other.reachingManifolds)
      {
         ReachingManifoldMessage manifold = new ReachingManifoldMessage();
         manifold.set(otherManifold);
         reachingManifolds.add(manifold);
      }
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

   public void addReachingManifold(ReachingManifoldMessage reachingManifold)
   {
      addReachingManifolds(reachingManifold);
   }

   public void addReachingManifolds(ReachingManifoldMessage... reachingManifolds)
   {
      if (this.reachingManifolds == null)
         this.reachingManifolds = new ArrayList<>();
      for (ReachingManifoldMessage explorationConfiguration : reachingManifolds)
      {
         this.reachingManifolds.add(explorationConfiguration);
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

   public List<ReachingManifoldMessage> getReachingManifolds()
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
      if (reachingManifolds.size() != other.reachingManifolds.size())
         return false;
      for (int i = 0; i < reachingManifolds.size(); i++)
      {
         if (!reachingManifolds.get(i).epsilonEquals(other.reachingManifolds.get(i), epsilon))
            return false;
      }
      return true;
   }
}
