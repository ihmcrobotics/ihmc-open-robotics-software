package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.packets.MultiplePacketHolder;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.TrackablePacket;

public class WholeBodyTrajectoryToolboxMessage extends TrackablePacket<WholeBodyTrajectoryToolboxMessage> implements MultiplePacketHolder
{
   public WholeBodyTrajectoryToolboxConfigurationMessage configuration;
   public List<WaypointBasedTrajectoryMessage> endEffectorTrajectories;

   public WholeBodyTrajectoryToolboxMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public WholeBodyTrajectoryToolboxMessage(List<WaypointBasedTrajectoryMessage> endEffectorTrajectories)
   {
      this(null, endEffectorTrajectories);
   }

   public WholeBodyTrajectoryToolboxMessage(WholeBodyTrajectoryToolboxConfigurationMessage configuration,
                                            List<WaypointBasedTrajectoryMessage> endEffectorTrajectories)
   {
      this.configuration = configuration;
      this.endEffectorTrajectories = endEffectorTrajectories;
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

   @Override
   public List<Packet<?>> getPackets()
   {
      List<Packet<?>> allPackets = new ArrayList<>();

      allPackets.addAll(endEffectorTrajectories);
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
      return true;
   }
}
