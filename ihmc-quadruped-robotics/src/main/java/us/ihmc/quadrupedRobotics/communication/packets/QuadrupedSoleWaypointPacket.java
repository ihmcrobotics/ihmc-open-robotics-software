package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;

public class QuadrupedSoleWaypointPacket extends Packet<QuadrupedSoleWaypointPacket>
{
   private QuadrupedSoleWaypointList quadrupedSoleWaypointList;

   public QuadrupedSoleWaypointPacket()
   {
      this.quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();
   }

   public QuadrupedSoleWaypointPacket(QuadrupedSoleWaypointList quadrupedSoleWaypointList)
   {
      this.quadrupedSoleWaypointList = new QuadrupedSoleWaypointList(quadrupedSoleWaypointList);
   }

   @Override
   public boolean epsilonEquals(QuadrupedSoleWaypointPacket other, double epsilon)
   {
      return quadrupedSoleWaypointList.epsilonEquals(other.quadrupedSoleWaypointList, epsilon);
   }

   public QuadrupedSoleWaypointList get()
   {
      return quadrupedSoleWaypointList;
   }
}
