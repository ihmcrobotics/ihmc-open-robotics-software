package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedSoleWaypointPacket extends Packet<QuadrupedSoleWaypointPacket>
{
   private final QuadrantDependentList<QuadrupedSoleWaypointList> quadrupedSoleWaypointLists = new QuadrantDependentList<>();

   public QuadrupedSoleWaypointPacket()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         this.quadrupedSoleWaypointLists.set(robotQuadrant, new QuadrupedSoleWaypointList());
   }

   public QuadrupedSoleWaypointPacket(QuadrantDependentList<QuadrupedSoleWaypointList> quadrupedSoleWaypointLists)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         this.quadrupedSoleWaypointLists.set(robotQuadrant, new QuadrupedSoleWaypointList(quadrupedSoleWaypointLists.get(robotQuadrant)));
   }

   @Override
   public boolean epsilonEquals(QuadrupedSoleWaypointPacket other, double epsilon)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if (!quadrupedSoleWaypointLists.get(quadrant).epsilonEquals(other.quadrupedSoleWaypointLists.get(quadrant), epsilon))
            return false;
      }
      return true;
   }

   public QuadrantDependentList<QuadrupedSoleWaypointList> get()
   {
      return quadrupedSoleWaypointLists;
   }
}
