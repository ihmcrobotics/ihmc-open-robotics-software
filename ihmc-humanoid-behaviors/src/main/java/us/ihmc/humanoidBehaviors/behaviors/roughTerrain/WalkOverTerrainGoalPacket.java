package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class WalkOverTerrainGoalPacket extends Packet<WalkOverTerrainGoalPacket>
{
   public Point3D position;
   public Quaternion orientation;

   public WalkOverTerrainGoalPacket()
   {
   }

   public WalkOverTerrainGoalPacket(Point3D position, Quaternion orientation)
   {
      this.position = position;
      this.orientation = orientation;
   }

   @Override
   public boolean epsilonEquals(WalkOverTerrainGoalPacket other, double epsilon)
   {
      if(other == null)
         return false;

      return other.position.epsilonEquals(position, epsilon) && other.orientation.epsilonEquals(orientation, epsilon);
   }
}
