package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class WalkOverTerrainGoalPacket extends Packet<WalkOverTerrainGoalPacket>
{
   public Point3D position = new Point3D();
   public Quaternion orientation = new Quaternion();

   public WalkOverTerrainGoalPacket()
   {
   }

   @Override
   public void set(WalkOverTerrainGoalPacket other)
   {
      position.set(other.position);
      orientation.set(other.orientation);
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(WalkOverTerrainGoalPacket other, double epsilon)
   {
      if(other == null)
         return false;

      return other.position.epsilonEquals(position, epsilon) && other.orientation.epsilonEquals(orientation, epsilon);
   }
}
