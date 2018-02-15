package us.ihmc.communication.packets;

import java.util.Arrays;

import us.ihmc.euclid.tuple3D.Point3D;

public class DetectedFacesPacket extends Packet<DetectedFacesPacket>
{
   public String[] ids;
   public Point3D[] positions;

   public DetectedFacesPacket()
   {
   }

   @Override
   public void set(DetectedFacesPacket other)
   {
      ids = Arrays.copyOf(other.ids, other.ids.length);
      positions = Arrays.stream(other.positions).map(Point3D::new).toArray(Point3D[]::new);
      setPacketInformation(other);
   }

   public String[] getIds()
   {
      return ids;
   }

   public Point3D[] getPositions()
   {
      return positions;
   }

   @Override
   public boolean epsilonEquals(DetectedFacesPacket other, double epsilon)
   {
      boolean ret = true;

      for (int i = 0; i < ids.length; i++)
      {
         ret &= ids[i].equals(other.getIds()[i]) && positions[i].epsilonEquals(other.getPositions()[i], epsilon);
      }

      return ret;
   }
}
