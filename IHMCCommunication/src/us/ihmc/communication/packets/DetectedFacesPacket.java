package us.ihmc.communication.packets;

import javax.vecmath.Point3d;

public class DetectedFacesPacket extends Packet<DetectedFacesPacket>
{
   public String[] ids;
   public Point3d[] positions;

   public DetectedFacesPacket()
   {

   }

   public DetectedFacesPacket(String[] ids, Point3d[] positions)
   {
      this.ids = ids;
      this.positions = positions;
   }

   public String[] getIds()
   {
      return ids;
   }

   public Point3d[] getPositions()
   {
      return positions;
   }

   @Override public boolean epsilonEquals(DetectedFacesPacket other, double epsilon)
   {
      boolean ret = true;

      for(int i = 0; i < ids.length; i++)
      {
         ret &= ids[i].equals(other.getIds()[i]) && positions[i].epsilonEquals(other.getPositions()[i], epsilon);
      }

      return ret;
   }
}
