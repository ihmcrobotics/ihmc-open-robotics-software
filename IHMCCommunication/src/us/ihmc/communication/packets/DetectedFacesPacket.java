package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D;

public class DetectedFacesPacket extends Packet<DetectedFacesPacket>
{
   public String[] ids;
   public Point3D[] positions;

   public DetectedFacesPacket()
   {

   }

   public DetectedFacesPacket(String[] ids, Point3D[] positions)
   {
      this.ids = ids;
      this.positions = positions;
   }

   public String[] getIds()
   {
      return ids;
   }

   public Point3D[] getPositions()
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
