package us.ihmc.ihmcPerception.faceDetection;

import javax.vecmath.Point3d;

public class DetectedFacesPacket
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
}
