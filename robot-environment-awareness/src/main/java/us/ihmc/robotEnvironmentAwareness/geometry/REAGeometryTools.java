package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class REAGeometryTools
{
   public static double distanceSquaredBetweenTwoBoundingBox3Ds(Point3DReadOnly min1, Point3DReadOnly max1, Point3DReadOnly min2, Point3DReadOnly max2)
   {
      double dx = EuclidCoreTools.max(min1.getX() - max2.getX(), 0.0, min2.getX() - max1.getX());
      double dy = EuclidCoreTools.max(min1.getY() - max2.getY(), 0.0, min2.getY() - max1.getY());
      double dz = EuclidCoreTools.max(min1.getZ() - max2.getZ(), 0.0, min2.getZ() - max1.getZ());
      return dx * dx + dy * dy + dz * dz;
   }

}
