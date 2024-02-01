package us.ihmc.robotics;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidGeometryMissingTools
{
   public static double getZOnPlane(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal, double pointX, double pointY)
   {
      // The three components of the plane origin
      double x0 = pointOnPlane.getX();
      double y0 = pointOnPlane.getY();
      double z0 = pointOnPlane.getZ();
      // The three components of the plane normal
      double a = planeNormal.getX();
      double b = planeNormal.getY();
      double c = planeNormal.getZ();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - pointX) + b / c * (y0 - pointY) + z0;
      return z;
   }
}
