package us.ihmc.commonWalkingControlModules.sensors;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * Object to hold pressure Point3d and Force Vector3d
 */
public class CenterOfPressureForceAndPosition
{
   public Point3d centerOfPressure;
   public Vector3d totalZForce;

   public CenterOfPressureForceAndPosition(Point3d centerOfPressure, Vector3d totalZForce)
   {
      this.centerOfPressure = centerOfPressure;
      this.totalZForce = totalZForce;
   }

   public static CenterOfPressureForceAndPosition combine(CenterOfPressureForceAndPosition centerOfPressureOne,
           CenterOfPressureForceAndPosition centerOfPressureTwo)
   {
      Point3d centerOfPressure = new Point3d();
      Vector3d totalForce = new Vector3d();

      if (isAValidForcePosition(centerOfPressureOne.centerOfPressure) && (isAValidForceVector(centerOfPressureOne.totalZForce)))
      {
         Point3d scaledCenterOfPressure = new Point3d(centerOfPressureOne.centerOfPressure);
         scaledCenterOfPressure.scale(centerOfPressureOne.totalZForce.getZ());
         centerOfPressure.add(scaledCenterOfPressure);

         totalForce.add(centerOfPressureOne.totalZForce);
      }

      if (isAValidForcePosition(centerOfPressureTwo.centerOfPressure) && (isAValidForceVector(centerOfPressureTwo.totalZForce)))
      {
         Point3d scaledCenterOfPressure = new Point3d(centerOfPressureTwo.centerOfPressure);
         scaledCenterOfPressure.scale(centerOfPressureTwo.totalZForce.getZ());
         centerOfPressure.add(scaledCenterOfPressure);

         totalForce.add(centerOfPressureTwo.totalZForce);
      }

      centerOfPressure.scale(1.0 / totalForce.getZ());


      CenterOfPressureForceAndPosition ret = new CenterOfPressureForceAndPosition(centerOfPressure, totalForce);

      return ret;
   }

   private static boolean isAValidForceVector(Vector3d vector3d)
   {
      if (vector3d == null)
         return false;

      if (Double.isNaN(vector3d.getX()))
         return false;
      if (Double.isNaN(vector3d.getY()))
         return false;
      if (Double.isNaN(vector3d.getZ()))
         return false;

      return true;
   }

   private static boolean isAValidForcePosition(Point3d point3d)
   {
      if (point3d == null)
         return false;

      if (Double.isNaN(point3d.getX()))
         return false;
      if (Double.isNaN(point3d.getY()))
         return false;
      if (Double.isNaN(point3d.getZ()))
         return false;

      return true;
   }
}
