package us.ihmc.commonWalkingControlModules.sensors;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * Object to hold pressure Point3d and Force Vector3d
 */
public class CenterOfPressureForceAndPosition extends ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;
   private final Point3d centerOfPressure;
   private final Vector3d totalForce;

   public CenterOfPressureForceAndPosition(ReferenceFrame referenceFrame, Tuple3d centerOfPressure, Vector3d totalZForce)
   {
      this.referenceFrame = referenceFrame;
      this.centerOfPressure = new Point3d(centerOfPressure);
      this.totalForce = new Vector3d(totalZForce);
   }

   public CenterOfPressureForceAndPosition(CenterOfPressureForceAndPosition other)
   {
      this.referenceFrame = other.referenceFrame;
      this.centerOfPressure = new Point3d(other.centerOfPressure);
      this.totalForce = new Vector3d(other.totalForce);
   }

   public static CenterOfPressureForceAndPosition combine(CenterOfPressureForceAndPosition centerOfPressureOne,
           CenterOfPressureForceAndPosition centerOfPressureTwo)
   {
      centerOfPressureOne.checkReferenceFrameMatch(centerOfPressureTwo);
      ReferenceFrame referenceFrame = centerOfPressureOne.getReferenceFrame();
      Point3d centerOfPressure = new Point3d();
      Vector3d totalForce = new Vector3d();

      if (isAValidForcePosition(centerOfPressureOne.centerOfPressure) && (isAValidForceVector(centerOfPressureOne.totalForce)))
      {
         Point3d scaledCenterOfPressure = new Point3d(centerOfPressureOne.centerOfPressure);
         scaledCenterOfPressure.scale(centerOfPressureOne.totalForce.getZ());
         centerOfPressure.add(scaledCenterOfPressure);

         totalForce.add(centerOfPressureOne.totalForce);
      }

      if (isAValidForcePosition(centerOfPressureTwo.centerOfPressure) && (isAValidForceVector(centerOfPressureTwo.totalForce)))
      {
         Point3d scaledCenterOfPressure = new Point3d(centerOfPressureTwo.centerOfPressure);
         scaledCenterOfPressure.scale(centerOfPressureTwo.totalForce.getZ());
         centerOfPressure.add(scaledCenterOfPressure);

         totalForce.add(centerOfPressureTwo.totalForce);
      }

      centerOfPressure.scale(1.0 / totalForce.getZ());


      CenterOfPressureForceAndPosition ret = new CenterOfPressureForceAndPosition(referenceFrame, centerOfPressure, totalForce);

      return ret;
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      RigidBodyTransform toDesired = referenceFrame.getTransformToDesiredFrame(desiredFrame);
      toDesired.transform(centerOfPressure);
      toDesired.transform(totalForce);
      this.referenceFrame = desiredFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public Point3d getCenterOfPressure()
   {
      return new Point3d(centerOfPressure);
   }

   public Vector3d getTotalForce()
   {
      return new Vector3d(totalForce);
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
