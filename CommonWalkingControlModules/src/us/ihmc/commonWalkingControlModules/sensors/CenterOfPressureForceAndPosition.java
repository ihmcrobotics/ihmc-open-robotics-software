package us.ihmc.commonWalkingControlModules.sensors;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrameHolder;
import us.ihmc.utilities.math.geometry.ReferenceFrameMismatchException;

/**
 * Object to hold pressure Point3d and Force Vector3d
 */
public class CenterOfPressureForceAndPosition implements ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;
   private final Point3d centerOfPressure;
   private final Vector3d totalZForce;

   public CenterOfPressureForceAndPosition(ReferenceFrame referenceFrame, Point3d centerOfPressure, Vector3d totalZForce)
   {
      this.referenceFrame = referenceFrame;
      this.centerOfPressure = new Point3d(centerOfPressure);
      this.totalZForce = new Vector3d(totalZForce);
   }

   public CenterOfPressureForceAndPosition(CenterOfPressureForceAndPosition other)
   {
      this.referenceFrame = other.referenceFrame;
      this.centerOfPressure = new Point3d(other.centerOfPressure);
      this.totalZForce = new Vector3d(other.totalZForce);
   }

   public static CenterOfPressureForceAndPosition combine(CenterOfPressureForceAndPosition centerOfPressureOne,
           CenterOfPressureForceAndPosition centerOfPressureTwo)
   {
      centerOfPressureOne.checkReferenceFrameMatch(centerOfPressureTwo);
      ReferenceFrame referenceFrame = centerOfPressureOne.getReferenceFrame();
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


      CenterOfPressureForceAndPosition ret = new CenterOfPressureForceAndPosition(referenceFrame, centerOfPressure, totalForce);

      return ret;
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      Transform3D toDesired = referenceFrame.getTransformToDesiredFrame(desiredFrame);
      toDesired.transform(centerOfPressure);
      toDesired.transform(totalZForce);
      this.referenceFrame = desiredFrame;
   }

   public void checkReferenceFrameMatch(ReferenceFrameHolder referenceFrameHolder)
   {
      referenceFrame.checkReferenceFrameMatch(referenceFrameHolder.getReferenceFrame());
   }

   public void checkReferenceFrameMatch(ReferenceFrame frame) throws ReferenceFrameMismatchException
   {
      referenceFrame.checkReferenceFrameMatch(frame);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public ReferenceFrameHolder changeFrameCopy(ReferenceFrame desiredFrame)
   {
      CenterOfPressureForceAndPosition ret = new CenterOfPressureForceAndPosition(this);
      ret.changeFrame(desiredFrame);

      return ret;
   }

   public Point3d getCenterOfPressure()
   {
      return new Point3d(centerOfPressure);
   }

   public Vector3d getTotalZForce()
   {
      return new Vector3d(totalZForce);
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
