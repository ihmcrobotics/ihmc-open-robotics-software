package us.ihmc.commonWalkingControlModules.terrain;


import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.GroundProfile;


public class StepUpGroundProfile implements GroundProfile
{
   private final double xMin, xMax, yMin, yMax;

   private final double groundXStep;
   private final double groundZStep;

   public StepUpGroundProfile(double groundXStep, double groundZStep)
   {
      this.xMin = -1.0;
      this.xMax = 4.0;
      this.yMin = -1.0;
      this.yMax = 1.0;

      this.groundXStep = groundXStep;
      this.groundZStep = groundZStep;
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersection, Vector3d normal)
   {
      closestIntersectionTo(x, y, z, intersection);
      surfaceNormalAt(x, y, z, normal);

   }

   public void closestIntersectionTo(double x, double y, double z, Point3d intersection)
   {
      intersection.set(x, y, heightAt(x, y, z));
   }

   public double getXMax()
   {
      return xMax;
   }

   public double getXMin()
   {
      return xMin;
   }

   public double getYMax()
   {
      return yMax;
   }

   public double getYMin()
   {
      return yMin;
   }

   public double heightAt(double x, double y, double z)
   {
      if (x > groundXStep)
         return groundZStep;

      return 0.0;
   }

   public boolean isClose(double x, double y, double z)
   {
      return true;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
   {
      normal.set(0.0, 0.0, 1.0);
   }

}
