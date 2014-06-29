package us.ihmc.commonWalkingControlModules.terrain;

import java.util.Arrays;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.utilities.CheckTools;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.BoundingBox3d;


public class VaryingStairGroundProfile implements GroundProfile
{
   private final BoundingBox3d boundingBox;

   private final double[] stepStartXValues;
   private final double[] groundHeights;

   public VaryingStairGroundProfile(double startX, double startZ, double[] stepTreads, double[] stepRises)
   {
      if (MathTools.min(stepTreads) <= 0.0)
         throw new RuntimeException("Step treads must be positive");

      if (stepRises.length != stepTreads.length + 1)
         throw new RuntimeException("stepHeights.length != stepTreads.length + 1");
      
      double[] xCumulativeSum = MathTools.cumulativeSumDoubles(stepTreads);
      stepStartXValues = new double[xCumulativeSum.length + 1];
      System.arraycopy(xCumulativeSum, 0, stepStartXValues, 1, xCumulativeSum.length);
      for (int i = 0; i < stepStartXValues.length; i++)
         stepStartXValues[i] = stepStartXValues[i] + startX;

      double[] heightCumulativeSum = MathTools.cumulativeSumDoubles(stepRises);
      groundHeights = new double[heightCumulativeSum.length + 1];
      System.arraycopy(heightCumulativeSum, 0, groundHeights, 1, heightCumulativeSum.length);
      for (int i = 0; i < groundHeights.length; i++)
         groundHeights[i] = groundHeights[i] + startZ;

      double leadInX = 1.0;
      double leadOutX = 1.0;
      double xMin = startX - leadInX;
      double xMax = MathTools.max(stepStartXValues) + leadOutX;

      double yMin = -1.0;
      double yMax = 1.0;
      
      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = Double.POSITIVE_INFINITY;
      
      boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
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

   public double heightAt(double x, double y, double z)
   {
      int index = computeStepNumber(x);

      double height = computeGroundHeight(index);

      return height;
   }

   public boolean isClose(double x, double y, double z)
   {
      return true;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
   {
      normal.set(0.0, 0.0, 1.0);
   }

   public int computeStepNumber(double x)
   {
      // read Arrays.binarySearch javadoc for index magic.
      int index = Arrays.binarySearch(stepStartXValues, x);
      if (index < 0)
         index = -(index + 1);
      return index;
   }

   public double computeStepStartX(int index)
   {
      CheckTools.checkRange(index, 0, stepStartXValues.length);

      if (index == stepStartXValues.length)
         return Double.POSITIVE_INFINITY;
      else
         return stepStartXValues[index];
   }

   public double computeGroundHeight(int index)
   {
      return groundHeights[index];
   }
   
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }
}
