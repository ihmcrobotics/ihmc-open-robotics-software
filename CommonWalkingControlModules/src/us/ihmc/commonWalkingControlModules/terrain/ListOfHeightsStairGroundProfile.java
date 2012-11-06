package us.ihmc.commonWalkingControlModules.terrain;

import java.util.Arrays;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.MathTools;

import com.yobotics.simulationconstructionset.GroundProfile;

public class ListOfHeightsStairGroundProfile implements GroundProfile
{
   private final double xMin, xMax, yMin, yMax;
   private final double
      xTiles = 1.0, yTiles = 1.0;

   private final double[] stepStartXValues;
   private final double[] groundHeights;

   public ListOfHeightsStairGroundProfile(double[] stepHeights, double[] stepTreads, double initialHeight, double startX)
   {
      if (MathTools.min(stepTreads) <= 0.0)
         throw new RuntimeException("Step treads must be positive");

      if (stepHeights.length != stepTreads.length + 1)
         throw new RuntimeException("stepHeights.length != stepTreads.length + 1");
      
      double[] xCumulativeSum = MathTools.cumulativeSumDoubles(stepTreads);
      stepStartXValues = new double[xCumulativeSum.length + 1];
      System.arraycopy(xCumulativeSum, 0, stepStartXValues, 1, xCumulativeSum.length);
      for (int i = 0; i < stepStartXValues.length; i++)
         stepStartXValues[i] = stepStartXValues[i] + startX;

      double[] heightCumulativeSum = MathTools.cumulativeSumDoubles(stepHeights);
      groundHeights = new double[heightCumulativeSum.length + 1];
      System.arraycopy(heightCumulativeSum, 0, groundHeights, 1, heightCumulativeSum.length);
      for (int i = 0; i < groundHeights.length; i++)
         groundHeights[i] = groundHeights[i] + initialHeight;

      double leadInX = 1.0;
      double leadOutX = 1.0;
      this.xMin = startX - leadInX;
      this.xMax = MathTools.max(stepStartXValues) + leadOutX;

      this.yMin = -1.0;
      this.yMax = 1.0;
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

   public double getXTiles()
   {
      return xTiles;
   }

   public double getYMax()
   {
      return yMax;
   }

   public double getYMin()
   {
      return yMin;
   }

   public double getYTiles()
   {
      return yTiles;
   }

   public double heightAt(double x, double y, double z)
   {
      // read info on how Arrays.binarySearch works for index magic.
      int index = Arrays.binarySearch(stepStartXValues, x);
      if (index < 0)
         index = -(index + 1);

      double height = groundHeights[index];

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

}
