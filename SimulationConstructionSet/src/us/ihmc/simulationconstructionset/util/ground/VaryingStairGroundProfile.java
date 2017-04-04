package us.ihmc.simulationconstructionset.util.ground;

import java.util.Arrays;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;

public class VaryingStairGroundProfile extends GroundProfileFromHeightMap
{
   private final BoundingBox3D boundingBox;

   private final double[] stepStartXValues;
   private final double[] groundHeights;

   public VaryingStairGroundProfile(double startX, double startZ, double[] stepTreads, double[] stepRises)
   {
      if (MathTools.min(stepTreads) <= 0.0)
         throw new RuntimeException("Step treads must be positive");

      if (stepRises.length != stepTreads.length + 1)
         throw new RuntimeException("stepHeights.length != stepTreads.length + 1");
      
      double[] xCumulativeSum = MathTools.cumulativeSum(stepTreads);
      stepStartXValues = new double[xCumulativeSum.length + 1];
      System.arraycopy(xCumulativeSum, 0, stepStartXValues, 1, xCumulativeSum.length);
      for (int i = 0; i < stepStartXValues.length; i++)
         stepStartXValues[i] = stepStartXValues[i] + startX;

      double[] heightCumulativeSum = MathTools.cumulativeSum(stepRises);
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
      
      boundingBox = new BoundingBox3D(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double height = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      
      return height;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      int index = computeStepNumber(x);

      double height = computeGroundHeight(index);

      return height;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
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
      MathTools.checkIntervalContains(index, 0, stepStartXValues.length);

      if (index == stepStartXValues.length)
         return Double.POSITIVE_INFINITY;
      else
         return stepStartXValues[index];
   }

   public double computeGroundHeight(int index)
   {
      return groundHeights[index];
   }
   
   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }
}