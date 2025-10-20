package us.ihmc.perception.gpuMapping;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

public class PlanarityChecker
{
   private static final double MAX_HEIGHT_VARIANCE = 0.05;
   private static final double PLANARITY_TOLERANCE = 0.03;
   private static final double CONTINUOUS_SURFACE_TOLERANCE = 0.02;

   private double footLength;
   private double footWidth;
   private HeightMapData currentHeightMap;

   public PlanarityChecker(double footLength, double footWidth)
   {
      this.footLength = footLength;
      this.footWidth = footWidth;
   }

   /**
    * Helper class to hold the results of sampling the foot corners and identifying the worst one.
    */
   private class CornerSamplingResults
   {
      public double[][] cornersXY;
      public double[][] cornersXYSubset;
      public double[] heights;
      public double[] heightsSubset;
      public int maxDiffIndex;
      public boolean isSurfaceDiscontinuous;
   }

   /**
    * Samples the foot corners, calculates the corresponding world coordinates and heights,
    * and identifies the corner with the largest height difference to the mean.
    *
    * @param q The footstep pose (x, y, yaw).
    * @return A CornerSamplingResults object containing the sampling results.
    */
   private CornerSamplingResults sampleFootCorners(double[] q)
   {
      CornerSamplingResults results = new CornerSamplingResults();

      double fx = footLength;
      double fy = footWidth - 0.02;
      double[][] corners = {{-fx / 2, -fy / 2}, {fx / 2, -fy / 2}, {fx / 2, fy / 2}, {-fx / 2, fy / 2}, {0, 0}};

      results.cornersXY = new double[5][2];
      results.heights = new double[5];

      for (int i = 0; i < 5; i++)
      {
         double x = q[0] + corners[i][0] * Math.cos(q[2]) - corners[i][1] * Math.sin(q[2]);
         double y = q[1] + corners[i][0] * Math.sin(q[2]) + corners[i][1] * Math.cos(q[2]);
         results.heights[i] = currentHeightMap.getHeight(x, y);
         results.cornersXY[i] = new double[] {x, y};
      }

      // Find the corner with the largest height difference to the mean
      results.maxDiffIndex = 0;
      double meanHeight = 0;
      for (double height : results.heights)
      {
         meanHeight += height;
      }
      meanHeight /= 5;

      double maxDiff = Math.abs(results.heights[0] - meanHeight);
      for (int j = 1; j < 5; j++)
      {
         double diff = Math.abs(results.heights[j] - meanHeight);
         if (diff > maxDiff)
         {
            maxDiff = diff;
            results.maxDiffIndex = j;
         }
      }

      // Remove the worst corner
      results.cornersXYSubset = new double[4][2];
      results.heightsSubset = new double[4];
      int cornerIndex = 0;
      for (int j = 0; j < 5; j++)
      {
         if (j != results.maxDiffIndex)
         {
            results.cornersXYSubset[cornerIndex] = results.cornersXY[j];
            results.heightsSubset[cornerIndex] = results.heights[j];
            cornerIndex++;
         }
      }

      // Check continuity of surface
      double frontHeightAvg = (results.heights[0] + results.heights[3]) / 2.0;
      double backHeightAvg = (results.heights[1] + results.heights[2]) / 2.0;
      results.isSurfaceDiscontinuous = Math.abs(results.heights[4] - (frontHeightAvg + backHeightAvg) / 2.0) > CONTINUOUS_SURFACE_TOLERANCE;

      return results;
   }

   /**
    * Fits a plane to the given points using a least squares method.
    *
    * @param points  An array of 2D points (x, y).
    * @param heights An array of corresponding heights (z) for each point.
    * @return An array containing the coefficients of the plane equation (A, B, C, D) for Ax + By + Cz + D = 0.
    */
   private double[] fitPlane(double[][] points, double[] heights)
   {
      // Simple least squares plane fitting
      int n = points.length;
      double sumX = 0, sumY = 0, sumZ = 0, sumX2 = 0, sumY2 = 0, sumXY = 0, sumXZ = 0, sumYZ = 0;

      for (int i = 0; i < n; i++)
      {
         double x = points[i][0];
         double y = points[i][1];
         double z = heights[i];

         sumX += x;
         sumY += y;
         sumZ += z;
         sumX2 += x * x;
         sumY2 += y * y;
         sumXY += x * y;
         sumXZ += x * z;
         sumYZ += y * z;
      }

      double det = n * (sumX2 * sumY2 - sumXY * sumXY) + sumX * (sumXY * sumY - sumX * sumY2) + sumY * (sumX * sumXY - sumX2 * sumY);

      double a = (n * (sumXZ * sumY2 - sumYZ * sumXY) + sumY * (sumYZ * sumX - sumXZ * sumY) + sumZ * (sumXY * sumY - sumX * sumY2)) / det;

      double b = (n * (sumX2 * sumYZ - sumXZ * sumXY) + sumX * (sumXZ * sumY - sumYZ * sumX) + sumZ * (sumX * sumXY - sumX2 * sumY)) / det;

      double c = (sumZ - a * sumX - b * sumY) / n;

      return new double[] {a, b, -1, c};  // Ax + By - z + C = 0 -> z = Ax + By + C
   }

   public boolean isOnPlane(HeightMapData heightMapData, FramePose3DReadOnly footstepPose)
   {
      currentHeightMap = heightMapData;
      CornerSamplingResults samplingResults = sampleFootCorners(new double[] {footstepPose.getX(), footstepPose.getY(), footstepPose.getYaw()});

      if (samplingResults.isSurfaceDiscontinuous)
      {
         return false;
      }

      // Fit a plane to the remaining points
      double[] planeCoeffs = fitPlane(samplingResults.cornersXYSubset, samplingResults.heightsSubset);

      // Check if points lie on the same plane within tolerance
      boolean onSamePlane = true;
      double normalizer = Math.sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1] + 1); // Since C = -1
      for (int i = 0; i < 4; i++)
      {
         double distance = Math.abs(planeCoeffs[0] * samplingResults.cornersXYSubset[i][0] + planeCoeffs[1] * samplingResults.cornersXYSubset[i][1]
                                    - samplingResults.heightsSubset[i] + planeCoeffs[3]) / normalizer;
         if (distance > PLANARITY_TOLERANCE)
         {
            onSamePlane = false;
            break;
         }
      }

      // check if the other excluded point is too distant
      double distance = Math.abs(planeCoeffs[0] * samplingResults.cornersXY[samplingResults.maxDiffIndex][0]
                                 + planeCoeffs[1] * samplingResults.cornersXY[samplingResults.maxDiffIndex][1]
                                 - samplingResults.heights[samplingResults.maxDiffIndex] + planeCoeffs[3]) / normalizer;
      if (distance > MAX_HEIGHT_VARIANCE)
      {
         onSamePlane = false;
      }

      return onSamePlane;
   }
}
