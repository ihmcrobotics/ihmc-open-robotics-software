package us.ihmc.rdx.ui.vr;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicBoolean;

import static com.badlogic.gdx.math.MathUtils.random;

/**
 * This class implements a footstep optimizer for virtual reality (VR) applications.
 * It uses a simulated annealing approach to find the best footstep placement
 * based on position, yaw, and planarity costs in a neighbour region,
 * while also satisfying constraints on height, slope of terrain and kinematic steppability.
 */
public class RDXFootstepOptimizer
{
   private static final double GRID_RESOLUTION_XY = 0.02;
   private static final double GRID_RESOLUTION_YAW = Math.toRadians(5);

   private static final double POSITION_W = 10;
   private static final double YAW_W = 50;
   private static final double PLANARITY_W = 70;
   private static final double HEIGHT_W = 20;

   private static final double MAX_HEIGHT_VARIANCE = 0.05;
   private static final double PLANARITY_TOLERANCE = 0.03;
   private static final double CONTINUOUS_SURFACE_TOLERANCE = 0.02;
   private static final double MAX_SLOPE_ANGLE = Math.toRadians(50);

   private static final double DISCONTINUITY_PENALTY = 10.0;
   private static final double HEIGHT_CONSTRAINT_PENALTY = 5.0;
   private static final double SLOPE_CONSTRAINT_PENALTY = 10.0;

   private double footLength;
   private double footWidth;
   private HeightMapData currentHeightMap;
   private double qCurrentMaxSlope = -1;
   private double qMaxHeightVariance = -1;

   private volatile FramePose3D bestSolution; // Holds the most recent best solution
   private ExecutorService executor;
   private Future<?> currentFuture; // Store the future for the current computation
   private AtomicBoolean stopRequested = new AtomicBoolean(false);
   private boolean hasConverged = false;

   /**
    * Constructor for the RDXFootstepOptimizer.
    *
    * @param footLength The length of the foot.
    * @param footWidth  The width of the foot.
    */
   public RDXFootstepOptimizer(double footLength, double footWidth)
   {
      this.footLength = footLength;
      this.footWidth = footWidth;
      this.executor = Executors.newSingleThreadExecutor();
   }

   /**
    * Computes the optimized footstep pose asynchronously.  Returns a
    * CompletableFuture that can be cancelled.
    *
    * @param heightMapData The height map data of the environment.
    * @param initialPose   The initial pose of the footstep.
    * @return A CompletableFuture containing the optimized FramePose3D.
    */
   public CompletableFuture<FramePose3D> computeAsync(HeightMapData heightMapData, FramePose3DReadOnly initialPose)
   {
      // Store the future in a field so it can be cancelled later.
      stopRequested.set(false);
      currentFuture = CompletableFuture.supplyAsync(() ->
       {
          bestSolution = new FramePose3D(initialPose); // Initialize with initial pose
          return compute(heightMapData, initialPose);
       }, executor);

      return (CompletableFuture<FramePose3D>) currentFuture; // Store and return
   }

   /**
    * Attempts to cancel the current running computation, if any.
    */
   public void cancelCompute()
   {
      stopRequested.set(true);
      if (currentFuture != null)
      {
         currentFuture.cancel(true); // Interrupt the thread
         currentFuture = null;
      }
      reset();
   }

   /**
    * Computes the optimized footstep pose.
    *
    * @param heightMapData The height map data of the environment.
    * @param initialPose   The initial pose of the footstep.
    * @return The optimized FramePose3D.
    */
   public FramePose3D compute(HeightMapData heightMapData, FramePose3DReadOnly initialPose)
   {
      currentHeightMap = heightMapData;
      if (isInitialSolutionValid(initialPose))
      {
         LogTools.info("Initial solution is valid, no optimization needed.");
         return new FramePose3D(initialPose);
      }

      double[] qBest = {initialPose.getX(), initialPose.getY(), initialPose.getYaw()};
      double bestCost = Double.POSITIVE_INFINITY;

      double searchRadius = footLength;
      int stepsXY = (int) (2 * searchRadius / GRID_RESOLUTION_XY) + 1;
      int stepsYaw = (int) (Math.PI/4 / GRID_RESOLUTION_YAW);
      for (int i = 0; i < stepsXY; i++)
      {
         for (int j = 0; j < stepsXY; j++)
         {
            for (int k = 0; k < stepsYaw; k++)
            {
               if (stopRequested.get())
               {
                  LogTools.warn("Grid search interrupted.");
                  return createPoseFromQ(qBest, initialPose);
               }

               double x = initialPose.getX() - searchRadius + i * GRID_RESOLUTION_XY;
               double y = initialPose.getY() - searchRadius + j * GRID_RESOLUTION_XY;
               double yaw = initialPose.getYaw() + k * GRID_RESOLUTION_YAW;

               double[] q = {x, y, yaw};
               double cost = computeCost(q, initialPose);

               if (cost < bestCost)
               {
                  bestCost = cost;
                  System.arraycopy(q, 0, qBest, 0, q.length);
               }
            }
         }
      }

      LogTools.info("Grid search finished. Best cost: " + bestCost);
      hasConverged = true;
      bestSolution = createPoseFromQ(qBest, initialPose);
      return bestSolution;
   }

   private FramePose3D createPoseFromQ(double[] q, FramePose3DReadOnly initialPose)
   {
      FramePose3D optimizedPose = new FramePose3D(initialPose);
      optimizedPose.setX(q[0]);
      optimizedPose.setY(q[1]);
      optimizedPose.getRotation().set(new YawPitchRoll(q[2], initialPose.getPitch(), initialPose.getRoll()));
      optimizedPose.setZ(currentHeightMap.getHeightAt(q[0], q[1]));
      return optimizedPose;
   }

   /**
    * Computes the cost associated with a given footstep pose.
    * The cost is a combination of position cost, yaw cost, and planarity cost.
    *
    * @param q          The current footstep pose (x, y, yaw).
    * @param targetPose The target footstep pose.
    * @return The total cost.
    */
   private double computeCost(double[] q, FramePose3DReadOnly targetPose)
   {
      double positionCost = POSITION_W * Math.abs(q[0] - targetPose.getX()) + Math.abs(q[1] - targetPose.getY());
      double yawCost = YAW_W * Math.abs(q[2] - targetPose.getYaw());
      double planarityCost = PLANARITY_W * computePlanarityCost(q);

      // Add penalty for Z distance from initial pose
      double z = currentHeightMap.getHeightAt(q[0], q[1]); // Z of the point to be placed.
      double initialZ = targetPose.getZ(); // Z of the initial Point
      double zDistancePenalty = HEIGHT_W * Math.abs(z - initialZ);

      // Add constraint penalties
      double constraintCost = 0;
      if (!satisfiesHeightConstraint(q))
      {
         constraintCost += HEIGHT_CONSTRAINT_PENALTY;
         LogTools.warn("HEIGHT constraint violated");
      }
      if (!satisfiesSlopeConstraint(q))
      {
         constraintCost += SLOPE_CONSTRAINT_PENALTY;
         LogTools.warn("SLOPE constraint violated");
      }
      return positionCost + yawCost + planarityCost + constraintCost + zDistancePenalty;
   }

   /**
    * Computes the planarity cost for a given foot position.
    *
    * This method:
    * 1. Fits a plane to the five sampled points (four corners and center of the foot) using a least squares method.
    * 2. Calculates the root mean square error (RMSE) of the distances from each point to the fitted plane.
    * 3. Returns this RMSE as the planarity cost.
    *
    * A perfectly planar surface (flat or inclined) will have a cost of zero,
    * while any deviation from planarity will increase the cost.
    *
    * @param q An array containing the foot's position (x, y) and orientation (yaw)
    * @return The planarity cost as the RMSE of distances from sampled points to the fitted plane
    */
   private double computePlanarityCost(double[] q)
   {
      CornerSamplingResults samplingResults = sampleFootCorners(new double[] {q[0], q[1], q[2]});

      if (samplingResults.isSurfaceDiscontinuous)
      {
         // Calculate height differences relative to the center point
         double diff1 = Math.abs(samplingResults.heights[0] - samplingResults.heights[4]); // Corner 1 - Center
         double diff2 = Math.abs(samplingResults.heights[1] - samplingResults.heights[4]); // Corner 2 - Center
         double diff3 = Math.abs(samplingResults.heights[2] - samplingResults.heights[4]); // Corner 3 - Center
         double diff4 = Math.abs(samplingResults.heights[3] - samplingResults.heights[4]); // Corner 4 - Center

         return DISCONTINUITY_PENALTY * (diff1 + diff2 + diff3 + diff4);
      }
      else
      {
         // Fit a plane to the remaining points
         double[] planeCoeffs = fitPlane(samplingResults.cornersXYSubset, samplingResults.heightsSubset);

         // Check if points lie on the same plane within tolerance
         // Calculate the sum of squared distances from each point to the fitted plane
         double sumSquaredDistances = 0;
         double maxVariance = 0;
         double normalizer = Math.sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1] + 1); // Since C = -1
         for (int i = 0; i < 4; i++)
         {
            double distance = Math.abs(planeCoeffs[0] * samplingResults.cornersXYSubset[i][0] + planeCoeffs[1] * samplingResults.cornersXYSubset[i][1]
                                       - samplingResults.heightsSubset[i] + planeCoeffs[3]) / normalizer;
            sumSquaredDistances += distance * distance;
            maxVariance = Math.max(maxVariance, distance);
            LogTools.warn(distance);
         }
         qMaxHeightVariance = maxVariance;

         double rmse = Math.sqrt(sumSquaredDistances / samplingResults.cornersXYSubset.length);  // Root mean square error

         // Calculate the maximum slope of the fitted plane
         double maxSlope = Math.sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1]);
         qCurrentMaxSlope = maxSlope;

         return rmse + maxSlope;
      }
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
         results.heights[i] = currentHeightMap.getHeightAt(x, y);
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

   private boolean isInitialSolutionValid(FramePose3DReadOnly initialPose)
   {
      CornerSamplingResults samplingResults = sampleFootCorners(new double[] {initialPose.getX(), initialPose.getY(), initialPose.getYaw()});

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

   private boolean satisfiesHeightConstraint(double[] q)
   {
      return qMaxHeightVariance <= MAX_HEIGHT_VARIANCE;
   }

   private boolean satisfiesSlopeConstraint(double[] q)
   {
      return qCurrentMaxSlope <= MAX_SLOPE_ANGLE;
   }

   private boolean satisfiesYawConstraint(double[] q, FramePose3DReadOnly initialPose)
   {
      double yawDifference = q[2] - initialPose.getYaw();
      return yawDifference > 0.0;
   }

   public void shutdown()
   {
      executor.shutdown();
   }

   private void reset()
   {
      qCurrentMaxSlope = -1;
      qMaxHeightVariance = -1;
      currentFuture = null;
      hasConverged = false;
   }

   public FramePose3D getCurrentBestSolution()
   {
      return bestSolution; // Return the most recent best solution
   }

   public boolean hasConverged()
   {
      return hasConverged;
   }
}
