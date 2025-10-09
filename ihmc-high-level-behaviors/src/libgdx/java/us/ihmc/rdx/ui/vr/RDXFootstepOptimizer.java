package us.ihmc.rdx.ui.vr;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuMapping.HeightMapData;

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
   private static final float GRID_RESOLUTION_XY = 0.02f;
   private static final float GRID_RESOLUTION_YAW = (float) Math.toRadians(5.0);

   private static final float POSITION_W = 10f;
   private static final float YAW_W = 50f;
   private static final float PLANARITY_W = 100f;
   private static final float HEIGHT_W = 20f;

   private static final float MAX_HEIGHT_VARIANCE = 0.05f;
   private static final float PLANARITY_TOLERANCE = 0.03f;
   private static final float CONTINUOUS_SURFACE_TOLERANCE = 0.02f;
   private static final float MAX_SLOPE_ANGLE = (float) Math.toRadians(50);

   private static final float DISCONTINUITY_PENALTY = 50.0f;
   private static final float HEIGHT_CONSTRAINT_PENALTY = 5.0f;
   private static final float SLOPE_CONSTRAINT_PENALTY = 10.0f;

   private float footLength;
   private float footWidth;
   private HeightMapData currentHeightMap;

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
      this.footLength = (float) footLength;
      this.footWidth = (float) footWidth;
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

      float[] qBest = {(float) initialPose.getX(), (float) initialPose.getY(), (float) initialPose.getYaw()};
      float bestCost = Float.POSITIVE_INFINITY;

      float searchRadius = footLength;
      int stepsXY = (int) (2 * searchRadius / GRID_RESOLUTION_XY) + 1;
      int stepsYaw = (int) (Math.PI/4 / GRID_RESOLUTION_YAW);
      int n =0;
      int bestIndex = 0;
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

               float x = (float) initialPose.getX() - searchRadius + i * GRID_RESOLUTION_XY;
               float y = (float) initialPose.getY() - searchRadius + j * GRID_RESOLUTION_XY;
               float yaw = (float) initialPose.getYaw() + k * GRID_RESOLUTION_YAW;

               float[] q = {x, y, yaw};
               float cost = computeCost(q, initialPose);

               if (cost < bestCost)
               {
                  bestCost = cost;
                  bestIndex = n;
                  System.arraycopy(q, 0, qBest, 0, q.length);
               }
               n++;
            }
         }
      }

      LogTools.info("Grid search finished. Best cost: {}, idx: {}", bestCost, bestIndex);
      hasConverged = true;
      bestSolution = createPoseFromQ(qBest, initialPose);
      return bestSolution;
   }

   private FramePose3D createPoseFromQ(float[] q, FramePose3DReadOnly initialPose)
   {
      FramePose3D optimizedPose = new FramePose3D(initialPose);
      optimizedPose.setX(q[0]);
      optimizedPose.setY(q[1]);
      optimizedPose.getRotation().set(new YawPitchRoll(q[2], initialPose.getPitch(), initialPose.getRoll()));
      optimizedPose.setZ(currentHeightMap.getHeight(q[0], q[1]));
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
   private float computeCost(float[] q, FramePose3DReadOnly targetPose)
   {
      float positionCost = POSITION_W * (Math.abs(q[0] - (float) targetPose.getX()) + Math.abs(q[1] - (float) targetPose.getY()));
      float yawCost = YAW_W * Math.abs(q[2] - (float) targetPose.getYaw());
      float planarityCost = PLANARITY_W * computePlanarityCost(q);

      // Add penalty for Z distance from initial pose
      float z = (float) currentHeightMap.getHeight(q[0], q[1]); // Z of the point to be placed.
      float initialZ = (float) targetPose.getZ(); // Z of the initial Point
      float zDistancePenalty = HEIGHT_W * Math.abs(z - initialZ);

      return positionCost + yawCost + planarityCost + zDistancePenalty;
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
   private float computePlanarityCost(float[] q)
   {
      CornerSamplingResults samplingResults = sampleFootCorners(new float[] {q[0], q[1], q[2]});

      if (samplingResults.isSurfaceDiscontinuous)
      {
         // Calculate height differences relative to the center point
         float diff1 = Math.abs(samplingResults.heights[0] - samplingResults.heights[4]); // Corner 1 - Center
         float diff2 = Math.abs(samplingResults.heights[1] - samplingResults.heights[4]); // Corner 2 - Center
         float diff3 = Math.abs(samplingResults.heights[2] - samplingResults.heights[4]); // Corner 3 - Center
         float diff4 = Math.abs(samplingResults.heights[3] - samplingResults.heights[4]); // Corner 4 - Center

         return DISCONTINUITY_PENALTY * (diff1 + diff2 + diff3 + diff4);
      }
      else
      {
         // Fit a plane to the remaining points
         float[] planeCoeffs = fitPlane(samplingResults.cornersXYSubset, samplingResults.heightsSubset);
         // Check if points lie on the same plane within tolerance
         // Calculate the sum of squared distances from each point to the fitted plane
         float sumDistances = 0;
         float maxVariance = 0;
         float normalizer = (float) Math.sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1] + 1); // Since C = -1
         for (int i = 0; i < 4; i++)
         {
            float distance = Math.abs(planeCoeffs[0] * samplingResults.cornersXYSubset[i][0] +
                                      planeCoeffs[1] * samplingResults.cornersXYSubset[i][1] -
                                      samplingResults.heightsSubset[i] + planeCoeffs[3]) / normalizer;
            sumDistances += distance;
            maxVariance = Math.max(maxVariance, distance);
         }
         float meanError = sumDistances / 4;  // Root mean square error

         // Calculate the maximum slope of the fitted plane
         float maxSlope = (float) Math.atan(Math.sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1]));

         float penalties = 0.0f;
         if (maxVariance > MAX_HEIGHT_VARIANCE)
         {
            penalties += HEIGHT_CONSTRAINT_PENALTY;
         }
         if (maxSlope > MAX_SLOPE_ANGLE)
         {
            penalties += SLOPE_CONSTRAINT_PENALTY;
         }

         return meanError + penalties;
      }
   }

   /**
    * Helper class to hold the results of sampling the foot corners and identifying the worst one.
    */
   private class CornerSamplingResults
   {
      public float[][] cornersXY;
      public float[][] cornersXYSubset;
      public float[] heights;
      public float[] heightsSubset;
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
   private CornerSamplingResults sampleFootCorners(float[] q)
   {
      CornerSamplingResults results = new CornerSamplingResults();

      float fx = footLength;
      float fy = footWidth;
      float[][] corners = {{-fx / 2, -fy / 2}, {fx / 2, -fy / 2}, {fx / 2, fy / 2}, {-fx / 2, fy / 2}, {0, 0}};

      results.cornersXY = new float[5][2];
      results.heights = new float[5];

      for (int i = 0; i < 5; i++)
      {
         float x = (float) (q[0] + corners[i][0] * Math.cos(q[2]) - corners[i][1] * Math.sin(q[2]));
         float y = (float) (q[1] + corners[i][0] * Math.sin(q[2]) + corners[i][1] * Math.cos(q[2]));
         results.heights[i] = (float) currentHeightMap.getHeight(x, y);
         results.cornersXY[i] = new float[] {x, y};
      }

      // Find the corner with the largest height difference to the mean
      results.maxDiffIndex = 0;
      float meanHeight = 0;
      for (float height : results.heights)
      {
         meanHeight += height;
      }
      meanHeight /= 5;

      float maxDiff = Math.abs(results.heights[0] - meanHeight);
      for (int j = 1; j < 5; j++)
      {
         float diff = Math.abs(results.heights[j] - meanHeight);
         if (diff > maxDiff)
         {
            maxDiff = diff;
            results.maxDiffIndex = j;
         }
      }

      // Remove the worst corner
      results.cornersXYSubset = new float[4][2];
      results.heightsSubset = new float[4];
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
      float frontHeightAvg = (results.heights[0] + results.heights[3]) / 2.0f;
      float backHeightAvg = (results.heights[1] + results.heights[2]) / 2.0f;
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
   private float[] fitPlane(float[][] points, float[] heights)
   {
      // Simple least squares plane fitting
      int n = points.length;
      float sumX = 0, sumY = 0, sumZ = 0, sumX2 = 0, sumY2 = 0, sumXY = 0, sumXZ = 0, sumYZ = 0;

      for (int i = 0; i < n; i++)
      {
         float x = points[i][0];
         float y = points[i][1];
         float z = heights[i];

         sumX += x;
         sumY += y;
         sumZ += z;
         sumX2 += x * x;
         sumY2 += y * y;
         sumXY += x * y;
         sumXZ += x * z;
         sumYZ += y * z;
      }

      float det = n * (sumX2 * sumY2 - sumXY * sumXY) + sumX * (sumXY * sumY - sumX * sumY2) + sumY * (sumX * sumXY - sumX2 * sumY);

      float a = (n * (sumXZ * sumY2 - sumYZ * sumXY) + sumY * (sumYZ * sumX - sumXZ * sumY) + sumZ * (sumXY * sumY - sumX * sumY2)) / det;

      float b = (n * (sumX2 * sumYZ - sumXZ * sumXY) + sumX * (sumXZ * sumY - sumYZ * sumX) + sumZ * (sumX * sumXY - sumX2 * sumY)) / det;

      float c = (sumZ - a * sumX - b * sumY) / n;

      return new float[] {a, b, -1.0f, c};  // Ax + By - z + C = 0 -> z = Ax + By + C
   }

   private boolean isInitialSolutionValid(FramePose3DReadOnly initialPose)
   {
      CornerSamplingResults samplingResults = sampleFootCorners(new float[] {(float) initialPose.getX(), (float) initialPose.getY(), (float) initialPose.getYaw()});

      if (samplingResults.isSurfaceDiscontinuous)
      {
         return false;
      }

      // Fit a plane to the remaining points
      float[] planeCoeffs = fitPlane(samplingResults.cornersXYSubset, samplingResults.heightsSubset);

      // Check if points lie on the same plane within tolerance
      boolean onSamePlane = true;
      float normalizer = (float) Math.sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1] + 1); // Since C = -1
      for (int i = 0; i < 4; i++)
      {
         float distance = Math.abs(planeCoeffs[0] * samplingResults.cornersXYSubset[i][0] + planeCoeffs[1] * samplingResults.cornersXYSubset[i][1]
                                    - samplingResults.heightsSubset[i] + planeCoeffs[3]) / normalizer;
         if (distance > PLANARITY_TOLERANCE)
         {
            onSamePlane = false;
            break;
         }
      }

      // check if the other excluded point is too distant
      float distance = Math.abs(planeCoeffs[0] * samplingResults.cornersXY[samplingResults.maxDiffIndex][0]
                                 + planeCoeffs[1] * samplingResults.cornersXY[samplingResults.maxDiffIndex][1]
                                 - samplingResults.heights[samplingResults.maxDiffIndex] + planeCoeffs[3]) / normalizer;
      if (distance > MAX_HEIGHT_VARIANCE)
      {
         onSamePlane = false;
      }

      return onSamePlane;
   }

   public void shutdown()
   {
      executor.shutdown();
   }

   public void reset()
   {
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
