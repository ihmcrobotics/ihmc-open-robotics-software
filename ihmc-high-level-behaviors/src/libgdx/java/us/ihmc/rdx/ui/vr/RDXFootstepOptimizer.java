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
   private static final double POSITION_W = 0.2;
   private static final double YAW_W = 0.5;
   private static final double PLANARITY_W = 0.3;

   private static final double POSITION_STD_DEV = 0.05;  // probability density of 0.7 is achieved at roughly std_dev/2, hence 70% of times we get half as a step
   private static final double YAW_STD_DEV = Math.toRadians(0);
   private static final double INITIAL_TEMPERATURE = 100.0;
   private static final double COOLING_RATE = 0.95;
   private static final int MAX_ITERATIONS = 100;

   private static final double MAX_HEIGHT_VARIANCE = 0.05;
   private static final double PLANARITY_TOLERANCE = 0.03;
   private static final double MAX_SLOPE_ANGLE = Math.toRadians(50);

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
    * Computes the optimized footstep pose. This method is private and called from computeAsync.
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

      double[] q = {initialPose.getX(), initialPose.getY(), initialPose.getYaw()}; // q as initial pose.
      double[] qBest = q.clone(); // keep the best solution here

      double bestCost = computeCost(q, initialPose);

      double temperature = INITIAL_TEMPERATURE;

      for (int i = 0; i < MAX_ITERATIONS; i++)
      {
         if (stopRequested.get())
         {
            LogTools.warn("Optimization interrupted.");
            break;
         }

         // Generate a neighbor solution by randomly perturbing the current pose
         double[] qNew = generateNeighbor(q, initialPose);

         double newCost = computeCost(qNew, initialPose);
         double costDelta = newCost - bestCost;

         // Metropolis acceptance criterion
         if (costDelta < 0 || random.nextDouble() < Math.exp(-costDelta / temperature))
         {
            System.arraycopy(qNew, 0, q, 0, q.length);  // Update q (current solution)
            if (newCost < bestCost)
            {
               bestCost = newCost;
               System.arraycopy(qNew, 0, qBest, 0, q.length);   // Update qBest (best solution)
               LogTools.info("New best cost: " + bestCost);
            }
         }

         // Cool down the temperature
         temperature *= COOLING_RATE;

         LogTools.info("Iteration: " + i + ", Temperature: " + temperature + ", Best Cost: " + bestCost);
      }

      LogTools.info("Simulated Annealing finished. Best cost: " + bestCost);
      hasConverged = true;

      // Create and return the best solution, using qBest
      FramePose3D optimizedPose = new FramePose3D(initialPose);
      optimizedPose.setX(qBest[0]);
      optimizedPose.setY(qBest[1]);
      optimizedPose.getRotation().set(new YawPitchRoll(qBest[2], initialPose.getPitch(), initialPose.getRoll()));
      optimizedPose.setZ(currentHeightMap.getHeightAt(qBest[0], qBest[1]));
      bestSolution = optimizedPose;  // Ensure bestSolution is correctly set
      return bestSolution;
   }

   private double[] generateNeighbor(double[] q, FramePose3DReadOnly initialPose)
   {
      double x = q[0] + random.nextGaussian() * POSITION_STD_DEV;
      double y = q[1] + random.nextGaussian() * POSITION_STD_DEV;
      double yaw = q[2] + random.nextGaussian() * YAW_STD_DEV;

      double[] newQ = new double[] {x, y, yaw};
      if (!satisfiesPositionConstraint(q, initialPose))
      {
         LogTools.warn("Out of bound neighbor. Getting new one");
         generateNeighbor(q, initialPose);
      }
      return newQ;
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
      double positionCost = POSITION_W * (Math.pow(q[0] - targetPose.getX(), 2) + Math.pow(q[1] - targetPose.getY(), 2));
      double yawCost = YAW_W * Math.pow(q[2] - targetPose.getYaw(), 2);
      double planarityCost = PLANARITY_W * computePlanarityCost(q);
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
      return positionCost + yawCost + planarityCost + constraintCost;
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
      CornerSamplingResults samplingResults = sampleFootCorners(q);

      // Fit a plane to the remaining points
      double[] planeCoeffs = fitPlane(samplingResults.cornersXYSubset, samplingResults.heightsSubset);

      // Calculate the sum of squared distances from each point to the fitted plane
      double sumSquaredDistances = 0;
      double maxVariance = 0;
      for (int i = 0; i < samplingResults.cornersXYSubset.length; i++)
      {
         double distance = Math.abs(
               planeCoeffs[0] * samplingResults.cornersXYSubset[i][0] + planeCoeffs[1] * samplingResults.cornersXYSubset[i][1] + planeCoeffs[2]
               - samplingResults.heightsSubset[i]);
         sumSquaredDistances += distance * distance;
         maxVariance = Math.max(maxVariance, distance);
      }
      qMaxHeightVariance = maxVariance;

      double rmse = Math.sqrt(sumSquaredDistances / samplingResults.cornersXYSubset.length);  // Root mean square error

      // Calculate the maximum slope of the fitted plane
      double maxSlope = Math.sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1]);
      qCurrentMaxSlope = maxSlope;

      return rmse + maxSlope;
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
      double fy = footWidth;
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

   private boolean satisfiesPositionConstraint(double[] q, FramePose3DReadOnly initialPose)
   {
      double distance = Math.sqrt(Math.pow(q[0] - initialPose.getX(), 2) + Math.pow(q[1] - initialPose.getY(), 2));
      return distance <= 2 * footLength;
   }

   private boolean satisfiesHeightConstraint(double[] q)
   {
      return qMaxHeightVariance <= MAX_HEIGHT_VARIANCE;
   }

   private boolean satisfiesSlopeConstraint(double[] q)
   {
      return qCurrentMaxSlope <= Math.tan(MAX_SLOPE_ANGLE);
   }

   private boolean satisfiesYawConstraint(double[] q, FramePose3DReadOnly initialPose)
   {
      double yawDifference = q[2] - initialPose.getYaw();
      return yawDifference > 0.0;
   }

   /**
    * Computes the numerical gradient of the cost function with respect to the footstep pose.
    *
    * @param q          The current footstep pose (x, y, yaw).
    * @param targetPose The target footstep pose.
    * @return The gradient of the cost function.
    */
   private double[] computeGradient(double[] q, FramePose3DReadOnly targetPose)
   {
      // Compute numerical gradient since analytical one is not available
      double h = currentHeightMap.getGridResolutionXY();
      double[] gradient = new double[3];
      for (int i = 0; i < 2; i++)
      {
         double[] qPlus = q.clone();
         double[] qMinus = q.clone();
         qPlus[i] += h;
         qMinus[i] -= h;
         gradient[i] = (computeCost(qPlus, targetPose) - computeCost(qMinus, targetPose)) / (2 * h);
      }
      return gradient;
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
