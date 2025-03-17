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

/**
 * This class implements a footstep optimizer for virtual reality (VR) applications.
 * It uses a gradient descent approach to find the best footstep placement based on
 * position, yaw, and planarity costs, while also satisfying constraints on height,
 * slope, and yaw.
 */
public class RDXFootstepOptimizer
{
   private static final double ALPHA = 0.35;
   private static final double BETA = 0.45;
   private static final double GAMMA = 0.2;
   private static final double LEARNING_RATE = 0.1;
   private static final double EPSILON = 1e-3;
   private static final int MAX_ITERATIONS = 100;
   private static final double MAX_HEIGHT_VARIANCE = 0.05;
   private static final double MAX_SLOPE_ANGLE = Math.toRadians(50);

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
   public CompletableFuture<FramePose3D> computeAsync(HeightMapData heightMapData, FramePose3DReadOnly initialPose) {
      // Store the future in a field so it can be cancelled later.
      stopRequested.set(false);
      currentFuture = CompletableFuture.supplyAsync(() -> {
         bestSolution = new FramePose3D(initialPose); // Initialize with initial pose
         return compute(heightMapData, initialPose);
      }, executor);

      return (CompletableFuture<FramePose3D>) currentFuture; // Store and return
   }

   /**
    * Attempts to cancel the current running computation, if any.
    */
   public void cancelCompute() {
      stopRequested.set(true);
      if (currentFuture != null) {
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
      double[] q = {initialPose.getX(), initialPose.getY(), initialPose.getYaw()};
      double[] qBest = q.clone();
      double costBest = Double.POSITIVE_INFINITY;

      for (int i = 0; i < MAX_ITERATIONS; i++)
      {
         // Check if a stop has been requested
         if (stopRequested.get()) {
            LogTools.warn("Optimization interrupted");
            return bestSolution;
         }

         double cost = computeCost(q, initialPose);
         if (cost < costBest)
         {
            costBest = cost;
            System.arraycopy(q, 0, qBest, 0, q.length);

            // Update the best solution so far
            FramePose3D currentBestSolution = new FramePose3D(initialPose);
            currentBestSolution.setX(qBest[0]);
            currentBestSolution.setY(qBest[1]);
            currentBestSolution.getRotation().set(new YawPitchRoll(qBest[2], initialPose.getPitch(), initialPose.getRoll()));
            currentBestSolution.setZ(heightMapData.getHeightAt(qBest[0], qBest[1]));
            bestSolution = currentBestSolution; // Update volatile field
         }

         double[] gradient = computeGradient(q, initialPose);
         for (int j = 0; j < q.length; j++)
         {
            q[j] -= LEARNING_RATE * gradient[j];
         }

         if (!satisfiesConstraints(q, initialPose))
         {
            // If constraints are violated, revert to previous valid state
            System.arraycopy(qBest, 0, q, 0, q.length);
         }

         if (Math.sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1] + gradient[2] * gradient[2]) < EPSILON)
         {
            LogTools.warn("Convergence reached");
            hasConverged = true;
            break; // Convergence reached
         }

         if (i == MAX_ITERATIONS - 1)
         {
            LogTools.warn("Max iterations reached");
            hasConverged = true;
         }
      }

      FramePose3D optimizedPose = new FramePose3D(initialPose);
      optimizedPose.setX(qBest[0]);
      optimizedPose.setY(qBest[1]);
      optimizedPose.getRotation().set(new YawPitchRoll(qBest[2], initialPose.getPitch(), initialPose.getRoll()));
      optimizedPose.setZ(heightMapData.getHeightAt(qBest[0], qBest[1]));
      return optimizedPose;
   }

   /**
    * Computes the cost associated with a given footstep pose.
    * The cost is a combination of position cost, yaw cost, and planarity cost.
    *
    * @param q           The current footstep pose (x, y, yaw).
    * @param targetPose  The target footstep pose.
    * @return The total cost.
    */
   private double computeCost(double[] q, FramePose3DReadOnly targetPose)
   {
      double positionCost = ALPHA * (Math.pow(q[0] - targetPose.getX(), 2) + Math.pow(q[1] - targetPose.getY(), 2));
      double yawCost = BETA * Math.pow(q[2] - targetPose.getYaw(), 2);
      double planarityCost = GAMMA * computePlanarityCost(q);
      return positionCost + yawCost + planarityCost;
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
      double[][] corners = {{q[0] - footLength / 2, q[1] - footWidth / 2},
                            {q[0] + footLength / 2, q[1] - footWidth / 2},
                            {q[0] + footLength / 2, q[1] + footWidth / 2},
                            {q[0] - footLength / 2, q[1] + footWidth / 2},
                            {q[0], q[1]}};  // Center point

      double[] heights = new double[5];
      for (int i = 0; i < 5; i++)
      {
         heights[i] = currentHeightMap.getHeightAt(corners[i][0], corners[i][1]);
      }

      // Fit a plane to the points
      double[] planeCoeffs = fitPlane(corners, heights);

      // Calculate the sum of squared distances from each point to the fitted plane
      double sumSquaredDistances = 0;
      double maxVariance = 0;
      for (int i = 0; i < 5; i++)
      {
         double distance = Math.abs(planeCoeffs[0] * corners[i][0] + planeCoeffs[1] * corners[i][1] + planeCoeffs[2] - heights[i]);
         sumSquaredDistances += distance * distance;
         maxVariance = Math.max(maxVariance, distance);
      }
      qMaxHeightVariance = maxVariance;

      double rmse = Math.sqrt(sumSquaredDistances / 5);  // Root mean square error

      // Calculate the maximum slope of the fitted plane
      double maxSlope = Math.sqrt(planeCoeffs[0] * planeCoeffs[0] + planeCoeffs[1] * planeCoeffs[1]);
      qCurrentMaxSlope = maxSlope;

      return rmse + maxSlope;
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

      return new double[] {a, b, -1, c};  // Ax + By - z + C = 0
   }

   /**
    * Checks if the given footstep pose satisfies all constraints.
    *
    * @param q           The current footstep pose (x, y, yaw).
    * @param initialPose The initial footstep pose.
    * @return True if all constraints are satisfied, false otherwise.
    */
   private boolean satisfiesConstraints(double[] q, FramePose3DReadOnly initialPose)
   {
      return satisfiesPositionConstraint(q, initialPose)
             && satisfiesHeightConstraint(q)
             && satisfiesSlopeConstraint(q);
//             && satisfiesYawConstraint(q, initialPose);
   }

   private boolean satisfiesPositionConstraint(double[] q, FramePose3DReadOnly initialPose)
   {
      double distance = Math.sqrt(Math.pow(q[0] - initialPose.getX(), 2) + Math.pow(q[1] - initialPose.getY(), 2));
      return distance <= 2 * footLength;
   }

   private boolean satisfiesHeightConstraint(double[] q)
   {
      return  qMaxHeightVariance <= MAX_HEIGHT_VARIANCE;
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
    * @param q           The current footstep pose (x, y, yaw).
    * @param targetPose  The target footstep pose.
    * @return The gradient of the cost function.
    */
   private double[] computeGradient(double[] q, FramePose3DReadOnly targetPose)
   {
      // Compute numerical gradient since analytical one is not available
      double h = 1e-5; // Small value for finite difference
      double[] gradient = new double[3];
      for (int i = 0; i < 3; i++)
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
