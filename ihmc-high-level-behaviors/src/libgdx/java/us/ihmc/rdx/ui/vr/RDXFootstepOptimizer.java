package us.ihmc.rdx.ui.vr;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class RDXFootstepOptimizer
{
   private final double ALPHA = 0.7;
   private final double BETA = 1.0;
   private final double GAMMA = 0.3;
   private final double LEARNING_RATE = 0.1;
   private final double EPSILON = 1e-6;
   private final int MAX_ITERATIONS = 100;
   private final double MAX_HEIGHT_DIFFERENCE = 0.02; // 2 cm
   private final double MAX_SLOPE_ANGLE = Math.toRadians(50); // 15 degrees

   private double footLength;
   private double footWidth;
   private HeightMapData currentHeightMap;

   public RDXFootstepOptimizer(double footLength, double footWidth)
   {
      this.footLength = footLength;
      this.footWidth = footWidth;
   }

   public FramePose3D compute(HeightMapData heightMapData, FramePose3DReadOnly initialPose)
   {
      currentHeightMap = heightMapData;
      double[] q = {initialPose.getX(), initialPose.getY(), initialPose.getYaw()};
      double[] qBest = q.clone();
      double costBest = Double.POSITIVE_INFINITY;

      for (int i = 0; i < MAX_ITERATIONS; i++)
      {
         double cost = computeCost(q, initialPose);
         if (cost < costBest)
         {
            costBest = cost;
            System.arraycopy(q, 0, qBest, 0, q.length);
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
            break; // Convergence reached
         }
      }

      FramePose3D optimizedPose = new FramePose3D(initialPose);
      optimizedPose.setX(qBest[0]);
      optimizedPose.setY(qBest[1]);
      optimizedPose.getRotation().set(new YawPitchRoll(qBest[2], initialPose.getPitch(), initialPose.getRoll()));
      optimizedPose.setZ(heightMapData.getHeightAt(qBest[0], qBest[1]));
      return optimizedPose;
   }

   private boolean satisfiesConstraints(double[] q, FramePose3DReadOnly initialPose)
   {
      return satisfiesPositionConstraint(q, initialPose) &&
             satisfiesHeightConstraint(q) &&
             satisfiesSlopeConstraint(q) &&
             satisfiesYawConstraint(q, initialPose);
   }

   private boolean satisfiesPositionConstraint(double[] q, FramePose3DReadOnly initialPose)
   {
      double distance = Math.sqrt(Math.pow(q[0] - initialPose.getX(), 2) + Math.pow(q[1] - initialPose.getY(), 2));
      return distance <= 2 * footLength;
   }

   private boolean satisfiesHeightConstraint(double[] q)
   {
      double[] heights = sampleHeights(q);
      return computeMaxHeightDifference(heights) <= MAX_HEIGHT_DIFFERENCE;
   }

   private boolean satisfiesSlopeConstraint(double[] q)
   {
      double[] heights = sampleHeights(q);
      return computeMaxSlope(q, heights) <= Math.tan(MAX_SLOPE_ANGLE);
   }

   private boolean satisfiesYawConstraint(double[] q, FramePose3DReadOnly initialPose)
   {
      double yawDifference = q[2] - initialPose.getYaw();
      return yawDifference >= MIN_YAW && yawDifference <= MAX_YAW;
   }

   private double computeCost(double[] q, FramePose3DReadOnly targetPose)
   {
      double positionCost = ALPHA * (Math.pow(q[0] - targetPose.getX(), 2) + Math.pow(q[1] - targetPose.getY(), 2));
      double yawCost = BETA * Math.pow(q[2] - targetPose.getYaw(), 2);
      double planarityCost = GAMMA * computePlanarityCost(q);
      return positionCost + yawCost + planarityCost;
   }

   private double computePlanarityCost(double[] q)
   {
      double[] heights = sampleHeights(q);
      double phiMax = computeMaxHeightDifference(heights);
      double phiVar = computeHeightVariance(heights);
      double phiSlope = computeMaxSlope(q, heights);
      return phiMax + phiVar + phiSlope;
   }

   private double[] sampleHeights(double[] q)
   {
      double[] heights = new double[5];
      double[][] corners = {{-footLength / 2, -footWidth / 2},
                            {footLength / 2, -footWidth / 2},
                            {footLength / 2, footWidth / 2},
                            {-footLength / 2, footWidth / 2},
                            {0, 0}};
      for (int i = 0; i < 5; i++)
      {
         double x = q[0] + corners[i][0] * Math.cos(q[2]) - corners[i][1] * Math.sin(q[2]);
         double y = q[1] + corners[i][0] * Math.sin(q[2]) + corners[i][1] * Math.cos(q[2]);
         heights[i] = currentHeightMap.getHeightAt(x, y);
      }
      return heights;
   }

   private double computeMaxHeightDifference(double[] heights)
   {
      double max = Double.NEGATIVE_INFINITY;
      double min = Double.POSITIVE_INFINITY;
      for (double height : heights)
      {
         max = Math.max(max, height);
         min = Math.min(min, height);
      }
      return max - min;
   }

   private double computeHeightVariance(double[] heights)
   {
      double mean = 0;
      for (double height : heights)
      {
         mean += height;
      }
      mean /= heights.length;

      double variance = 0;
      for (double height : heights)
      {
         variance += Math.pow(height - mean, 2);
      }
      return variance / heights.length;
   }

   private double computeMaxSlope(double[] q, double[] heights)
   {
      double slopeX1 = Math.abs(heights[1] - heights[0]) / footLength;
      double slopeX2 = Math.abs(heights[2] - heights[3]) / footLength;
      double slopeY1 = Math.abs(heights[2] - heights[1]) / footWidth;
      double slopeY2 = Math.abs(heights[3] - heights[0]) / footWidth;
      return Math.max(Math.max(slopeX1, slopeX2), Math.max(slopeY1, slopeY2));
   }

   private double[] computeGradient(double[] q, FramePose3DReadOnly targetPose)
   {
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

   public void reset()
   {

   }
}
