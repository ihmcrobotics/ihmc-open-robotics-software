package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Vector2D;

public class SmoothCMPPlannerParameters
{
   private final double modelScale;

   public SmoothCMPPlannerParameters()
   {
      this(1.0);
   }

   public SmoothCMPPlannerParameters(double modelScale)
   {
      this.modelScale = modelScale;
   }

   /**
    * How many footsteps the ICP planner will use to build the plan. The more the better, but will
    * increase the number of YoVariables and increase the computation time. The values 3 and 4 seem
    * to be good.
    */
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   public int getNumberOfWayPointsPerFoot()
   {
      return 2;
   }

   public double[] getMaxCoPForwardOffsetsFootFrame()
   {
      double maxHeelOffset = modelScale * 0.03;
      double maxBallOffset = modelScale * 0.08;

      double[] maxCoPOffsets = { maxHeelOffset, maxBallOffset };

      return maxCoPOffsets;
   }

   public double[] getMinCoPForwardOffsetsFootFrame()
   {
      double minHeelOffset = modelScale * 0.0;
      double minBallOffset = modelScale * -0.04;

      double[] minCoPOffsets = { minHeelOffset, minBallOffset };

      return minCoPOffsets;
   }

   public List<Vector2D> getCoPOffsetsFootFrame()
   {
      Vector2D heelOffset = new Vector2D(0.0, -0.005);
      Vector2D ballOffset = new Vector2D(0.0, 0.025);

      heelOffset.scale(modelScale);
      ballOffset.scale(modelScale);

      List<Vector2D> copOffsets = new ArrayList<>();
      copOffsets.add(heelOffset);
      copOffsets.add(ballOffset);

      return copOffsets;
   }

   public double getTransferDurationAlpha()
   {
      return 0.5;
   }

   public double getSwingDurationAlpha()
   {
      return 0.5;
   }

   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return modelScale * 0.01;
   }

   public double getMaxDurationForSmoothingEntryToExitCoPSwitch()
   {
      return 0.5;
   }

   public double getStepLengthToCoPOffsetFactor()
   {
      return 1.0 / 3.0;
   }

   public double getMinTimeToSpendOnExitCoPInSingleSupport()
   {
      return 0.0;
   }

   /**
    * Provides a list of the alphas that denote the percentage of time taken to transition from one CoP way point to another.
    * Summation of the list must be equal to 1. 
    * @return
    */
   public CoPSplineType getOrderOfCoPInterpolation()
   {
      return CoPSplineType.LINEAR;
   }
}
