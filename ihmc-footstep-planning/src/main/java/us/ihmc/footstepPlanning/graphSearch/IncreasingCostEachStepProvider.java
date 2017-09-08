package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.robotics.geometry.FramePose;

public class IncreasingCostEachStepProvider implements BipedalStepCostCalculator
{
   private double cost = 0.0;

   @Override
   public double calculateCost(FramePose stanceFoot, FramePose swingStartFoot, FramePose idealFootstep, FramePose candidateFootstep,
                               double percentageOfFoothold)
   {
      cost = cost + 1.0;
      return cost;
   }

}
