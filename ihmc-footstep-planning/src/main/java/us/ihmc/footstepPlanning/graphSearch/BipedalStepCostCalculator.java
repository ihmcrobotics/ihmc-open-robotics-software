package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.robotics.geometry.FramePose;

public interface BipedalStepCostCalculator
{
   // Returns the cost for taking a step with the swingSide given the soleTransforms.
   public abstract double calculateCost(FramePose stanceFoot, FramePose swingStartFoot, FramePose idealFootstep, FramePose candidateFootstep,
                                        double percentageOfFoothold);
}
