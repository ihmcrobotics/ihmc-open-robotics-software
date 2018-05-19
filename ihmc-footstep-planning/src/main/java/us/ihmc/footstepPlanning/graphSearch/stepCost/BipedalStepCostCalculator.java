package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.referenceFrame.FramePose3D;

public interface BipedalStepCostCalculator
{
   // Returns the cost for taking a step with the swingSide given the soleTransforms.
   public abstract double calculateCost(FramePose3D stanceFoot, FramePose3D swingStartFoot, FramePose3D idealFootstep, FramePose3D candidateFootstep,
                                        double percentageOfFoothold);
}
