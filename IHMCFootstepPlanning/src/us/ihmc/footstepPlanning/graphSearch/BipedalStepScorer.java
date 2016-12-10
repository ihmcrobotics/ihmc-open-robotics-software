package us.ihmc.footstepPlanning.graphSearch;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.FramePose;

public interface BipedalStepScorer
{
   // Returns the score for taking a step with the swingSide given the soleTransforms.
   public abstract double scoreFootstep(FramePose stanceFoot, FramePose swingStartFoot, FramePose idealFootstep, FramePose candidateFootstep,
                                        Point3d swingFootGoal, double percentageOfFoothold);
}
