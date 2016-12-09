package us.ihmc.footstepPlanning.graphSearch;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.FramePose;

public class OrderInWhichConstructedStepScorer implements BipedalStepScorer
{
   private double score = 0.0;
   
   @Override
   public double scoreFootstep(FramePose stanceFoot, FramePose swingStartFoot, FramePose idealFootstep, FramePose candidateFootstep, Point3d goal, double percentageOfFoothold)
   {
      score = score - 1.0;
      return score;
   }

}
