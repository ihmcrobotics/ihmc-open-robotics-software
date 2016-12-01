package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class OrderInWhichConstructedStepScorer implements BipedalStepScorer
{
   private double score = 0.0;
   
   @Override
   public double scoreStep(RobotSide swingSide, SideDependentList<RigidBodyTransform> soleTransforms)
   {
      score = score - 1.0;
      return score;
   }

}
