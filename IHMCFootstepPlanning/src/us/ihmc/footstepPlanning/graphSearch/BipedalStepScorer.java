package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface BipedalStepScorer
{

   // Returns the score for taking a step with the swingSide given the soleTransforms.
   public abstract double scoreStep(RobotSide swingSide, SideDependentList<RigidBodyTransform> soleTransforms);

}
