package us.ihmc.robotModels;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FullQuadrupedRobotModelFactory extends FullLeggedRobotModelFactory<RobotQuadrant>
{
   @Override
   FullQuadrupedRobotModel createFullRobotModel();
}
