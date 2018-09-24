package us.ihmc.robotModels;

import us.ihmc.robotics.robotSide.RobotSide;

public interface FullHumanoidRobotModelFactory extends FullLeggedRobotModelFactory<RobotSide>
{
   @Override
   FullHumanoidRobotModel createFullRobotModel();
}
