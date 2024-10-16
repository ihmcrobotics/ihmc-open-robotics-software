package us.ihmc.robotModels;

import us.ihmc.commons.robotics.robotSide.RobotSide;

public interface FullHumanoidRobotModelFactory extends FullLeggedRobotModelFactory<RobotSide>
{
   @Override
   default FullHumanoidRobotModel createFullRobotModel()
   {
      return createFullRobotModel(true);
   }

   @Override
   FullHumanoidRobotModel createFullRobotModel(boolean enforceUniqueReferenceFrames);
}
