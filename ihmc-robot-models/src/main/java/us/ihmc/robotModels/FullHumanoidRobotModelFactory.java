package us.ihmc.robotModels;

import us.ihmc.robotics.robotSide.RobotSide;

public interface FullHumanoidRobotModelFactory extends FullLeggedRobotModelFactory<RobotSide>
{
   @Override
   FullHumanoidRobotModel createFullRobotModel();

   @Override
   default FullHumanoidRobotModel createFullRobotModel(String namePrefix)
   {
      return createFullRobotModel(namePrefix, true);
   }

   @Override
   FullHumanoidRobotModel createFullRobotModel(String namePrefix, boolean enforceUniqueReferenceFrames);
}
