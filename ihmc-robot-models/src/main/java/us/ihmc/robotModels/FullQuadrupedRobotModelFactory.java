package us.ihmc.robotModels;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FullQuadrupedRobotModelFactory extends FullLeggedRobotModelFactory<RobotQuadrant>
{
   @Override
   FullQuadrupedRobotModel createFullRobotModel();

   @Override
   default FullQuadrupedRobotModel createFullRobotModel(String namePrefix)
   {
      return createFullRobotModel(namePrefix, true);
   }

   @Override
   FullQuadrupedRobotModel createFullRobotModel(String namePrefix, boolean enforceUniqueReferenceFrames);
}
