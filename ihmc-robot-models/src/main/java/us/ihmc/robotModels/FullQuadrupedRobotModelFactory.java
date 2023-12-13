package us.ihmc.robotModels;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface FullQuadrupedRobotModelFactory extends FullLeggedRobotModelFactory<RobotQuadrant>
{
   @Override
   default FullQuadrupedRobotModel createFullRobotModel()
   {
      return createFullRobotModel(true);
   }

   @Override
   FullQuadrupedRobotModel createFullRobotModel(boolean enforceUniqueReferenceFrames);
}
