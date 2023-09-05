package us.ihmc.robotModels;

import us.ihmc.scs2.definition.robot.RobotDefinition;

public interface FullRobotModelFactory
{
   RobotDefinition getRobotDefinition();

   default FullRobotModel createFullRobotModel()
   {
      return createFullRobotModel(true);
   }

   FullRobotModel createFullRobotModel(boolean enforceUniqueReferenceFrames);
}
