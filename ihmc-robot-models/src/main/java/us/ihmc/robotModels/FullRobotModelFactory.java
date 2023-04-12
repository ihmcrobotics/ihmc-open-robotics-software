package us.ihmc.robotModels;

import us.ihmc.scs2.definition.robot.RobotDefinition;

public interface FullRobotModelFactory
{
   RobotDefinition getRobotDefinition();

   FullRobotModel createFullRobotModel();

   default FullRobotModel createFullRobotModel(String namePrefix)
   {
      return createFullRobotModel(namePrefix, true);
   }

   FullRobotModel createFullRobotModel(String namePrefix, boolean enforceUniqueReferenceFrames);
}
