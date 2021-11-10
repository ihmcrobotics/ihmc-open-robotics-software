package us.ihmc.robotModels;

import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public interface FullRobotModelFactory
{
   RobotDescription getRobotDescription();

   default RobotDefinition getRobotDefinition()
   {
      return null;
   }

   FullRobotModel createFullRobotModel();
}
