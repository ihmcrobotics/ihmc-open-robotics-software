package us.ihmc.robotModels;

import us.ihmc.robotics.robotDescription.RobotDescription;

public interface FullRobotModelFactory
{
   public abstract RobotDescription getRobotDescription();
   public abstract FullRobotModel createFullRobotModel();
}
