package us.ihmc.SdfLoader.models;

import us.ihmc.robotics.robotDescription.RobotDescription;

public interface FullRobotModelFactory
{
   public abstract RobotDescription getRobotDescription();
   public abstract FullRobotModel createFullRobotModel();
}
