package us.ihmc.SdfLoader;

import us.ihmc.robotics.humanoidRobot.model.FullRobotModelFactory;

public interface SDFFullRobotModelFactory extends FullRobotModelFactory
{
   public SDFFullRobotModel createFullRobotModel();
}
