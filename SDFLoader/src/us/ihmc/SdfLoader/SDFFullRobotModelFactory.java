package us.ihmc.SdfLoader;

import us.ihmc.humanoidRobotics.model.FullRobotModelFactory;

public interface SDFFullRobotModelFactory extends FullRobotModelFactory
{
   public SDFFullRobotModel createFullRobotModel();
}
