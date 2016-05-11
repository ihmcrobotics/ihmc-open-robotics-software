package us.ihmc.quadrupedRobotics.model;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFRobot;

public abstract class QuadrupedModelFactory
{
   public abstract SDFRobot createSdfRobot();
   public abstract SDFFullQuadrupedRobotModel createFullRobotModel();
}
