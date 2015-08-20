package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.models.FullRobotModelFactory;

public interface SDFFullRobotModelFactory extends FullRobotModelFactory
{
   public SDFFullHumanoidRobotModel createFullRobotModel();
}
