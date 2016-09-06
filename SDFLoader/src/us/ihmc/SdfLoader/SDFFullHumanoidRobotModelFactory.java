package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;

public interface SDFFullHumanoidRobotModelFactory extends SDFFullRobotModelFactory
{
   public FullHumanoidRobotModel createFullRobotModel();
}
