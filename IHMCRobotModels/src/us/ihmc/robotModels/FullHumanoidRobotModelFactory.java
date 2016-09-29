package us.ihmc.robotModels;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModelFactory;

public interface FullHumanoidRobotModelFactory extends FullRobotModelFactory
{
   @Override
   public FullHumanoidRobotModel createFullRobotModel();
}
