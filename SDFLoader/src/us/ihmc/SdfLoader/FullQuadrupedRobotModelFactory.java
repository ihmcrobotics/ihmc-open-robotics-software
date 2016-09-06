package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.models.FullQuadrupedRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModelFactory;

public interface FullQuadrupedRobotModelFactory extends FullRobotModelFactory
{
   @Override
   public FullQuadrupedRobotModel createFullRobotModel();

}
