package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.models.FullQuadrupedRobotModel;

public interface SDFFullQuadrupedRobotModelFactory extends SDFFullRobotModelFactory
{
   @Override
   public FullQuadrupedRobotModel createFullRobotModel();

}
