package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.models.FullRobotModelFactory;

public interface SDFFullRobotModelFactory extends FullRobotModelFactory
{
   @Override
   public SDFFullRobotModel createFullRobotModel();

   public abstract GeneralizedSDFRobotModel getGeneralizedRobotModel();
}
