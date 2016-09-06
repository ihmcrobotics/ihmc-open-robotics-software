package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModelFactory;

public interface SDFFullRobotModelFactory extends FullRobotModelFactory
{
   @Override
   public FullRobotModel createFullRobotModel();

   public abstract GeneralizedSDFRobotModel getGeneralizedRobotModel();
}
