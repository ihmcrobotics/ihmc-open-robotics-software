package us.ihmc.SdfLoader;

public interface SDFFullQuadrupedRobotModelFactory extends SDFFullRobotModelFactory
{
   @Override
   public SDFFullQuadrupedRobotModel createFullRobotModel();
   
   @Override
   public GeneralizedSDFRobotModel getGeneralizedRobotModel();
}
