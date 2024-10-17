package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;

public class KinematicsToolboxPrivilegedConfigurationParameters extends JointPrivilegedConfigurationParameters
{
   @Override
   public double getNullspaceProjectionAlpha()
   {
      return 0.005;
   }

   @Override
   public double getDefaultWeight()
   {
      return 0.025;
   }

   @Override
   public double getDefaultConfigurationGain()
   {
      return 40.0;
   }

   @Override
   public double getDefaultMaxVelocity()
   {
      return Double.POSITIVE_INFINITY;
   }
}
