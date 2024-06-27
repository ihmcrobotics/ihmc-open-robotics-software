package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;

public class KinematicsToolboxPrivilegedConfigurationParameters extends JointPrivilegedConfigurationParameters
{
   @Override
   public double getNullspaceProjectionAlpha()
   {
      return 0.05;
   }

   @Override
   public double getDefaultWeight()
   {
      return 2.0;
   }
}
