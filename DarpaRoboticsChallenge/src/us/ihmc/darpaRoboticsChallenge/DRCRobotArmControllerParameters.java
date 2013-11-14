package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;

public class DRCRobotArmControllerParameters implements ArmControllerParameters
{
   public double getKpAllArmJoints()
   {
      if (DRCConfigParameters.USE_VRC_PARAMETERS) return 120.0;
      return 20.0; 
   }

   public double getZetaAllArmJoints()
   {
      return 0.8; //1.0;
   }

   public double getMaxAccelerationAllArmJoints()
   {
      if (DRCConfigParameters.USE_VRC_PARAMETERS) return 100.0;
      return 6.0;
   }

   public double getMaxJerkAllArmJoints()
   {
      if (DRCConfigParameters.USE_VRC_PARAMETERS) return 1000.0;
      return 60.0;
   }

}
