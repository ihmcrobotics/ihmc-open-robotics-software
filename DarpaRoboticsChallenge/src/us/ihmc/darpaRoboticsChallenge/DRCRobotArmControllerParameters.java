package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;

public class DRCRobotArmControllerParameters implements ArmControllerParameters
{
   public double getKpAllArmJoints()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 80.0;
      return 80.0; 
   }

   public double getZetaAllArmJoints()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 0.6;
      return 0.2;  // Lots of natural damping in the arms. Don't need to damp the controllers.
   }

   public double getMaxAccelerationAllArmJoints()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 100.0;
      return 6.0;
   }

   public double getMaxJerkAllArmJoints()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 1000.0;
      return 60.0;
   }

}
