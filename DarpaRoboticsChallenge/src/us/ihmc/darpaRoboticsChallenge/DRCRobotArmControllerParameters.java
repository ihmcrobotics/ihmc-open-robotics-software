package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;

public class DRCRobotArmControllerParameters implements ArmControllerParameters
{
   public double getKpAllArmJoints()
   {
      if (DRCRobotWalkingControllerParameters.USE_VRC_PARAMETERS) return 120.0;
      return 20.0; 
   }

   public double getZetaAllArmJoints()
   {
      return 1.0;
   }

}
