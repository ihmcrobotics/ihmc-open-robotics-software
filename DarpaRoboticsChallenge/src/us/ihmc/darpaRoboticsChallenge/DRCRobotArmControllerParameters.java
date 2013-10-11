package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;

public class DRCRobotArmControllerParameters implements ArmControllerParameters
{
   public double getKpAllArmJoints()
   {
      return 120.0;
   }

   public double getZetaAllArmJoints()
   {
      return 1.0;
   }

}
