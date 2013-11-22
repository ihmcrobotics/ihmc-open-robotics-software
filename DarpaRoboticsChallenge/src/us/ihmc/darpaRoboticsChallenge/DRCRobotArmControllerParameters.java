package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;

public class DRCRobotArmControllerParameters implements ArmControllerParameters
{
   public double getArmJointspaceKp()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 80.0;
      return 80.0; 
   }

   public double getArmJointspaceZeta()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 0.6;
      return 0.2;  // Lots of natural damping in the arms. Don't need to damp the controllers.
   }

   public double getArmJointspaceKi()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 0.0;
      return 0.0; 
   }

   public double getArmJointspaceMaxIntegralError()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 0.0;
      return 0.0; 
   }

   public double getArmJointspaceMaxAcceleration()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return Double.POSITIVE_INFINITY;
      return 10.0;
   }

   public double getArmJointspaceMaxJerk()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return Double.POSITIVE_INFINITY;
      return 100.0;
   }

   public double getArmTaskspaceKp()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 100.0;
      return 100.0; 
   }

   public double getArmTaskspaceZeta()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 1.0;
      return 1.0; 
   }

   public double getArmTaskspaceKi()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 0.0;
      return 0.0; 
   }

   public double getArmTaskspaceMaxIntegralError()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return 0.0;
      return 0.0; 
   }

   public double getArmTaskspaceMaxAcceleration()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return Double.POSITIVE_INFINITY;
      return 10.0; 
   }

   public double getArmTaskspaceMaxJerk()
   {
      if (DRCLocalConfigParameters.USE_VRC_PARAMETERS) return Double.POSITIVE_INFINITY;
      return 100.0; 
   }

}
