package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class JointTorqueAndPowerConstraintHandler
{
   private double torqueLimitLower;
   private double torqueLimitUpper;

   public JointTorqueAndPowerConstraintHandler(OneDoFJointBasics joint, double powerLimitLower, double powerLimitUpper)
   {
      double qd = joint.getQd();
      double torqueLimitFromPowerLower;
      double torqueLimitFromPowerUpper;
      if (qd < 0)
      {
         torqueLimitFromPowerLower = powerLimitUpper / qd;
         torqueLimitFromPowerUpper = powerLimitLower / qd;
      }
      else
      {
         torqueLimitFromPowerLower = powerLimitLower / qd;
         torqueLimitFromPowerUpper = powerLimitUpper / qd;
      }
      this.torqueLimitLower = Math.max(joint.getEffortLimitLower(), torqueLimitFromPowerLower);
      this.torqueLimitUpper = Math.min(joint.getEffortLimitUpper(), torqueLimitFromPowerUpper);

   }

   public double getTorqueLimitLower()
   {
      return torqueLimitLower;
   }

   public double getTorqueLimitUpper()
   {
      return torqueLimitUpper;
   }
}
