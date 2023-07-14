package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class JointTorqueAndPowerConstraintHandler
{
   private double torqueLimitLower;
   private double torqueLimitUpper;

   public JointTorqueAndPowerConstraintHandler(OneDoFJointBasics joint, double powerLimitLower, double powerLimitUpper, boolean hasTorqueConstraint)
   {
      if (powerLimitLower > powerLimitUpper)
         throw new RuntimeException("powerLimitLower cannot be larger than powerLimitUpper");
      
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

      if (hasTorqueConstraint)
      {
         this.torqueLimitLower = Math.max(joint.getEffortLimitLower(), torqueLimitFromPowerLower);
         this.torqueLimitUpper = Math.min(joint.getEffortLimitUpper(), torqueLimitFromPowerUpper);
      }
      else
      {
         this.torqueLimitLower = torqueLimitFromPowerLower;
         this.torqueLimitUpper = torqueLimitFromPowerUpper;
      }
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
