package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class JointTorqueAndPowerConstraintHandler
{
   private double torqueLimitLower;
   private double torqueLimitUpper;
   private double torqueLimitFromPowerLower;
   private double torqueLimitFromPowerUpper;

   /**
    * Computes the more restrictive constraints on torque given a joint (which has a velocity and may
    * have effort limits), power limits, and a boolean, which dictates whether the joint effort limits
    * are considered.
    */
   public JointTorqueAndPowerConstraintHandler(OneDoFJointBasics joint, double powerLimitLower, double powerLimitUpper, boolean hasTorqueConstraint)
   {
      if (powerLimitLower > powerLimitUpper)
         throw new RuntimeException("powerLimitLower cannot be larger than powerLimitUpper");

      double qd = joint.getQd();
      if (qd < 0)
      {
         this.torqueLimitFromPowerLower = powerLimitUpper / qd;
         this.torqueLimitFromPowerUpper = powerLimitLower / qd;
      }
      else
      {
         this.torqueLimitFromPowerLower = powerLimitLower / qd;
         this.torqueLimitFromPowerUpper = powerLimitUpper / qd;
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

   /**
    * Gets the least upper bound on torque
    */
   public double getTorqueLimitLower()
   {
      return torqueLimitLower;
   }

   /**
    * Gets the greatest lower bound on torque
    */
   public double getTorqueLimitUpper()
   {
      return torqueLimitUpper;
   }

   /**
    * This method only gets the torque limit relative to the power limit and is intended ONLY for
    * testing and debugging.
    */
   public double getTorqueLimitsFromPowerLower()
   {
      return torqueLimitFromPowerLower;
   }

   /**
    * This method only gets the torque limit relative to the power limit and is intended ONLY for
    * testing and debugging.
    */
   public double getTorqueLimitsFromPowerUpper()
   {
      return torqueLimitFromPowerUpper;
   }
}
