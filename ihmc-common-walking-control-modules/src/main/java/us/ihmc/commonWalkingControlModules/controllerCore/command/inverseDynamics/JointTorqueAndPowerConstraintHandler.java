package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class JointTorqueAndPowerConstraintHandler
{
   private double torqueLimitLower;
   private double torqueLimitUpper;
   private double torqueLimitFromPowerLower;
   private double torqueLimitFromPowerUpper;
   private double qd;

   /**
    * This class handles the computation of the more restrictive constraints on torque given a joint
    * (which has a velocity and may have effort limits), power limits, and a boolean, which dictates
    * whether the joint effort limits are considered. computeTorqueConstraints() must be called at each
    * tick to update the limits.
    */
   public JointTorqueAndPowerConstraintHandler()
   {
      this.torqueLimitLower = Double.NEGATIVE_INFINITY;
      this.torqueLimitUpper = Double.POSITIVE_INFINITY;
      this.torqueLimitFromPowerLower = Double.NEGATIVE_INFINITY;
      this.torqueLimitFromPowerUpper = Double.POSITIVE_INFINITY;
      this.qd = 0.0;
   }

   /**
    * Computes the more restrictive constraints on torque given a joint (which has a velocity and may
    * have effort limits), power limits, and a boolean, which dictates whether the joint effort limits
    * are considered. You must call this each tick to update the torque limits.
    */
   public void computeTorqueConstraints(OneDoFJointBasics joint, double powerLimitLower, double powerLimitUpper, boolean hasTorqueConstraint)
   {
      if (powerLimitLower > powerLimitUpper)
         throw new RuntimeException("powerLimitLower cannot be larger than powerLimitUpper");

      qd = joint.getQd();
      torqueLimitFromPowerLower = powerLimitUpper / qd;
      torqueLimitFromPowerUpper = powerLimitLower / qd;
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
         torqueLimitLower = Math.max(joint.getEffortLimitLower(), torqueLimitFromPowerLower);
         torqueLimitUpper = Math.min(joint.getEffortLimitUpper(), torqueLimitFromPowerUpper);
      }
      else
      {
         torqueLimitLower = torqueLimitFromPowerLower;
         torqueLimitUpper = torqueLimitFromPowerUpper;
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
