package us.ihmc.robotics.kinematics;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointLimitData
{
   private double positionSoftUpperLimit = Double.NaN;
   private double positionSoftLowerLimit = Double.NaN;

   private double velocityLimitUpper = Double.NaN;
   private double velocityLimitLower = Double.NaN;

   private double torqueLimitUpper = Double.NaN;
   private double torqueLimitLower = Double.NaN;

   private double positionLimitStiffness = Double.NaN;
   private double positionLimitDamping = Double.NaN;

   public JointLimitData()
   {
      clear();
   }

   public JointLimitData(OneDoFJoint joint)
   {
      clear();
      setJointLimits(joint);
   }

   public void clear()
   {
      positionSoftUpperLimit = Double.NaN;
      positionSoftLowerLimit = Double.NaN;

      velocityLimitUpper = Double.NaN;
      velocityLimitLower = Double.NaN;

      torqueLimitUpper = Double.NaN;
      torqueLimitLower = Double.NaN;

      positionLimitStiffness = Double.NaN;
      positionLimitDamping = Double.NaN;
   }

   public void set(JointLimitData other)
   {
      positionSoftUpperLimit = other.positionSoftUpperLimit;
      positionSoftLowerLimit = other.positionSoftLowerLimit;

      velocityLimitUpper = other.velocityLimitUpper;
      velocityLimitLower = other.velocityLimitLower;

      torqueLimitUpper = other.torqueLimitUpper;
      torqueLimitLower = other.torqueLimitLower;

      positionLimitStiffness = other.positionLimitStiffness;
      positionLimitDamping = other.positionLimitDamping;
   }

   public void setJointLimits(OneDoFJoint joint)
   {
      positionSoftUpperLimit = joint.getJointLimitUpper();
      positionSoftLowerLimit = joint.getJointLimitLower();

      velocityLimitUpper = joint.getVelocityLimitUpper();
      velocityLimitLower = joint.getVelocityLimitLower();

      torqueLimitUpper = joint.getMaxEffortLimit();
      torqueLimitLower = joint.getMinEffortLimit();
   }

   public void completeWith(JointLimitData other)
   {
      if (!hasPositionSoftUpperLimit())
         positionSoftUpperLimit = other.positionSoftUpperLimit;
      if (!hasPositionSoftLowerLimit())
         positionSoftLowerLimit = other.positionSoftLowerLimit;

      if (!hasVelocityUpperLimit())
         velocityLimitUpper = other.velocityLimitUpper;
      if (!hasVelocityLowerLimit())
         velocityLimitLower = other.velocityLimitLower;

      if (!hasTorqueUpperLimit())
         torqueLimitUpper = other.torqueLimitUpper;
      if (!hasTorqueLowerLimit())
         torqueLimitLower = other.torqueLimitLower;

      if (!hasPositionLimitStiffness())
         positionLimitStiffness = other.positionLimitStiffness;
      if (!hasPositionLimitDamping())
         positionLimitDamping = other.positionLimitDamping;
   }

   public boolean hasPositionSoftUpperLimit()
   {
      return !Double.isNaN(positionSoftUpperLimit);
   }

   public boolean hasPositionSoftLowerLimit()
   {
      return !Double.isNaN(positionSoftLowerLimit);
   }

   public boolean hasVelocityUpperLimit()
   {
      return !Double.isNaN(velocityLimitUpper);
   }

   public boolean hasVelocityLowerLimit()
   {
      return !Double.isNaN(velocityLimitLower);
   }

   public boolean hasTorqueUpperLimit()
   {
      return !Double.isNaN(torqueLimitUpper);
   }

   public boolean hasTorqueLowerLimit()
   {
      return !Double.isNaN(torqueLimitLower);
   }

   public boolean hasPositionLimitStiffness()
   {
      return !Double.isNaN(positionLimitStiffness);
   }

   public boolean hasPositionLimitDamping()
   {
      return !Double.isNaN(positionLimitDamping);
   }

   public double getPositionSoftUpperLimit()
   {
      return positionSoftUpperLimit;
   }

   public double getPositionSoftLowerLimit()
   {
      return positionSoftLowerLimit;
   }

   public double getTorqueUpperLimit()
   {
      return torqueLimitUpper;
   }

   public double getTorqueLowerLimit()
   {
      return torqueLimitLower;
   }

   public double getVelocityUpperLimit()
   {
      return velocityLimitUpper;
   }

   public double getVelocityLowerLimit()
   {
      return velocityLimitLower;
   }

   public double getJointLimitStiffness()
   {
      return positionLimitStiffness;
   }

   public double getJointLimitDamping()
   {
      return positionLimitDamping;
   }

   public void setVelocityUpperLimit(double velocityLimit)
   {
      this.velocityLimitUpper = velocityLimit;
   }

   public void setVelocityLowerLimit(double velocityLimit)
   {
      this.velocityLimitLower = velocityLimit;
   }

   public void setPositionSoftUpperLimit(double positionSoftUpperLimit)
   {
      this.positionSoftUpperLimit = positionSoftUpperLimit;
   }

   public void setPositionSoftLowerLimit(double positionSoftLowerLimit)
   {
      this.positionSoftLowerLimit = positionSoftLowerLimit;
   }

   public void setTorqueUpperLimit(double torqueLimit)
   {
      this.torqueLimitUpper = torqueLimit;
   }

   public void setTorqueLowerLimit(double torqueLimit)
   {
      this.torqueLimitLower = torqueLimit;
   }

   public void setPositionLimitStiffness(double stiffness)
   {
      this.positionLimitStiffness = stiffness;
   }

   public void setPositionLimitDamping(double damping)
   {
      this.positionLimitDamping = damping;
   }
}
