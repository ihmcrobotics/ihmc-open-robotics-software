package us.ihmc.robotics.kinematics;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointLimitData
{
   private double softUpperPositionLimit = Double.NaN;
   private double softLowerPositionLimit = Double.NaN;

   private double velocityLimit = Double.NaN;
   private double torqueLimit = Double.NaN;

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
      softUpperPositionLimit = Double.NaN;
      softLowerPositionLimit = Double.NaN;

      velocityLimit = Double.NaN;
      torqueLimit = Double.NaN;

      positionLimitStiffness = Double.NaN;
      positionLimitDamping = Double.NaN;
   }

   public void set(JointLimitData other)
   {
      softUpperPositionLimit = other.softUpperPositionLimit;
      softLowerPositionLimit = other.softLowerPositionLimit;

      velocityLimit = other.velocityLimit;
      torqueLimit = other.torqueLimit;

      positionLimitStiffness = other.positionLimitStiffness;
      positionLimitDamping = other.positionLimitDamping;
   }

   public void setJointLimits(OneDoFJoint joint)
   {
      softUpperPositionLimit = joint.getJointLimitUpper();
      softLowerPositionLimit = joint.getJointLimitLower();

      velocityLimit = joint.getVelocityLimit();
      torqueLimit = joint.getEffortLimit();
   }

   public void completeWith(JointLimitData other)
   {
      if (!hasSoftUpperLimit())
         softUpperPositionLimit = other.softUpperPositionLimit;
      if (!hasSoftLowerLimit())
         softLowerPositionLimit = other.softLowerPositionLimit;

      if (!hasVelocityLimit())
         velocityLimit = other.velocityLimit;

      if (!hasTorqueLimit())
         torqueLimit = other.torqueLimit;

      if (!hasPositionLimitStiffness())
         positionLimitStiffness = other.positionLimitStiffness;
      if (!hasPositionLimitDamping())
         positionLimitDamping = other.positionLimitDamping;
   }

   public boolean hasSoftUpperLimit()
   {
      return !Double.isNaN(softUpperPositionLimit);
   }

   public boolean hasSoftLowerLimit()
   {
      return !Double.isNaN(softLowerPositionLimit);
   }

   public boolean hasVelocityLimit()
   {
      return !Double.isNaN(velocityLimit);
   }

   public boolean hasTorqueLimit()
   {
      return !Double.isNaN(torqueLimit);
   }

   public boolean hasPositionLimitStiffness()
   {
      return !Double.isNaN(positionLimitStiffness);
   }

   public boolean hasPositionLimitDamping()
   {
      return !Double.isNaN(positionLimitDamping);
   }

   public double getSoftUpperPositionLimit()
   {
      return softUpperPositionLimit;
   }

   public double getSoftLowerPositionLimit()
   {
      return softLowerPositionLimit;
   }

   public double getTorqueLimit()
   {
      return torqueLimit;
   }

   public double getVelocityLimit()
   {
      return velocityLimit;
   }

   public double getJointLimitStiffness()
   {
      return positionLimitStiffness;
   }

   public double getJointLimitDamping()
   {
      return positionLimitDamping;
   }

   public void setVelocityLimit(double velocityLimit)
   {
      this.velocityLimit = velocityLimit;
   }

   public void setSoftUpperPositionLimit(double softUpperPositionLimit)
   {
      this.softUpperPositionLimit = softUpperPositionLimit;
   }

   public void setSoftLowerPositionLimit(double softLowerPositionLimit)
   {
      this.softLowerPositionLimit = softLowerPositionLimit;
   }

   public void setTorqueLimit(double torqueLimit)
   {
      this.torqueLimit = torqueLimit;
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
