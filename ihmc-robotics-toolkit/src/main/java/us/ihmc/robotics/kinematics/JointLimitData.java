package us.ihmc.robotics.kinematics;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointLimitData
{
   private double upperPositionLimit = Double.NaN;
   private double lowerPositionLimit = Double.NaN;

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

   public void clear()
   {
      upperPositionLimit = Double.NaN;
      lowerPositionLimit = Double.NaN;

      softUpperPositionLimit = Double.NaN;
      softLowerPositionLimit = Double.NaN;

      velocityLimit = Double.NaN;
      torqueLimit = Double.NaN;

      positionLimitStiffness = Double.NaN;
      positionLimitDamping = Double.NaN;
   }

   public void set(JointLimitData other)
   {
      upperPositionLimit = other.upperPositionLimit;
      lowerPositionLimit = other.lowerPositionLimit;
      softUpperPositionLimit = other.softUpperPositionLimit;
      softLowerPositionLimit = other.softLowerPositionLimit;
      velocityLimit = other.velocityLimit;
      torqueLimit = other.torqueLimit;
      positionLimitStiffness = other.positionLimitStiffness;
      positionLimitDamping = other.positionLimitDamping;
   }

   public void completeWith(JointLimitData other)
   {
      if (!hasUpperLimit())
         upperPositionLimit = other.upperPositionLimit;
      if (!hasLowerLimit())
         lowerPositionLimit = other.lowerPositionLimit;
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

   public boolean hasUpperLimit()
   {
      return !Double.isNaN(upperPositionLimit);
   }

   public boolean hasLowerLimit()
   {
      return !Double.isNaN(lowerPositionLimit);
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

   public double getUpperPositionLimit()
   {
      return upperPositionLimit;
   }

   public double getLowerPositionLimit()
   {
      return lowerPositionLimit;
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

   public void setUpperPositionLimit(double upperPositionLimit)
   {
      this.upperPositionLimit = upperPositionLimit;
   }

   public void setLowerPositionLimit(double lowerPositionLimit)
   {
      this.lowerPositionLimit = lowerPositionLimit;
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
}
