package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

public class JointLimitParameters
{
   private double maxAbsJointVelocity;               // rad/s
   private double jointLimitDistanceForMaxVelocity;  // rad
   private double jointLimitFilterBreakFrequency;    // 1/s
   private double velocityControlGain;

   private double rangeOfMotionMarginFraction;  // should not exceed 0.5
   private double velocityDeadbandSize;
   
   private double jointLowerLimit;
   private double jointUpperLimit;

   public JointLimitParameters()
   {
      maxAbsJointVelocity = Double.POSITIVE_INFINITY;
      jointLimitDistanceForMaxVelocity = 0.0;
      jointLimitFilterBreakFrequency = Double.POSITIVE_INFINITY;
      velocityControlGain = 0.0;
      rangeOfMotionMarginFraction = 0.0;
      velocityDeadbandSize = 0.0;

      jointLowerLimit = Double.NEGATIVE_INFINITY;
      jointUpperLimit = Double.POSITIVE_INFINITY;
   }


   public double getJointLowerLimit()
   {
      return jointLowerLimit;
   }
   public double getJointUpperLimit()
   {
      return jointUpperLimit;
   }
   public void setJointLowerLimit(double limit)
   {
      this.jointLowerLimit = limit;
   }
   public void setJointUpperLimit(double limit)
   {
      this.jointUpperLimit = limit;
   }
   
   public double getMaxAbsJointVelocity()
   {
      return maxAbsJointVelocity;
   }

   public void setMaxAbsJointVelocity(double maxAbsJointVelocity)
   {
      this.maxAbsJointVelocity = maxAbsJointVelocity;
   }

   public double getJointLimitDistanceForMaxVelocity()
   {
      return jointLimitDistanceForMaxVelocity;
   }

   public void setJointLimitDistanceForMaxVelocity(double jointLimitDistanceForMaxVelocity)
   {
      this.jointLimitDistanceForMaxVelocity = jointLimitDistanceForMaxVelocity;
   }

   public double getJointLimitFilterBreakFrequency()
   {
      return jointLimitFilterBreakFrequency;
   }

   public void setJointLimitFilterBreakFrequency(double jointLimitFilterBreakFrequency)
   {
      this.jointLimitFilterBreakFrequency = jointLimitFilterBreakFrequency;
   }

   public double getVelocityControlGain()
   {
      return velocityControlGain;
   }

   public void setVelocityControlGain(double velocityControlGain)
   {
      this.velocityControlGain = velocityControlGain;
   }

   public double getRangeOfMotionMarginFraction()
   {
      return rangeOfMotionMarginFraction;
   }

   public void setRangeOfMotionMarginFraction(double rangeOfMotionMarginFraction)
   {
      this.rangeOfMotionMarginFraction = rangeOfMotionMarginFraction;
   }

   public double getVelocityDeadbandSize()
   {
      return velocityDeadbandSize;
   }

   public void setVelocityDeadbandSize(double velocityDeadbandSize)
   {
      this.velocityDeadbandSize = velocityDeadbandSize;
   }


   public void set(JointLimitParameters other)
   {
      maxAbsJointVelocity = other.maxAbsJointVelocity;
      jointLimitDistanceForMaxVelocity = other.jointLimitDistanceForMaxVelocity;
      jointLimitFilterBreakFrequency = other.jointLimitFilterBreakFrequency;
      velocityControlGain = other.velocityControlGain;
      setRangeOfMotionMarginFraction(other.getRangeOfMotionMarginFraction());
      setVelocityDeadbandSize(other.getVelocityDeadbandSize());
      setJointLowerLimit(other.getJointLowerLimit());
      setJointUpperLimit(other.getJointUpperLimit());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof JointLimitParameters)
      {
         JointLimitParameters other = (JointLimitParameters) object;

         if (maxAbsJointVelocity != other.maxAbsJointVelocity)
            return false;
         if (jointLimitDistanceForMaxVelocity != other.jointLimitDistanceForMaxVelocity)
            return false;
         if (jointLimitFilterBreakFrequency != other.jointLimitFilterBreakFrequency)
            return false;
         if (velocityControlGain != other.velocityControlGain)
            return false;
         if (rangeOfMotionMarginFraction != other.rangeOfMotionMarginFraction)
            return false;
         if (velocityDeadbandSize != other.velocityDeadbandSize)
            return false;
         if (jointLowerLimit != other.jointLowerLimit)
            return false;
         if (jointUpperLimit != other.jointUpperLimit)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": qd max: " + maxAbsJointVelocity + ", joint limit distance for qd max: " + jointLimitDistanceForMaxVelocity
            + ", joint limit filter break frequency: " + jointLimitFilterBreakFrequency + ", velocity control gain: " + velocityControlGain
            + ", range of motion margin fraction: " + rangeOfMotionMarginFraction +  ", velocity deadband size: " + velocityDeadbandSize;
   }
}
