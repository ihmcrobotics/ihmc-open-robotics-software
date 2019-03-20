package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

public class JointLimitParameters
{
   private double maxAbsJointVelocity;               // rad/s
   private double jointLimitDistanceForMaxVelocity;  // rad
   private double jointLimitFilterBreakFrequency;    // 1/s
   private double velocityControlGain;

   public JointLimitParameters()
   {
      maxAbsJointVelocity = Double.POSITIVE_INFINITY;
      jointLimitDistanceForMaxVelocity = 0.0;
      jointLimitFilterBreakFrequency = Double.POSITIVE_INFINITY;
      velocityControlGain = 0.0;
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

   public void set(JointLimitParameters other)
   {
      maxAbsJointVelocity = other.maxAbsJointVelocity;
      jointLimitDistanceForMaxVelocity = other.jointLimitDistanceForMaxVelocity;
      jointLimitFilterBreakFrequency = other.jointLimitFilterBreakFrequency;
      velocityControlGain = other.velocityControlGain;
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
            + ", joint limit filter break frequency: " + jointLimitFilterBreakFrequency + ", velocity control gain: " + velocityControlGain;
   }
}
