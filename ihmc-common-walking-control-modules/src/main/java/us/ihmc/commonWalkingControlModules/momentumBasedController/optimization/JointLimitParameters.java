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
}
