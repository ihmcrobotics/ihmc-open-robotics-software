package us.ihmc.sensorProcessing.stateEstimation;

public class IMUBasedJointStateEstimatorParameters
{
   private final String estimatorName;
   private final boolean enableOutput;
   private final String parentIMUName, childIMUName;
   private final double breakFrequencyForVelocityEstimation;
   private final double breakFrequencyForPositionEstimation;
   private double velocityEstimationBacklashSlopTime = 0.0;

   public IMUBasedJointStateEstimatorParameters(String estimatorName,
                                                boolean enableOutput,
                                                String parentIMUName,
                                                String childIMUName,
                                                double breakFrequencyForVelocityEstimation,
                                                double breakFrequencyForPositionEstimation)
   {
      this.estimatorName = estimatorName;
      this.enableOutput = enableOutput;
      this.parentIMUName = parentIMUName;
      this.childIMUName = childIMUName;
      this.breakFrequencyForVelocityEstimation = breakFrequencyForVelocityEstimation;
      this.breakFrequencyForPositionEstimation = breakFrequencyForPositionEstimation;
   }

   public void setVelocityEstimationBacklashSlopTime(double velocityEstimationBacklashSlopTime)
   {
      this.velocityEstimationBacklashSlopTime = velocityEstimationBacklashSlopTime;
   }

   public String getEstimatorName()
   {
      return estimatorName;
   }

   public boolean isOuputEnabled()
   {
      return enableOutput;
   }

   public String getParentIMUName()
   {
      return parentIMUName;
   }

   public String getChildIMUName()
   {
      return childIMUName;
   }

   public double getBreakFrequencyForVelocityEstimation()
   {
      return breakFrequencyForVelocityEstimation;
   }

   public double getBreakFrequencyForPositionEstimation()
   {
      return breakFrequencyForPositionEstimation;
   }

   public double getVelocityEstimationBacklashSlopTime()
   {
      return velocityEstimationBacklashSlopTime;
   }
}
