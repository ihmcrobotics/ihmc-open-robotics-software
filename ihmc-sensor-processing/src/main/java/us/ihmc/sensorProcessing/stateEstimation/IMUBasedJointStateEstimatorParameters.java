package us.ihmc.sensorProcessing.stateEstimation;

public class IMUBasedJointStateEstimatorParameters
{
   private final String estimatorName;
   private final boolean enableOutput;
   private final String parentIMUName, childIMUName;
   private final double breakFrequencyForVelocityEstimation;
   private final double breakFrequencyForPositionEstimation;

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
}
