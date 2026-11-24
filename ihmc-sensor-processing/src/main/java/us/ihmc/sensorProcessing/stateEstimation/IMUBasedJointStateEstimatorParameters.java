package us.ihmc.sensorProcessing.stateEstimation;

public class IMUBasedJointStateEstimatorParameters
{
   private final String estimatorName;
   private final boolean enableOutput;
   private final boolean enablePositionOutput;
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
      this(estimatorName, enableOutput, enableOutput, parentIMUName, childIMUName, breakFrequencyForVelocityEstimation, breakFrequencyForPositionEstimation);
   }

   public IMUBasedJointStateEstimatorParameters(String estimatorName,
                                                boolean enableOutput,
                                                boolean enablePositionOutput,
                                                String parentIMUName,
                                                String childIMUName,
                                                double breakFrequencyForVelocityEstimation,
                                                double breakFrequencyForPositionEstimation)
   {
      this.estimatorName = estimatorName;
      this.enableOutput = enableOutput;
      this.enablePositionOutput = enablePositionOutput;
      this.parentIMUName = parentIMUName;
      this.childIMUName = childIMUName;
      this.breakFrequencyForVelocityEstimation = breakFrequencyForVelocityEstimation;
      this.breakFrequencyForPositionEstimation = breakFrequencyForPositionEstimation;
   }

   public String getEstimatorName()
   {
      return estimatorName;
   }

   public boolean isOutputEnabled()
   {
      return enableOutput;
   }

   public boolean isPositionOutputEnabled()
   {
      return enablePositionOutput;
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
