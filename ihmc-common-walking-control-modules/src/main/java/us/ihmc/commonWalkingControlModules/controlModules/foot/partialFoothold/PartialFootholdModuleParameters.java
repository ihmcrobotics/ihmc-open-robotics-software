package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

public class PartialFootholdModuleParameters
{
   private final double geometricDetectionAngleThreshold;

   private final double copHistoryBreakFrequency;

   private final double angularVelocityAroundLoRThreshold;
   private final double footDropThreshold;
   private final double angularVelocityFilterBreakFrequency;
   private final double stableRotationDirectionThreshold;
   private final double stableRotationPositionThreshold;
   private final int stableEdgeWindowSize;
   private final double perpendicularCoPError;
   private final double distanceFromLineToComputeDesiredCoPOccupancy;
   private final int numberOfDesiredCopsOnCropSide;
   private final double copAreaThreshold;
   private final double inlineCoPHistoryStdDev;
   private final double transverseCoPHistoryStdDev;

   private final double omegaThresholdForEstimation;
   private final double decayBreakFrequency;
   private final double rotationAngleThreshold;
   private final double velocityEdgeFilterBreakFrequency;

   private final int thresholdForCoPRegionOccupancy;
   private final double distanceFromLineOfRotationToComputeCoPOccupancy;

   private final double minimumAreaForCropping;
   private final int shrinkMaxLimit;
   private final double distanceFromRotationToCrop;

   private final boolean useCoPOccupancyGridForCropping;
   private final double footDropThresholdForCrop;

   private final boolean createPartialFootholdModule;
   private final boolean doPartialFootholdDetection;
   private final boolean applyPartialFootholds;

   public PartialFootholdModuleParameters()
   {
      this(false);
   }

   public PartialFootholdModuleParameters(boolean createPartialFootholdModule)
   {
      geometricDetectionAngleThreshold = Math.toRadians(10.0);

      copHistoryBreakFrequency = 100.0;

      stableRotationDirectionThreshold = 10.0;
      stableRotationPositionThreshold = 0.9;
      stableEdgeWindowSize = 5;
      perpendicularCoPError = 0.005;
      numberOfDesiredCopsOnCropSide = 2;
      distanceFromLineToComputeDesiredCoPOccupancy = 0.005;
      copAreaThreshold = 0.9;
      inlineCoPHistoryStdDev = 0.015;
      transverseCoPHistoryStdDev = 0.002;

      angularVelocityAroundLoRThreshold = 0.5;
      footDropThreshold = 0.04;
      angularVelocityFilterBreakFrequency = 16.0;

      omegaThresholdForEstimation = 1.5;
      decayBreakFrequency = 1.0;
      rotationAngleThreshold = 0.05;
      velocityEdgeFilterBreakFrequency = 5.0;

      thresholdForCoPRegionOccupancy = 3;
      distanceFromLineOfRotationToComputeCoPOccupancy = 0.001;
      minimumAreaForCropping = 0.0;
      shrinkMaxLimit = 1;
      distanceFromRotationToCrop = 0.0;

      useCoPOccupancyGridForCropping = true;
      footDropThresholdForCrop = 0.01;

      this.createPartialFootholdModule = createPartialFootholdModule;
      doPartialFootholdDetection = false;
      applyPartialFootholds = false;
   }

   public double getGeometricDetectionAngleThreshold()
   {
      return geometricDetectionAngleThreshold;
   }

   public double getCopHistoryBreakFrequency()
   {
      return copHistoryBreakFrequency;
   }

   public double getAngularVelocityAroundLoRThreshold()
   {
      return angularVelocityAroundLoRThreshold;
   }

   public double getFootDropThreshold()
   {
      return footDropThreshold;
   }

   public double getAngularVelocityFilterBreakFrequency()
   {
      return angularVelocityFilterBreakFrequency;
   }

   public double getStableRotationDirectionThreshold()
   {
      return stableRotationDirectionThreshold;
   }

   public double getStableRotationPositionThreshold()
   {
      return stableRotationPositionThreshold;
   }

   public int getStableEdgeWindowSize()
   {
      return stableEdgeWindowSize;
   }

   public double getOmegaMagnitudeThresholdForEstimation()
   {
      return omegaThresholdForEstimation;
   }

   public double getVelocityRotationAngleDecayBreakFrequency()
   {
      return decayBreakFrequency;
   }

   public double getVelocityRotationAngleThreshold()
   {
      return rotationAngleThreshold;
   }

   public double getVelocityEdgeFilterBreakFrequency()
   {
      return velocityEdgeFilterBreakFrequency;
   }

   public int getThresholdForCoPRegionOccupancy()
   {
      return thresholdForCoPRegionOccupancy;
   }

   public double getDistanceFromLineOfRotationToComputeCoPOccupancy()
   {
      return distanceFromLineOfRotationToComputeCoPOccupancy;
   }

   public double getMinimumAreaForCropping()
   {
      return minimumAreaForCropping;
   }

   public int getShrinkMaxLimit()
   {
      return shrinkMaxLimit;
   }

   public double getPerpendicularCoPErrorThreshold()
   {
      return perpendicularCoPError;
   }

   public double getDistanceFromLineToComputeDesiredCoPOccupancy()
   {
      return distanceFromLineToComputeDesiredCoPOccupancy;
   }

   public int getNumberOfDesiredCopsOnCropSide()
   {
      return numberOfDesiredCopsOnCropSide;
   }

   public double getDistanceFromRotationToCrop()
   {
      return distanceFromRotationToCrop;
   }

   public double getCopHullAreaRatioThreshold()
   {
      return copAreaThreshold;
   }

   public double getInlineCoPHistoryStdDev()
   {
      return inlineCoPHistoryStdDev;
   }

   public double getTransverseCoPHistoryStdDev()
   {
      return transverseCoPHistoryStdDev;
   }

   public boolean getUseCoPOccupancyGridForCropping()
   {
      return useCoPOccupancyGridForCropping;
   }

   public double getFootDropThresholdForCrop()
   {
      return footDropThresholdForCrop;
   }

   public boolean createPartialFootholdModule()
   {
      return createPartialFootholdModule;
   }

   public boolean getApplyPartialFootholds()
   {
      return applyPartialFootholds;
   }

   public boolean getDoPartialFootholdDetection()
   {
      return doPartialFootholdDetection;
   }
}
