package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FootholdRotationParameters
{
   private final DoubleProvider geometricDetectionAngleThreshold;

   private final DoubleProvider copHistoryBreakFrequency;

   private final DoubleProvider angularVelocityAroundLoRThreshold;
   private final DoubleProvider footDropThreshold;
   private final DoubleProvider angularVelocityFilterBreakFrequency;
   private final DoubleProvider stableRotationDirectionThreshold;
   private final DoubleProvider stableRotationPositionThreshold;
   private final IntegerProvider stableEdgeWindowSize;
   private final DoubleProvider perpendicularCoPError;
   private final DoubleProvider distanceFromLineToComputeDesiredCoPOccupancy;
   private final IntegerProvider numberOfDesiredCopsOnCropSide;
   private final DoubleProvider copAreaThreshold;
   private final DoubleProvider inlineCoPHistoryStdDev;
   private final DoubleProvider transverseCoPHistoryStdDev;

   private final DoubleProvider omegaThresholdForEstimation;
   private final DoubleProvider decayBreakFrequency;
   private final DoubleProvider rotationAngleThreshold;
   private final DoubleProvider velocityEdgeFilterBreakFrequency;

   private final IntegerProvider thresholdForCoPRegionOccupancy;
   private final DoubleProvider distanceFromLineOfRotationToComputeCoPOccupancy;

   private final DoubleProvider minimumAreaForCropping;
   private final IntegerProvider shrinkMaxLimit;
   private final DoubleProvider distanceFromRotationToCrop;

   private final BooleanProvider useCoPOccupancyGridForCropping;
   private final DoubleProvider footDropThresholdForCrop;

   private final BooleanParameter doPartialFootholdDetection;
   private final BooleanParameter applyPartialFootholds;

   public FootholdRotationParameters(YoRegistry registry)
   {
      String namePrefix = "Geometric_";
      geometricDetectionAngleThreshold = new DoubleParameter(namePrefix + "AngleThreshold", registry, Math.toRadians(10.0));

      namePrefix = "CoPHistory_";
      copHistoryBreakFrequency = new DoubleParameter(namePrefix + "CoPHistoryBreakFrequency", registry, 100.0);

      namePrefix = "Verification_";
      stableRotationDirectionThreshold = new DoubleParameter(namePrefix + "StableRotationDirectionThreshold", registry, 10.0);
      stableRotationPositionThreshold = new DoubleParameter(namePrefix + "StableRotationPositionThreshold", registry, 0.9);
      stableEdgeWindowSize = new IntegerParameter(namePrefix + "StableEdgeWindowSize", registry, 5);
      perpendicularCoPError = new DoubleParameter(namePrefix + "PerpendicularCoPErrorThreshold", registry, 0.005);
      numberOfDesiredCopsOnCropSide = new IntegerParameter(namePrefix + "NumberOfDesiredCopsOnCropSide", registry, 2);
      distanceFromLineToComputeDesiredCoPOccupancy = new DoubleParameter(namePrefix + "DistanceFromLineToComputeDesiredCoPOccupancy", registry, 0.005);
      copAreaThreshold = new DoubleParameter(namePrefix + "CopHullAreaRatioThreshold", registry, 0.9);
      inlineCoPHistoryStdDev = new DoubleParameter(namePrefix + "InlineCoPHistoryStdDev", registry, 0.015);
      transverseCoPHistoryStdDev = new DoubleParameter(namePrefix + "TransverseCoPHistoryStdDev", registry, 0.002);

      namePrefix = "Kinematic_";
      angularVelocityAroundLoRThreshold = new DoubleParameter(namePrefix + "omegaMagnitudeThreshold", registry, 0.5);
      footDropThreshold = new DoubleParameter(namePrefix + "FootDropThreshold", registry, 0.04);
      angularVelocityFilterBreakFrequency = new DoubleParameter(namePrefix + "omegaFilterBreakFrequency", registry, 16.0);

      namePrefix = "Velocity_";
      omegaThresholdForEstimation = new DoubleParameter(namePrefix + "omegaMagnitudeThreshold", registry, 1.5);
      decayBreakFrequency = new DoubleParameter(namePrefix + "rotationAngleDecayBreakFrequency", registry, 1.0);
      rotationAngleThreshold = new DoubleParameter(namePrefix + "rotationAngleThreshold", registry, 0.05);
      velocityEdgeFilterBreakFrequency = new DoubleParameter(namePrefix + "velocityEdgeFilterBreakFrequency", registry, 5.0);

      namePrefix = "Cropping_";
      thresholdForCoPRegionOccupancy = new IntegerParameter(namePrefix + "ThresholdForCoPRegionOccupancy", registry, 3);
      distanceFromLineOfRotationToComputeCoPOccupancy = new DoubleParameter(namePrefix + "DistanceFromLineOfRotationToComputeCoPOccupancy", registry, 0.001);
      minimumAreaForCropping = new DoubleParameter(namePrefix + "MinimumAreaForCropping", registry, 0.0);
      shrinkMaxLimit = new IntegerParameter(namePrefix + "ShrinkMaxLimit", registry, 1);
      distanceFromRotationToCrop = new DoubleParameter(namePrefix + "DistanceFromRotationToCrop", registry, 0.0);

      useCoPOccupancyGridForCropping = new BooleanParameter(namePrefix + "UseCopOccupancyGrid", registry, true);
      footDropThresholdForCrop = new DoubleParameter(namePrefix + "FootDropThresholdForCrop", registry, 0.01);

      doPartialFootholdDetection = new BooleanParameter("doPartialFootholdDetection", registry, true);
      applyPartialFootholds = new BooleanParameter("applyPartialFootholds", registry, false);
   }

   public DoubleProvider getGeometricDetectionAngleThreshold()
   {
      return geometricDetectionAngleThreshold;
   }

   public DoubleProvider getCopHistoryBreakFrequency()
   {
      return copHistoryBreakFrequency;
   }

   public DoubleProvider getAngularVelocityAroundLoRThreshold()
   {
      return angularVelocityAroundLoRThreshold;
   }

   public DoubleProvider getFootDropThreshold()
   {
      return footDropThreshold;
   }

   public DoubleProvider getAngularVelocityFilterBreakFrequency()
   {
      return angularVelocityFilterBreakFrequency;
   }

   public DoubleProvider getStableRotationDirectionThreshold()
   {
      return stableRotationDirectionThreshold;
   }

   public DoubleProvider getStableRotationPositionThreshold()
   {
      return stableRotationPositionThreshold;
   }

   public IntegerProvider getStableEdgeWindowSize()
   {
      return stableEdgeWindowSize;
   }

   public DoubleProvider getOmegaMagnitudeThresholdForEstimation()
   {
      return omegaThresholdForEstimation;
   }

   public DoubleProvider getVelocityRotationAngleDecayBreakFrequency()
   {
      return decayBreakFrequency;
   }

   public DoubleProvider getVelocityRotationAngleThreshold()
   {
      return rotationAngleThreshold;
   }

   public DoubleProvider getVelocityEdgeFilterBreakFrequency()
   {
      return velocityEdgeFilterBreakFrequency;
   }

   public IntegerProvider getThresholdForCoPRegionOccupancy()
   {
      return thresholdForCoPRegionOccupancy;
   }

   public DoubleProvider getDistanceFromLineOfRotationToComputeCoPOccupancy()
   {
      return distanceFromLineOfRotationToComputeCoPOccupancy;
   }

   public DoubleProvider getMinimumAreaForCropping()
   {
      return minimumAreaForCropping;
   }

   public IntegerProvider getShrinkMaxLimit()
   {
      return shrinkMaxLimit;
   }

   public DoubleProvider getPerpendicularCoPErrorThreshold()
   {
      return perpendicularCoPError;
   }

   public DoubleProvider getDistanceFromLineToComputeDesiredCoPOccupancy()
   {
      return distanceFromLineToComputeDesiredCoPOccupancy;
   }

   public IntegerProvider getNumberOfDesiredCopsOnCropSide()
   {
      return numberOfDesiredCopsOnCropSide;
   }

   public DoubleProvider getDistanceFromRotationToCrop()
   {
      return distanceFromRotationToCrop;
   }

   public DoubleProvider getCopHullAreaRatioThreshold()
   {
      return copAreaThreshold;
   }

   public DoubleProvider getInlineCoPHistoryStdDev()
   {
      return inlineCoPHistoryStdDev;
   }

   public DoubleProvider getTransverseCoPHistoryStdDev()
   {
      return transverseCoPHistoryStdDev;
   }

   public BooleanProvider getUseCoPOccupancyGridForCropping()
   {
      return useCoPOccupancyGridForCropping;
   }

   public DoubleProvider getFootDropThresholdForCrop()
   {
      return footDropThresholdForCrop;
   }

   public BooleanProvider getApplyPartialFootholds()
   {
      return applyPartialFootholds;
   }

   public BooleanProvider getDoPartialFootholdDetection()
   {
      return doPartialFootholdDetection;
   }
}
