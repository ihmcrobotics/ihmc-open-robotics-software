package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoPartialFootholdModuleParameters
{
   private final DoubleProvider geometricDetectionAngleThreshold;

   private final DoubleProvider copHistoryBreakFrequency;

   private final DoubleProvider stableRotationDirectionThreshold;
   private final DoubleProvider stableRotationPositionThreshold;
   private final IntegerProvider stableEdgeWindowSize;

   private final DoubleProvider angularVelocityAroundLoRThreshold;
   private final DoubleProvider footDropThreshold;
   private final DoubleProvider angularVelocityFilterBreakFrequency;

   private final DoubleProvider omegaThresholdForEstimation;
   private final DoubleProvider decayBreakFrequency;
   private final DoubleProvider rotationAngleThreshold;
   private final DoubleProvider velocityEdgeFilterBreakFrequency;

   private final DoubleProvider inlineCoPHistoryStdDev;
   private final DoubleProvider transverseCoPHistoryStdDev;

   private final boolean createPartialFootholdModule;
   private final FootholdCroppingParameters footholdCroppingParameters;

   public YoPartialFootholdModuleParameters(PartialFootholdModuleParameters parameters, YoRegistry registry)
   {
      String namePrefix = "Geometric_";
      geometricDetectionAngleThreshold = new DoubleParameter(namePrefix + "AngleThreshold", registry, parameters.getGeometricDetectionAngleThreshold());

      namePrefix = "CoPHistory_";
      copHistoryBreakFrequency = new DoubleParameter(namePrefix + "CoPHistoryBreakFrequency", registry, parameters.getCopHistoryBreakFrequency());

      namePrefix = "Verification_";
      stableRotationDirectionThreshold = new DoubleParameter(namePrefix + "StableRotationDirectionThreshold", registry, parameters.getStableRotationDirectionThreshold());
      stableRotationPositionThreshold = new DoubleParameter(namePrefix + "StableRotationPositionThreshold", registry, parameters.getStableRotationPositionThreshold());
      stableEdgeWindowSize = new IntegerParameter(namePrefix + "StableEdgeWindowSize", registry, parameters.getStableEdgeWindowSize());
      inlineCoPHistoryStdDev = new DoubleParameter(namePrefix + "InlineCoPHistoryStdDev", registry, parameters.getInlineCoPHistoryStdDev());
      transverseCoPHistoryStdDev = new DoubleParameter(namePrefix + "TransverseCoPHistoryStdDev", registry, parameters.getTransverseCoPHistoryStdDev());

      namePrefix = "Kinematic_";
      angularVelocityAroundLoRThreshold = new DoubleParameter(namePrefix + "omegaMagnitudeThreshold", registry, parameters.getAngularVelocityAroundLoRThreshold());
      footDropThreshold = new DoubleParameter(namePrefix + "FootDropThreshold", registry, parameters.getFootDropThreshold());
      angularVelocityFilterBreakFrequency = new DoubleParameter(namePrefix + "omegaFilterBreakFrequency", registry, parameters.getAngularVelocityFilterBreakFrequency());

      namePrefix = "Velocity_";
      omegaThresholdForEstimation = new DoubleParameter(namePrefix + "omegaMagnitudeThreshold", registry, parameters.getOmegaMagnitudeThresholdForEstimation());
      decayBreakFrequency = new DoubleParameter(namePrefix + "rotationAngleDecayBreakFrequency", registry, parameters.getVelocityRotationAngleDecayBreakFrequency());
      rotationAngleThreshold = new DoubleParameter(namePrefix + "rotationAngleThreshold", registry, parameters.getVelocityRotationAngleThreshold());
      velocityEdgeFilterBreakFrequency = new DoubleParameter(namePrefix + "velocityEdgeFilterBreakFrequency", registry, parameters.getVelocityEdgeFilterBreakFrequency());

      createPartialFootholdModule = parameters.createPartialFootholdModule();
      if (parameters.createPartialFootholdModule())
         footholdCroppingParameters = new FootholdCroppingParameters(parameters, registry);
      else
         footholdCroppingParameters = null;
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

   public DoubleProvider getInlineCoPHistoryStdDev()
   {
      return inlineCoPHistoryStdDev;
   }

   public DoubleProvider getTransverseCoPHistoryStdDev()
   {
      return transverseCoPHistoryStdDev;
   }

   public boolean createPartialFootholdModule()
   {
      return createPartialFootholdModule;
   }

   public FootholdCroppingParameters getFootholdCroppingParameters()
   {
      return footholdCroppingParameters;
   }

   public static class FootholdCroppingParameters
   {
      private final DoubleProvider distanceFromLineToComputeDesiredCoPOccupancy;
      private final IntegerProvider numberOfDesiredCopsOnCropSide;
      private final DoubleProvider copAreaThreshold;

      private final DoubleProvider perpendicularCoPError;

      private final DoubleProvider minimumAreaForCropping;
      private final DoubleProvider distanceFromRotationToCrop;
      private final IntegerProvider shrinkMaxLimit;
      private final IntegerProvider thresholdForCoPRegionOccupancy;
      private final DoubleProvider distanceFromLineOfRotationToComputeCoPOccupancy;

      private final BooleanParameter doPartialFootholdDetection;
      private final BooleanParameter applyPartialFootholds;

      private final BooleanProvider useCoPOccupancyGridForCropping;
      private final DoubleProvider footDropThresholdForCrop;

      public FootholdCroppingParameters(PartialFootholdModuleParameters parameters, YoRegistry registry)
      {
         String namePrefix = "Verification_";
         perpendicularCoPError = new DoubleParameter(namePrefix + "PerpendicularCoPErrorThreshold", registry, parameters.getPerpendicularCoPErrorThreshold());
         numberOfDesiredCopsOnCropSide = new IntegerParameter(namePrefix + "NumberOfDesiredCopsOnCropSide", registry, parameters.getNumberOfDesiredCopsOnCropSide());
         distanceFromLineToComputeDesiredCoPOccupancy = new DoubleParameter(namePrefix + "DistanceFromLineToComputeDesiredCoPOccupancy", registry, parameters.getDistanceFromLineToComputeDesiredCoPOccupancy());
         copAreaThreshold = new DoubleParameter(namePrefix + "CopHullAreaRatioThreshold", registry, parameters.getCopHullAreaRatioThreshold());

          namePrefix = "Cropping_";
         minimumAreaForCropping = new DoubleParameter(namePrefix + "MinimumAreaForCropping", registry, parameters.getMinimumAreaForCropping());
         distanceFromRotationToCrop = new DoubleParameter(namePrefix + "DistanceFromRotationToCrop", registry, parameters.getDistanceFromRotationToCrop());
         thresholdForCoPRegionOccupancy = new IntegerParameter(namePrefix + "ThresholdForCoPRegionOccupancy", registry,
                                                               parameters.getThresholdForCoPRegionOccupancy());
         distanceFromLineOfRotationToComputeCoPOccupancy = new DoubleParameter(namePrefix + "DistanceFromLineOfRotationToComputeCoPOccupancy",
                                                                               registry,
                                                                               parameters.getDistanceFromLineOfRotationToComputeCoPOccupancy());
         shrinkMaxLimit = new IntegerParameter(namePrefix + "ShrinkMaxLimit", registry, parameters.getShrinkMaxLimit());

         useCoPOccupancyGridForCropping = new BooleanParameter(namePrefix + "UseCopOccupancyGrid", registry, parameters.getUseCoPOccupancyGridForCropping());
         footDropThresholdForCrop = new DoubleParameter(namePrefix + "FootDropThresholdForCrop", registry, parameters.getFootDropThresholdForCrop());

         doPartialFootholdDetection = new BooleanParameter("doPartialFootholdDetection", registry, parameters.getDoPartialFootholdDetection());
         applyPartialFootholds = new BooleanParameter("applyPartialFootholds", registry, parameters.getApplyPartialFootholds());
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

      public DoubleProvider getCopHullAreaRatioThreshold()
      {
         return copAreaThreshold;
      }

      public DoubleProvider getMinimumAreaForCropping()
      {
         return minimumAreaForCropping;
      }

      public DoubleProvider getDistanceFromRotationToCrop()
      {
         return distanceFromRotationToCrop;
      }

      public IntegerProvider getShrinkMaxLimit()
      {
         return shrinkMaxLimit;
      }

      public IntegerProvider getThresholdForCoPRegionOccupancy()
      {
         return thresholdForCoPRegionOccupancy;
      }

      public DoubleProvider getDistanceFromLineOfRotationToComputeCoPOccupancy()
      {
         return distanceFromLineOfRotationToComputeCoPOccupancy;
      }

      public BooleanProvider getApplyPartialFootholds()
      {
         return applyPartialFootholds;
      }

      public BooleanProvider getDoPartialFootholdDetection()
      {
         return doPartialFootholdDetection;
      }

      public BooleanProvider getUseCoPOccupancyGridForCropping()
      {
         return useCoPOccupancyGridForCropping;
      }

      public DoubleProvider getFootDropThresholdForCrop()
      {
         return footDropThresholdForCrop;
      }
   }
}
