package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FootholdRotationParameters
{
   private final DoubleProvider geometricDetectionAngleThreshold;

   private final DoubleProvider copHistoryAlphaFilter;

   private final DoubleProvider angularVelocityAroundLoRThreshold;
   private final DoubleProvider footDropThreshold;
   private final DoubleProvider angularVelocityFilterBreakFrequency;
   private final DoubleProvider stableRotationDirectionThreshold;
   private final DoubleProvider stableRotationPositionThreshold;
   private final IntegerProvider minimumTicksForEstimate;

   private final DoubleProvider omegaThresholdForEstimation;
   private final DoubleProvider decayBreakFrequency;
   private final DoubleProvider rotationAngleThreshold;
   private final DoubleProvider velocityEdgeFilterBreakFrequency;

   private final IntegerProvider thresholdForCoPRegionOccupancy;
   private final DoubleProvider distanceFromLineOfRotationToComputeCoPOccupancy;

   private final DoubleProvider minimumAreaForCropping;
   private final IntegerProvider shrinkMaxLimit;

   public FootholdRotationParameters(YoVariableRegistry registry)
   {
      String namePrefix = "Geometric_";
      geometricDetectionAngleThreshold = new DoubleParameter(namePrefix + "AngleThreshold", registry, Math.toRadians(10.0));

      namePrefix = "CoPHistory_";
      copHistoryAlphaFilter = new DoubleParameter(namePrefix + "CoPHistoryAlphaFilter", registry, 0.0);

      namePrefix = "Verification_";
      stableRotationDirectionThreshold = new DoubleParameter(namePrefix + "StableRotationDirectionThrehsold", registry, 10.0);
      stableRotationPositionThreshold = new DoubleParameter(namePrefix + "StableRotationPositionThreshold", registry, 0.1);
      minimumTicksForEstimate = new IntegerParameter(namePrefix + "MinimumTicksForRotationEstimate", registry, 5);

      namePrefix = "Kinematic_";
      angularVelocityAroundLoRThreshold = new DoubleParameter(namePrefix + "omegaMagnitudeThreshold", registry, 0.5);
      footDropThreshold = new DoubleParameter(namePrefix + "FootDropThreshold", registry, 0.04);
      angularVelocityFilterBreakFrequency = new DoubleParameter(namePrefix + "omegaFilterBreakFrequency", registry, 16.0);

      namePrefix = "Velocity_";
      omegaThresholdForEstimation = new DoubleParameter(namePrefix + "omegaMagnitudeThreshold", registry, 1.5);
      decayBreakFrequency = new DoubleParameter(namePrefix + "rotationAngleDecayBreakFrequency", registry, 1.0);
      rotationAngleThreshold = new DoubleParameter(namePrefix + "rotationAngleThreshold", registry, 0.05);
      velocityEdgeFilterBreakFrequency = new DoubleParameter(namePrefix + "velocityEdgeFilterBreakFrequency", registry, 1.0);

      namePrefix = "Cropping_";
      thresholdForCoPRegionOccupancy = new IntegerParameter(namePrefix + "ThresholdForCoPRegionOccupancy", registry, 2);
      distanceFromLineOfRotationToComputeCoPOccupancy = new DoubleParameter(namePrefix + "DistanceFromLineOfRotationToComputeCoPOccupancy", registry, 0.02);
      minimumAreaForCropping = new DoubleParameter(namePrefix + "MinimumAreaForCropping", registry, 0.0);
      shrinkMaxLimit = new IntegerParameter(namePrefix + "ShrinkMaxLimit", registry, 6);
   }

   public DoubleProvider getGeometricDetectionAngleThreshold()
   {
      return geometricDetectionAngleThreshold;
   }

   public DoubleProvider getCopHistoryAlphaFilter()
   {
      return copHistoryAlphaFilter;
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

   public IntegerProvider getMinimumTicksForEstimate()
   {
      return minimumTicksForEstimate;
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
}
