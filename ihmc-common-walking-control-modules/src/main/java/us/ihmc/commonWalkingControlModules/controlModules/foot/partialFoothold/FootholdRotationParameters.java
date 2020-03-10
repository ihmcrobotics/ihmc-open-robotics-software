package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootholdRotationParameters
{
   private final DoubleProvider geometricDetectionAngleThreshold;

   private final DoubleProvider angularVelocityAroundLoRThreshold;
   private final DoubleProvider footDropThreshold;
   private final DoubleProvider angularVelocityFilterBreakFrequency;
   private final DoubleProvider stableLoRAngularVelocityThreshold;
   private final DoubleProvider stableCoRLinearVelocityThreshold;

   private final DoubleProvider omegaThresholdForEstimation;
   private final DoubleProvider decayBreakFrequency;
   private final DoubleProvider rotationAngleThreshold;
   private final DoubleProvider velocityEdgeFilterBreakFrequency;

   private final IntegerProvider thresholdForCoPRegionOccupancy;
   private final DoubleProvider distanceFromLineOfRotationToComputeCoPOccupancy;

   public FootholdRotationParameters(YoVariableRegistry registry)
   {
      String namePrefix = "Geometric_";
      geometricDetectionAngleThreshold = new DoubleParameter(namePrefix + "AngleThreshold", registry, Math.toRadians(10.0));

      namePrefix = "Verification_";
      stableLoRAngularVelocityThreshold = new DoubleParameter(namePrefix + "StableLoRAngularVelocityThreshold", registry, 10.0);
      stableCoRLinearVelocityThreshold = new DoubleParameter(namePrefix + "StableCoRLinearVelocityThreshold", registry, 0.1);

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
   }

   public DoubleProvider getGeometricDetectionAngleThreshold()
   {
      return geometricDetectionAngleThreshold;
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

   public DoubleProvider getStableLoRAngularVelocityThreshold()
   {
      return stableLoRAngularVelocityThreshold;
   }

   public DoubleProvider getStableCoRLinearVelocityThreshold()
   {
      return stableCoRLinearVelocityThreshold;
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
}
