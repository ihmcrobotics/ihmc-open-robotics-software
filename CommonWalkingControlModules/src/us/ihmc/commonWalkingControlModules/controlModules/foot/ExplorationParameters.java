package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.foot.PartialFootholdControlModule.RotationCalculatorType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This class provides parameters for foothold exploration that are common for both feet. In this
 * way they are not created twice for left and right and tuning is easier.
 *
 * @author Georg
 *
 */
public class ExplorationParameters
{
   /** Parameters for the geometric foothold detection */
   private final YoDouble geometricDetectionAngleThreshold;
   private final YoDouble geometricDetectionPlanePointAlpha;
   private final static double defaultGeometricDetectionAngleThreshold = 10.0 * Math.PI/180.0;
   private final static double defaultGeometricDetectionPlanePointAlpha = 0.99;

   /** Parameters for the cop occupancy grid */
   private final YoDouble copGridThresholdForOccupancy;
   private final YoDouble copGridDecayAlpha;
   private static final double defaultCopGridThresholdForOccupancy = 3.0;
   private static final double defaultCopGridDecayAlpha = 1.0;

   /** Parameters for the partial foothold control module */
   private final YoBoolean useCopOccupancyGrid;
   private final YoInteger thresholdForCoPRegionOccupancy;
   private final YoDouble distanceFromLineOfRotationToComputeCoPOccupancy;
   private final YoInteger shrinkMaxLimit;
   private final YoEnum<RotationCalculatorType> rotationCalculatorType;
   private final YoDouble minAreaToConsider;
   private static final boolean defaultUseCopOccupancyGrid = true;
   private static final int defaultThresholdForCoPRegionOccupancy = 2;
   private static final double defaultDistanceFromLineOfRotationToComputeCoPOccupancy = 0.02;
   private static final int defaultShrinkMaxLimit = 6;
   private static final RotationCalculatorType defaultRotationCalculatorType = RotationCalculatorType.GEOMETRY;
   private static final double defaultMinAreaToConsider = 0.0;

   /** Parameters for the cop based rotation verification */
   private final YoDouble perpendicluarCopErrorThreshold;
   private final YoDouble copAllowedAreaOpeningAngle;
   private static final double defaultPerpendicluarCopErrorThreshold = 0.005;
   private static final double defaultCopAllowedAreaOpeningAngle = 30.0 * Math.PI/180.0;

   /** Parameters for the velocity based foothold detection */
   private final YoDouble stableLoRAngularVelocityThreshold;
   private final YoDouble stableCoRLinearVelocityThreshold;
   private final YoDouble angularVelocityAroundLoRThreshold;
   private final YoDouble footDropThreshold;
   private final YoDouble angularVelocityFilterBreakFrequency;
   private static final double defaultStableLoRAngularVelocityThreshold = 10.0;
   private static final double defaultStableCoRLinearVelocityThreshold = 0.1;
   private static final double defaultAngularVelocityAroundLoRThreshold = 0.5;
   private static final double defaultFootDropThreshold = -0.04;
   private static final double defaultAngularVelocityFilterBreakFrequency = 16.0;

   /** Parameters for foothold exploration state */
   private final YoDouble recoverTime;
   private final YoDouble timeToGoToCorner;
   private final YoDouble timeToStayInCorner;
   private final YoDouble copCommandWeight;
   private final YoDouble timeBeforeExploring;
   private static final double defaultRecoverTime = 0.05;
   private static final double defaultTimeToGoToCorner = 0.3;
   private static final double defaultTimeToStayInCorner = 0.2;
   private static final double defaultCopCommandWeight = 200.0;
   private static final double defaultTimeBeforeExploring = 1.5;

   public ExplorationParameters(YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(this.getClass().getSimpleName());
      parentRegistry.addChild(registry);

      String namePrefix = "ExplorationGeometric_";
      geometricDetectionAngleThreshold = new YoDouble(namePrefix + "AngleThreshold", registry);
      geometricDetectionAngleThreshold.set(defaultGeometricDetectionAngleThreshold);
      geometricDetectionPlanePointAlpha = new YoDouble(namePrefix + "PlanePointAlpha", registry);
      geometricDetectionPlanePointAlpha.set(defaultGeometricDetectionPlanePointAlpha);

      namePrefix = "ExplorationCopGrid_";
      copGridThresholdForOccupancy = new YoDouble(namePrefix + "ThresholdForOccupancy", registry);
      copGridThresholdForOccupancy.set(defaultCopGridThresholdForOccupancy);
      copGridDecayAlpha = new YoDouble(namePrefix + "DecayAlpha", registry);
      copGridDecayAlpha.set(defaultCopGridDecayAlpha);

      namePrefix = "ExplorationFoothold_";
      useCopOccupancyGrid = new YoBoolean(namePrefix + "UseCopOccupancyGrid", registry);
      useCopOccupancyGrid.set(defaultUseCopOccupancyGrid);
      thresholdForCoPRegionOccupancy = new YoInteger(namePrefix + "ThresholdForCoPRegionOccupancy", registry);
      thresholdForCoPRegionOccupancy.set(defaultThresholdForCoPRegionOccupancy);
      distanceFromLineOfRotationToComputeCoPOccupancy = new YoDouble(namePrefix + "DistanceFromLineOfRotationToComputeCoPOccupancy", registry);
      distanceFromLineOfRotationToComputeCoPOccupancy.set(defaultDistanceFromLineOfRotationToComputeCoPOccupancy);
      shrinkMaxLimit = new YoInteger(namePrefix + "ShrinkMaxLimit", registry);
      shrinkMaxLimit.set(defaultShrinkMaxLimit);
      rotationCalculatorType = new YoEnum<>(namePrefix + "RotationCalculatorType", registry, RotationCalculatorType.class);
      rotationCalculatorType.set(defaultRotationCalculatorType);
      minAreaToConsider = new YoDouble(namePrefix + "MinAreaToConsider", registry);
      minAreaToConsider.set(defaultMinAreaToConsider);

      namePrefix = "ExplorationVerification_";
      perpendicluarCopErrorThreshold = new YoDouble(namePrefix + "PerpendicluarCopErrorThreshold", registry);
      perpendicluarCopErrorThreshold.set(defaultPerpendicluarCopErrorThreshold);
      copAllowedAreaOpeningAngle = new YoDouble(namePrefix + "CopAllowedAreaOpeningAngle", registry);
      copAllowedAreaOpeningAngle.set(defaultCopAllowedAreaOpeningAngle);

      namePrefix = "ExplorationVelocity_";
      stableLoRAngularVelocityThreshold = new YoDouble(namePrefix + "StableLoRAngularVelocityThreshold", registry);
      stableLoRAngularVelocityThreshold.set(defaultStableLoRAngularVelocityThreshold);
      stableCoRLinearVelocityThreshold = new YoDouble(namePrefix + "StableCoRLinearVelocityThreshold", registry);
      stableCoRLinearVelocityThreshold.set(defaultStableCoRLinearVelocityThreshold);
      angularVelocityAroundLoRThreshold = new YoDouble(namePrefix + "AngularVelocityAroundLoRThreshold", registry);
      angularVelocityAroundLoRThreshold.set(defaultAngularVelocityAroundLoRThreshold);
      footDropThreshold = new YoDouble(namePrefix + "FootDropThreshold", registry);
      footDropThreshold.set(defaultFootDropThreshold);
      angularVelocityFilterBreakFrequency = new YoDouble(namePrefix + "AngularVelocityFilterBreakFrequency", registry);
      angularVelocityFilterBreakFrequency.set(defaultAngularVelocityFilterBreakFrequency);

      namePrefix = "ExplorationState_";
      recoverTime = new YoDouble(namePrefix + "RecoverTime", registry);
      recoverTime.set(defaultRecoverTime);
      timeToGoToCorner = new YoDouble(namePrefix + "TimeToGoToCorner", registry);
      timeToGoToCorner.set(defaultTimeToGoToCorner);
      timeToStayInCorner = new YoDouble(namePrefix + "TimeToStayInCorner", registry);
      timeToStayInCorner.set(defaultTimeToStayInCorner);
      copCommandWeight = new YoDouble(namePrefix + "CopCommandWeight", registry);
      copCommandWeight.set(defaultCopCommandWeight);
      timeBeforeExploring = new YoDouble(namePrefix + "TimeBeforeExploring", registry);
      timeBeforeExploring.set(defaultTimeBeforeExploring);
   }

   public YoDouble getGeometricDetectionAngleThreshold()
   {
      return geometricDetectionAngleThreshold;
   }

   public YoDouble getGeometricDetectionPlanePointAlpha()
   {
      return geometricDetectionPlanePointAlpha;
   }

   public YoDouble getCopGridThresholdForOccupancy()
   {
      return copGridThresholdForOccupancy;
   }

   public YoDouble getCopGridDecayAlpha()
   {
      return copGridDecayAlpha;
   }

   public YoBoolean getUseCopOccupancyGrid()
   {
      return useCopOccupancyGrid;
   }

   public YoInteger getThresholdForCoPRegionOccupancy()
   {
      return thresholdForCoPRegionOccupancy;
   }

   public YoDouble getDistanceFromLineOfRotationToComputeCoPOccupancy()
   {
      return distanceFromLineOfRotationToComputeCoPOccupancy;
   }

   public YoInteger getShrinkMaxLimit()
   {
      return shrinkMaxLimit;
   }

   public YoEnum<RotationCalculatorType> getRotationCalculatorType()
   {
      return rotationCalculatorType;
   }

   public YoDouble getMinAreaToConsider()
   {
      return minAreaToConsider;
   }

   public YoDouble getPerpendicluarCopErrorThreshold()
   {
      return perpendicluarCopErrorThreshold;
   }

   public YoDouble getCopAllowedAreaOpeningAngle()
   {
      return copAllowedAreaOpeningAngle;
   }

   public YoDouble getStableLoRAngularVelocityThreshold()
   {
      return stableLoRAngularVelocityThreshold;
   }

   public YoDouble getStableCoRLinearVelocityThreshold()
   {
      return stableCoRLinearVelocityThreshold;
   }

   public YoDouble getAngularVelocityAroundLoRThreshold()
   {
      return angularVelocityAroundLoRThreshold;
   }

   public YoDouble getFootDropThreshold()
   {
      return footDropThreshold;
   }

   public YoDouble getAngularVelocityFilterBreakFrequency()
   {
      return angularVelocityFilterBreakFrequency;
   }

   public YoDouble getRecoverTime()
   {
      return recoverTime;
   }

   public YoDouble getTimeToGoToCorner()
   {
      return timeToGoToCorner;
   }

   public YoDouble getTimeToStayInCorner()
   {
      return timeToStayInCorner;
   }

   public YoDouble getCopCommandWeight()
   {
      return copCommandWeight;
   }

   public YoDouble getTimeBeforeExploring()
   {
      return timeBeforeExploring;
   }
}
