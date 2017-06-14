package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.foot.PartialFootholdControlModule.RotationCalculatorType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.BooleanYoVariable;
import us.ihmc.yoVariables.variable.DoubleYoVariable;
import us.ihmc.yoVariables.variable.EnumYoVariable;
import us.ihmc.yoVariables.variable.IntegerYoVariable;

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
   private final DoubleYoVariable geometricDetectionAngleThreshold;
   private final DoubleYoVariable geometricDetectionPlanePointAlpha;
   private final static double defaultGeometricDetectionAngleThreshold = 10.0 * Math.PI/180.0;
   private final static double defaultGeometricDetectionPlanePointAlpha = 0.99;

   /** Parameters for the cop occupancy grid */
   private final DoubleYoVariable copGridThresholdForOccupancy;
   private final DoubleYoVariable copGridDecayAlpha;
   private static final double defaultCopGridThresholdForOccupancy = 3.0;
   private static final double defaultCopGridDecayAlpha = 1.0;

   /** Parameters for the partial foothold control module */
   private final BooleanYoVariable useCopOccupancyGrid;
   private final IntegerYoVariable thresholdForCoPRegionOccupancy;
   private final DoubleYoVariable distanceFromLineOfRotationToComputeCoPOccupancy;
   private final IntegerYoVariable shrinkMaxLimit;
   private final EnumYoVariable<RotationCalculatorType> rotationCalculatorType;
   private final DoubleYoVariable minAreaToConsider;
   private static final boolean defaultUseCopOccupancyGrid = true;
   private static final int defaultThresholdForCoPRegionOccupancy = 2;
   private static final double defaultDistanceFromLineOfRotationToComputeCoPOccupancy = 0.02;
   private static final int defaultShrinkMaxLimit = 6;
   private static final RotationCalculatorType defaultRotationCalculatorType = RotationCalculatorType.GEOMETRY;
   private static final double defaultMinAreaToConsider = 0.0;

   /** Parameters for the cop based rotation verification */
   private final DoubleYoVariable perpendicluarCopErrorThreshold;
   private final DoubleYoVariable copAllowedAreaOpeningAngle;
   private static final double defaultPerpendicluarCopErrorThreshold = 0.005;
   private static final double defaultCopAllowedAreaOpeningAngle = 30.0 * Math.PI/180.0;

   /** Parameters for the velocity based foothold detection */
   private final DoubleYoVariable stableLoRAngularVelocityThreshold;
   private final DoubleYoVariable stableCoRLinearVelocityThreshold;
   private final DoubleYoVariable angularVelocityAroundLoRThreshold;
   private final DoubleYoVariable footDropThreshold;
   private final DoubleYoVariable angularVelocityFilterBreakFrequency;
   private static final double defaultStableLoRAngularVelocityThreshold = 10.0;
   private static final double defaultStableCoRLinearVelocityThreshold = 0.1;
   private static final double defaultAngularVelocityAroundLoRThreshold = 0.5;
   private static final double defaultFootDropThreshold = -0.04;
   private static final double defaultAngularVelocityFilterBreakFrequency = 16.0;

   /** Parameters for foothold exploration state */
   private final DoubleYoVariable recoverTime;
   private final DoubleYoVariable timeToGoToCorner;
   private final DoubleYoVariable timeToStayInCorner;
   private final DoubleYoVariable copCommandWeight;
   private final DoubleYoVariable timeBeforeExploring;
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
      geometricDetectionAngleThreshold = new DoubleYoVariable(namePrefix + "AngleThreshold", registry);
      geometricDetectionAngleThreshold.set(defaultGeometricDetectionAngleThreshold);
      geometricDetectionPlanePointAlpha = new DoubleYoVariable(namePrefix + "PlanePointAlpha", registry);
      geometricDetectionPlanePointAlpha.set(defaultGeometricDetectionPlanePointAlpha);

      namePrefix = "ExplorationCopGrid_";
      copGridThresholdForOccupancy = new DoubleYoVariable(namePrefix + "ThresholdForOccupancy", registry);
      copGridThresholdForOccupancy.set(defaultCopGridThresholdForOccupancy);
      copGridDecayAlpha = new DoubleYoVariable(namePrefix + "DecayAlpha", registry);
      copGridDecayAlpha.set(defaultCopGridDecayAlpha);

      namePrefix = "ExplorationFoothold_";
      useCopOccupancyGrid = new BooleanYoVariable(namePrefix + "UseCopOccupancyGrid", registry);
      useCopOccupancyGrid.set(defaultUseCopOccupancyGrid);
      thresholdForCoPRegionOccupancy = new IntegerYoVariable(namePrefix + "ThresholdForCoPRegionOccupancy", registry);
      thresholdForCoPRegionOccupancy.set(defaultThresholdForCoPRegionOccupancy);
      distanceFromLineOfRotationToComputeCoPOccupancy = new DoubleYoVariable(namePrefix + "DistanceFromLineOfRotationToComputeCoPOccupancy", registry);
      distanceFromLineOfRotationToComputeCoPOccupancy.set(defaultDistanceFromLineOfRotationToComputeCoPOccupancy);
      shrinkMaxLimit = new IntegerYoVariable(namePrefix + "ShrinkMaxLimit", registry);
      shrinkMaxLimit.set(defaultShrinkMaxLimit);
      rotationCalculatorType = new EnumYoVariable<>(namePrefix + "RotationCalculatorType", registry, RotationCalculatorType.class);
      rotationCalculatorType.set(defaultRotationCalculatorType);
      minAreaToConsider = new DoubleYoVariable(namePrefix + "MinAreaToConsider", registry);
      minAreaToConsider.set(defaultMinAreaToConsider);

      namePrefix = "ExplorationVerification_";
      perpendicluarCopErrorThreshold = new DoubleYoVariable(namePrefix + "PerpendicluarCopErrorThreshold", registry);
      perpendicluarCopErrorThreshold.set(defaultPerpendicluarCopErrorThreshold);
      copAllowedAreaOpeningAngle = new DoubleYoVariable(namePrefix + "CopAllowedAreaOpeningAngle", registry);
      copAllowedAreaOpeningAngle.set(defaultCopAllowedAreaOpeningAngle);

      namePrefix = "ExplorationVelocity_";
      stableLoRAngularVelocityThreshold = new DoubleYoVariable(namePrefix + "StableLoRAngularVelocityThreshold", registry);
      stableLoRAngularVelocityThreshold.set(defaultStableLoRAngularVelocityThreshold);
      stableCoRLinearVelocityThreshold = new DoubleYoVariable(namePrefix + "StableCoRLinearVelocityThreshold", registry);
      stableCoRLinearVelocityThreshold.set(defaultStableCoRLinearVelocityThreshold);
      angularVelocityAroundLoRThreshold = new DoubleYoVariable(namePrefix + "AngularVelocityAroundLoRThreshold", registry);
      angularVelocityAroundLoRThreshold.set(defaultAngularVelocityAroundLoRThreshold);
      footDropThreshold = new DoubleYoVariable(namePrefix + "FootDropThreshold", registry);
      footDropThreshold.set(defaultFootDropThreshold);
      angularVelocityFilterBreakFrequency = new DoubleYoVariable(namePrefix + "AngularVelocityFilterBreakFrequency", registry);
      angularVelocityFilterBreakFrequency.set(defaultAngularVelocityFilterBreakFrequency);

      namePrefix = "ExplorationState_";
      recoverTime = new DoubleYoVariable(namePrefix + "RecoverTime", registry);
      recoverTime.set(defaultRecoverTime);
      timeToGoToCorner = new DoubleYoVariable(namePrefix + "TimeToGoToCorner", registry);
      timeToGoToCorner.set(defaultTimeToGoToCorner);
      timeToStayInCorner = new DoubleYoVariable(namePrefix + "TimeToStayInCorner", registry);
      timeToStayInCorner.set(defaultTimeToStayInCorner);
      copCommandWeight = new DoubleYoVariable(namePrefix + "CopCommandWeight", registry);
      copCommandWeight.set(defaultCopCommandWeight);
      timeBeforeExploring = new DoubleYoVariable(namePrefix + "TimeBeforeExploring", registry);
      timeBeforeExploring.set(defaultTimeBeforeExploring);
   }

   public DoubleYoVariable getGeometricDetectionAngleThreshold()
   {
      return geometricDetectionAngleThreshold;
   }

   public DoubleYoVariable getGeometricDetectionPlanePointAlpha()
   {
      return geometricDetectionPlanePointAlpha;
   }

   public DoubleYoVariable getCopGridThresholdForOccupancy()
   {
      return copGridThresholdForOccupancy;
   }

   public DoubleYoVariable getCopGridDecayAlpha()
   {
      return copGridDecayAlpha;
   }

   public BooleanYoVariable getUseCopOccupancyGrid()
   {
      return useCopOccupancyGrid;
   }

   public IntegerYoVariable getThresholdForCoPRegionOccupancy()
   {
      return thresholdForCoPRegionOccupancy;
   }

   public DoubleYoVariable getDistanceFromLineOfRotationToComputeCoPOccupancy()
   {
      return distanceFromLineOfRotationToComputeCoPOccupancy;
   }

   public IntegerYoVariable getShrinkMaxLimit()
   {
      return shrinkMaxLimit;
   }

   public EnumYoVariable<RotationCalculatorType> getRotationCalculatorType()
   {
      return rotationCalculatorType;
   }

   public DoubleYoVariable getMinAreaToConsider()
   {
      return minAreaToConsider;
   }

   public DoubleYoVariable getPerpendicluarCopErrorThreshold()
   {
      return perpendicluarCopErrorThreshold;
   }

   public DoubleYoVariable getCopAllowedAreaOpeningAngle()
   {
      return copAllowedAreaOpeningAngle;
   }

   public DoubleYoVariable getStableLoRAngularVelocityThreshold()
   {
      return stableLoRAngularVelocityThreshold;
   }

   public DoubleYoVariable getStableCoRLinearVelocityThreshold()
   {
      return stableCoRLinearVelocityThreshold;
   }

   public DoubleYoVariable getAngularVelocityAroundLoRThreshold()
   {
      return angularVelocityAroundLoRThreshold;
   }

   public DoubleYoVariable getFootDropThreshold()
   {
      return footDropThreshold;
   }

   public DoubleYoVariable getAngularVelocityFilterBreakFrequency()
   {
      return angularVelocityFilterBreakFrequency;
   }

   public DoubleYoVariable getRecoverTime()
   {
      return recoverTime;
   }

   public DoubleYoVariable getTimeToGoToCorner()
   {
      return timeToGoToCorner;
   }

   public DoubleYoVariable getTimeToStayInCorner()
   {
      return timeToStayInCorner;
   }

   public DoubleYoVariable getCopCommandWeight()
   {
      return copCommandWeight;
   }

   public DoubleYoVariable getTimeBeforeExploring()
   {
      return timeBeforeExploring;
   }
}
