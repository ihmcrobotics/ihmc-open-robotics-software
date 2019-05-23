package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoDCMPlannerParameters implements DCMPlannerParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble safeDistanceFromSupportPolygonEdges = new YoDouble("safeDistanceFromSupportPolygonEdges", registry);

   private final YoDouble stanceWidthCoPShiftFactor = new YoDouble("stanceWidthCoPShiftFactor", registry);
   private final YoDouble stanceLengthCoPShiftFactor = new YoDouble("stanceLengthCoPShiftFactor", registry);
   private final YoDouble maxStanceWidthCoPShift = new YoDouble("maxStanceWidthCoPShift", registry);
   private final YoDouble maxStanceLengthCoPShift = new YoDouble("maxStanceLengthCoPShift", registry);
   private final YoDouble stepWidthCoPShiftFactor = new YoDouble("maxWidthCoPShiftFactor", registry);
   private final YoDouble stepLengthCoPShiftFactor = new YoDouble("maxLengthCoPShiftFactor", registry);
   private final YoDouble maxStepWidthCoPShift = new YoDouble("maxStepWidthCoPShift", registry);
   private final YoDouble maxStepLengthCoPShift = new YoDouble("maxStepLengthCoPShift", registry);

   private final YoDouble maximumWeightShiftForward = new YoDouble("maximumWeightShiftForward", registry);
   private final YoDouble angleForMaxWeightShiftForward = new YoDouble("angleForMaxWeightShiftForward", registry);

   public YoDCMPlannerParameters(DCMPlannerParameters other, YoVariableRegistry parentRegistry)
   {
      safeDistanceFromSupportPolygonEdges.set(other.getSafeDistanceFromSupportPolygonEdges());

      stanceWidthCoPShiftFactor.set(other.getStanceWidthCoPShiftFactor());
      stanceLengthCoPShiftFactor.set(other.getStanceLengthCoPShiftFactor());
      maxStanceWidthCoPShift.set(other.getMaxStanceWidthCoPShift());
      maxStanceLengthCoPShift.set(other.getMaxStanceLengthCoPShift());
      stepWidthCoPShiftFactor.set(other.getStepWidthCoPShiftFactor());
      stepLengthCoPShiftFactor.set(other.getStepLengthCoPShiftFactor());
      maxStepWidthCoPShift.set(other.getMaxStepWidthCoPShift());
      maxStepLengthCoPShift.set(other.getMaxStepLengthCoPShift());

      maximumWeightShiftForward.set(other.getMaximumWeightShiftForward());
      angleForMaxWeightShiftForward.set(other.getAngleForMaxWeightShiftForward());

      parentRegistry.addChild(registry);
   }

   /** {@inheritDoc} */
   @Override
   public double getSafeDistanceFromSupportPolygonEdges()
   {
      return safeDistanceFromSupportPolygonEdges.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getStanceWidthCoPShiftFactor()
   {
      return stanceWidthCoPShiftFactor.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getStanceLengthCoPShiftFactor()
   {
      return stanceLengthCoPShiftFactor.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxStanceWidthCoPShift()
   {
      return maxStanceWidthCoPShift.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxStanceLengthCoPShift()
   {
      return maxStanceLengthCoPShift.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getStepWidthCoPShiftFactor()
   {
      return stepWidthCoPShiftFactor.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthCoPShiftFactor()
   {
      return stepLengthCoPShiftFactor.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxStepWidthCoPShift()
   {
      return maxStepWidthCoPShift.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxStepLengthCoPShift()
   {
      return maxStepLengthCoPShift.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumWeightShiftForward()
   {
      return maximumWeightShiftForward.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getAngleForMaxWeightShiftForward()
   {
      return angleForMaxWeightShiftForward.getDoubleValue();
   }
}
