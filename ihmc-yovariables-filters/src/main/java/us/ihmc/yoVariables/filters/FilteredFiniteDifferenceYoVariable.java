package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * <p>
 * A {@link FilteredFiniteDifferenceYoVariable} computes the velocity of a position signal via finite differencing it, and then applies an alpha filter to that
 * value. The value contained in the parent {@link YoDouble} returns the filtered velocity signal.
 * </p>
 * <p>
 * To construct and use this variable, either a YoVariable holding the position is passed in at construction
 * constructor and {@link #update()} is called every tick, or {@link #update(double)} is
 * called every tick.
 * </p>
 * <p> The {@link FilteredFiniteDifferenceYoVariable} updates its value with the current velocity after a filter of
 * </p>
 *
 * <pre>
 *                  vel_{n} = alpha * vel{n-1} + (1 - alpha) * (pos_{n} - pos_{n-1})
 *       </pre>
 */
public class FilteredFiniteDifferenceYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final double dt;

   private final DoubleProvider alphaVariable;
   private final DoubleProvider position;

   // private double lastPosition;
   private final YoDouble lastPosition;
   private final YoBoolean hasBeenCalled;

   public FilteredFiniteDifferenceYoVariable(String name, String description, double alpha, double dt, YoRegistry registry)
   {
      this(name, description, alpha, null, dt, registry);
   }

   public FilteredFiniteDifferenceYoVariable(String name, String description, double alpha, DoubleProvider positionVariable, double dt, YoRegistry registry)
   {
      this(name, description, VariableTools.createAlphaYoDouble(name, "", alpha, registry), positionVariable, dt, registry);
   }

   public FilteredFiniteDifferenceYoVariable(String name, String description, DoubleProvider alphaVariable, double dt, YoRegistry registry)
   {
      this(name, description, alphaVariable, null, dt, registry);
   }

   public FilteredFiniteDifferenceYoVariable(String name,
                                             String description,
                                             DoubleProvider alphaVariable,
                                             DoubleProvider positionVariable,
                                             double dt,
                                             YoRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(name, "", registry);

      position = positionVariable;
      this.alphaVariable = alphaVariable;

      this.dt = dt;

      lastPosition = new YoDouble(name + "_lastPosition", registry);

      reset();
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
   }

   @Override
   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException(
               "YoFilteredVelocityVariable must be constructed with a non null " + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getValue());
   }

   public void updateForAngles()
   {
      if (position == null)
      {
         throw new NullPointerException(
               "YoFilteredVelocityVariable must be constructed with a non null " + "position variable to call update(), otherwise use update(double)");
      }

      updateForAngles(position.getValue());
   }

   public void update(double currentPosition)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         lastPosition.set(currentPosition);
         set(0.0);
      }

      double difference = currentPosition - lastPosition.getDoubleValue();

      updateUsingDifference(difference);

      lastPosition.set(currentPosition);
   }

   public void updateForAngles(double currentPosition)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         lastPosition.set(currentPosition);
         set(0.0);
      }

      double difference = AngleTools.computeAngleDifferenceMinusPiToPi(currentPosition, lastPosition.getDoubleValue());

      updateUsingDifference(difference);

      lastPosition.set(currentPosition);
   }

   private void updateUsingDifference(double difference)
   {
      double previousFilteredDerivative = getDoubleValue();
      double currentRawDerivative = difference / dt;

      double alpha = alphaVariable.getValue();
      set(alpha * previousFilteredDerivative + (1.0 - alpha) * currentRawDerivative);
   }
}
