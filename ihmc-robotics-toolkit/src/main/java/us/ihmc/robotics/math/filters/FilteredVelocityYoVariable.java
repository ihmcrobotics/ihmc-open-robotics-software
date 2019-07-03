package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * @author jrebula
 *         <p>
 *         A YoFilteredVelocityVariable is a filtered velocity of a position.
 *         Either a YoVariable holding the position is passed in to the
 *         constructor and update() is called every tick, or update(double) is
 *         called every tick. The YoFilteredVelocityVariable updates it's val
 *         with the current velocity after a filter of
 *         </p>
 *
 * <pre>
 *            vel_{n} = alpha * vel{n-1} + (1 - alpha) * (pos_{n} - pos_{n-1})
 * </pre>
 *
 */
public class FilteredVelocityYoVariable extends YoDouble implements ProcessingYoVariable
{
   private double alphaDouble;
   private final double dt;

   private final DoubleProvider alphaVariable;
   private final YoDouble position;

// private double lastPosition;
   private final YoDouble lastPosition;
   private final YoBoolean hasBeenCalled;

   public FilteredVelocityYoVariable(String name, String description, double alpha, double dt, YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      this.alphaDouble = alpha;
      this.dt = dt;

      this.alphaVariable = null;
      this.position = null;

      lastPosition = new YoDouble(name + "_lastPosition", registry);

      reset();
   }

   public FilteredVelocityYoVariable(String name, String description, double alpha, YoDouble positionVariable, double dt, YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      this.alphaDouble = alpha;
      this.position = positionVariable;
      this.dt = dt;

      this.alphaVariable = null;

      lastPosition = new YoDouble(name + "_lastPosition", registry);


      reset();
   }

   public FilteredVelocityYoVariable(String name, String description, DoubleProvider alphaVariable, YoDouble positionVariable, double dt, YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      position = positionVariable;
      this.alphaVariable = alphaVariable;
      this.alphaDouble = 0.0;

      this.dt = dt;

      lastPosition = new YoDouble(name + "_lastPosition", registry);

      reset();
   }

   public FilteredVelocityYoVariable(String name, String description, DoubleProvider alphaVariable, double dt, YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      this.position = null;
      this.alphaVariable = alphaVariable;
      this.alphaDouble = 0.0;

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
         throw new NullPointerException("YoFilteredVelocityVariable must be constructed with a non null "
                                        + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getDoubleValue());
   }

   public void updateForAngles()
   {
      if (position == null)
      {
         throw new NullPointerException("YoFilteredVelocityVariable must be constructed with a non null "
                                        + "position variable to call update(), otherwise use update(double)");
      }

      updateForAngles(position.getDoubleValue());
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

      double alpha = alphaVariable == null ? alphaDouble : alphaVariable.getValue();
      set(alpha * previousFilteredDerivative + (1.0 - alpha) * currentRawDerivative);
   }

   public void setAlpha(double alpha)
   {
      if (alphaVariable == null)
      {
         this.alphaDouble = alpha;
      }
      else
      {
         throw new RuntimeException("A double provider was used to construct this filtered variable. Modyfy the value of that provider directly.");
      }
   }
}
