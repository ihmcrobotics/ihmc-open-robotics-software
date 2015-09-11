package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;


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
public class FilteredVelocityYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private double alphaDouble;
   private final double dt;

   private final DoubleYoVariable alphaVariable;
   private final DoubleYoVariable position;

// private double lastPosition;
   private final DoubleYoVariable lastPosition;
   private final BooleanYoVariable hasBeenCalled;

   public FilteredVelocityYoVariable(String name, String description, double alpha, double dt, YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.alphaDouble = alpha;
      this.dt = dt;

      this.alphaVariable = null;
      this.position = null;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);

      reset();
   }

   public FilteredVelocityYoVariable(String name, String description, double alpha, DoubleYoVariable positionVariable, double dt, YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.alphaDouble = alpha;
      this.position = positionVariable;
      this.dt = dt;

      this.alphaVariable = null;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);


      reset();
   }

   public FilteredVelocityYoVariable(String name, String description, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable, double dt, YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      position = positionVariable;
      this.alphaVariable = alphaVariable;
      this.alphaDouble = 0.0;

      this.dt = dt;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);

      reset();
   }

   public FilteredVelocityYoVariable(String name, String description, DoubleYoVariable alphaVariable, double dt, YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.position = null;
      this.alphaVariable = alphaVariable;
      this.alphaDouble = 0.0;

      this.dt = dt;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

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

      double alpha = alphaVariable == null ? alphaDouble : alphaVariable.getDoubleValue();
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
         alphaVariable.set(alpha);
      }
   }
}
