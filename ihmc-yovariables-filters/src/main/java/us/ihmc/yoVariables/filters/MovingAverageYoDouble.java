package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * @author thutcheson
 *         </p>
 *         <p>
 *         </p>
 *         <p>
 *         A BetaFilteredYoVariable is a filtered version of an input YoVar.
 *         This is a moving average filter.
 *         Either a YoVariable holding the unfiltered val is passed in to the
 *         constructor and update() is called every tick, or update(double) is
 *         called every tick. The BetaFilteredYoVariable updates it's val
 *         with the current filtered version using
 *         </p>
 *         <pre>
 *            filtered_{n} = (raw_{0} + ... + raw_{n-1} + raw_{n}) / n
 *         </pre>
 */
public class MovingAverageYoDouble extends YoDouble
{
   private int beta;
   private int index = 0;
   @SuppressWarnings("unused")
   private final YoDouble betaVariable;

   private final YoDouble position;

   private final double[] raw;
   private boolean bufferFull = false;

   private final YoBoolean hasBeenCalled;

   public MovingAverageYoDouble(String name, YoRegistry registry, int beta)
   {
      this(name, "", registry, beta, null);
   }

   public MovingAverageYoDouble(String name, String description, YoRegistry registry, int beta)
   {
      this(name, description, registry, beta, null);
   }

   public MovingAverageYoDouble(String name, YoRegistry registry, int beta, YoDouble positionVariable)
   {
      this(name, "", registry, beta, positionVariable);
   }

   public MovingAverageYoDouble(String name, String description, YoRegistry registry, int beta, YoDouble positionVariable)
   {
      super(name, description, registry);
      this.hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(name, "", registry);

      this.beta = beta;
      this.position = positionVariable;
      this.betaVariable = null;

      raw = new double[beta];

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
      bufferFull = false;
      index = 0;

      for (int i = 0; i < beta; i++)
      {
         set(0.0);
      }
   }

   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException("BetaFilteredYoVariable must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getDoubleValue());
   }

   public void update(double currentPosition)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(currentPosition, false);
      }

      raw[index++] = currentPosition;
      if (index == beta)
      {
         index = 0;
         bufferFull = true;
      }

      final int size = bufferFull ? beta : index;
      double value = 0.0;
      for (int i = 0; i < size; i++)
      {
         value += raw[i];
      }

      set(value / size);
   }
}
