package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

/**
 * @author thutcheson
 *         </p>
 *         <p>
 *         </p>
 *         <p>
 *         A BetaFilteredYoVariable is a filtered version of an input YoVar.
 *         Either a YoVariable holding the unfiltered val is passed in to the
 *         constructor and update() is called every tick, or update(double) is
 *         called every tick. The BetaFilteredYoVariable updates it's val
 *         with the current filtered version using
 *         </p>
 *         <pre>
 *            filtered_{n} = (raw_{0} + ... + raw_{n-1} + raw_{n}) / n
 *         </pre>
 */
public class BetaFilteredYoVariable extends DoubleYoVariable
{
   private int beta;
   private int index = 0;
   @SuppressWarnings("unused")
   private final DoubleYoVariable betaVariable;

   private final DoubleYoVariable position;

   private static double raw[];

   private final BooleanYoVariable hasBeenCalled;

   public BetaFilteredYoVariable(String name, YoVariableRegistry registry, int beta)
   {
      this(name, "", registry, beta, null);
   }

   public BetaFilteredYoVariable(String name, String description, YoVariableRegistry registry, int beta)
   {
      this(name, description, registry, beta, null);
   }

   public BetaFilteredYoVariable(String name, YoVariableRegistry registry, int beta, DoubleYoVariable positionVariable)
   {
      this(name, "", registry, beta, positionVariable);
   }

   public BetaFilteredYoVariable(String name, String description, YoVariableRegistry registry, int beta, DoubleYoVariable positionVariable)
   {
      super(name, description, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      this.beta = beta;
      this.position = positionVariable;
      this.betaVariable = null;

      raw = new double[beta];

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);

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
         set(currentPosition);
      }

      raw[index++] = currentPosition;
      if (index == beta)
         index = 0;
      set(0.0);

      for (int i = 0; i < beta; i++)
      {
         set(getDoubleValue() + raw[i]);
      }

      set(getDoubleValue() / beta);
   }
}
