package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * @author jrebula
 *         <p>
 *         A YoFusedVariable takes two inputs measuring the same signal and
 *         filters them together. One of the signals is considered accurate
 *         at low frequencies and the other accurate at high frequencies. The
 *         user also supplies an alpha which relates the relative confidence
 *         frequencies. Either the underlying inputs must be passed in to a
 *         constructor as YoVariables and update() called every tick or
 *         update(double, double) must be called every tick with the variables.
 *         The YoAlphaFilteredVariable updates it's val with the current filtered
 *         version using
 *         </p>
 *         <pre>
 *            steady_state_offset_{n} = steady_state_offset_{n-1} + alpha * (fused_{n-1} - slow_signal_{n})
 *            fused_{n} = fast_signal_{n} - steady_state_offset_{n}
 *         </pre>
 *         <p>
 *         A lower alpha means that the relative confidence frequency of the
 *         slowSignal is lower.
 *         </p>
 */
public class AlphaFusedYoVariable extends YoDouble
{
   private final double alpha;

   private final DoubleProvider alphaVariable;

   private final YoDouble slowSignal;
   private final YoDouble fastSignal;
   private final YoDouble steadyStateOffset;

   private final YoBoolean hasBeenCalled;

   public AlphaFusedYoVariable(String name, YoVariableRegistry yoVariableRegistry, double alpha)
   {
      super(name, yoVariableRegistry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", yoVariableRegistry);

      this.alpha = alpha;
      this.alphaVariable = null;

      this.slowSignal = null;
      this.fastSignal = null;

      steadyStateOffset = new YoDouble(name + "_off", yoVariableRegistry);

      reset();
   }

   public AlphaFusedYoVariable(String name, YoVariableRegistry yoVariableRegistry, double alpha, YoDouble slowSignal, YoDouble fastSignal)
   {
      super(name, yoVariableRegistry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", yoVariableRegistry);

      this.alpha = alpha;
      this.alphaVariable = null;

      this.slowSignal = slowSignal;
      this.fastSignal = fastSignal;

      steadyStateOffset = new YoDouble(name + "_off", yoVariableRegistry);

      reset();
   }

   public AlphaFusedYoVariable(String name, YoVariableRegistry yoVariableRegistry, DoubleProvider alphaVariable, YoDouble slowSignal,
         YoDouble fastSignal)
   {
      super(name, yoVariableRegistry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", yoVariableRegistry);

      this.alpha = 0.0;
      this.alphaVariable = alphaVariable;

      this.slowSignal = slowSignal;
      this.fastSignal = fastSignal;

      steadyStateOffset = new YoDouble(name + "_off", yoVariableRegistry);

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
      steadyStateOffset.set(0.0);
   }

   public void update()
   {
      if ((slowSignal == null) || (fastSignal == null))
      {
         throw new NullPointerException("YoAlphaFusedVariable must be constructed with non null "
               + "signal variables to call update(), otherwise use update(double, double)");
      }

      update(slowSignal.getDoubleValue(), fastSignal.getDoubleValue());
   }

   public void update(double slowSignalVal, double fastSignalVal)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         steadyStateOffset.set(0.0);
         set(slowSignalVal);
      }

      double alphaValue;

      if (alphaVariable == null)
      {
         alphaValue = alpha;
      }
      else
      {
         alphaValue = alphaVariable.getValue();
      }

      steadyStateOffset.set(steadyStateOffset.getDoubleValue() + alphaValue * (getDoubleValue() - slowSignalVal));
      set(fastSignalVal - steadyStateOffset.getDoubleValue());
   }
}
