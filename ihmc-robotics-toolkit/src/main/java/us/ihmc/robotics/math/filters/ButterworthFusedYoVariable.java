package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable.ButterworthFilterType;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * <p>
 * A YoButterworthFusedVariable takes two inputs measuring the same signal and filters them
 * together. One of the signals is considered accurate at low frequencies and the other accurate at
 * high frequencies. The user also supplies an alpha which relates the relative confidence
 * frequencies. Either the underlying inputs must be passed in to a constructor as YoVariables and
 * update() called every tick or update(double, double) must be called every tick with the
 * variables. The YoAlphaFilteredVariable updates it's val with the current filtered version using a
 * low pass and a high pass Butterworth Filter
 * </p>
 * 
 * <pre>
 *            steady_state_offset_{n} = steady_state_offset_{n-1} + alpha * (fused_{n-1} - slow_signal_{n})
 *            fused_{n} = fast_signal_{n} - steady_state_offset_{n}
 * </pre>
 * <p>
 * A lower alpha means that the relative confidence frequency of the slowSignal is lower.
 * </p>
 */
public class ButterworthFusedYoVariable extends YoDouble
{
   private final ButterworthFilteredYoVariable lowPassFilteredSlowVariable, highPassFilteredFastVariable;

   public ButterworthFusedYoVariable(String name, YoRegistry registry, double alpha)
   {
      this(name, registry, alpha, null, null);
   }

   public ButterworthFusedYoVariable(String name, YoRegistry registry, double alpha, DoubleProvider slowSignal, DoubleProvider fastSignal)
   {
      this(name, registry, AlphaFilteredYoVariable.createAlphaYoDouble(name, alpha, registry), slowSignal, fastSignal);
   }

   public ButterworthFusedYoVariable(String name, YoRegistry registry, DoubleProvider alphaVariable)
   {
      this(name, registry, alphaVariable, null, null);
   }

   public ButterworthFusedYoVariable(String name, YoRegistry registry, DoubleProvider alphaVariable, DoubleProvider slowSignal, DoubleProvider fastSignal)
   {
      super(name, registry);

      lowPassFilteredSlowVariable = new ButterworthFilteredYoVariable(name + "LowPass", registry, alphaVariable, slowSignal, ButterworthFilterType.LOW_PASS);
      highPassFilteredFastVariable = new ButterworthFilteredYoVariable(name + "HighPass", registry, alphaVariable, fastSignal, ButterworthFilterType.HIGH_PASS);
      reset();
   }

   public void reset()
   {
      lowPassFilteredSlowVariable.reset();
      highPassFilteredFastVariable.reset();
   }

   public void update()
   {
      lowPassFilteredSlowVariable.update();
      highPassFilteredFastVariable.update();

      set(lowPassFilteredSlowVariable.getDoubleValue() + highPassFilteredFastVariable.getDoubleValue());
   }

   public void update(double slowSignalVal, double fastSignalVal)
   {
      lowPassFilteredSlowVariable.update(slowSignalVal);
      highPassFilteredFastVariable.update(fastSignalVal);

      set(lowPassFilteredSlowVariable.getDoubleValue() + highPassFilteredFastVariable.getDoubleValue());
   }
}
