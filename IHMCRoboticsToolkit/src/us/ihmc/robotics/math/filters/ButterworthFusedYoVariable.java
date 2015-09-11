package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable.ButterworthFilterType;


/**
 *         <p>
 *         A YoButterworthFusedVariable takes two inputs measuring the same signal and
 *         filters them together. One of the signals is considered accurate
 *         at low frequencies and the other accurate at high frequencies. The
 *         user also supplies an alpha which relates the relative confidence
 *         frequencies. Either the underlying inputs must be passed in to a
 *         constructor as YoVariables and update() called every tick or
 *         update(double, double) must be called every tick with the variables.
 *         The YoAlphaFilteredVariable updates it's val with the current filtered
 *         version using a low pass and a high pass Butterworth Filter
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
public class ButterworthFusedYoVariable extends DoubleYoVariable
{
   private final ButterworthFilteredYoVariable lowPassFilteredSlowVariable, highPassFilteredFastVariable;

   public ButterworthFusedYoVariable(String name, YoVariableRegistry yoVariableRegistry, double alpha)
   {
      super(name, yoVariableRegistry);

      lowPassFilteredSlowVariable = new ButterworthFilteredYoVariable(name + "lowPass", yoVariableRegistry, alpha, ButterworthFilterType.LOW_PASS);
      highPassFilteredFastVariable = new ButterworthFilteredYoVariable(name + "highPass", yoVariableRegistry, alpha, ButterworthFilterType.HIGH_PASS);
   }

   public ButterworthFusedYoVariable(String name, YoVariableRegistry yoVariableRegistry, double alpha, DoubleYoVariable slowSignal, DoubleYoVariable fastSignal)
   {
      super(name, yoVariableRegistry);

      lowPassFilteredSlowVariable = new ButterworthFilteredYoVariable(name + "lowPass", yoVariableRegistry, alpha, slowSignal, ButterworthFilterType.LOW_PASS);
      highPassFilteredFastVariable = new ButterworthFilteredYoVariable(name + "highPass", yoVariableRegistry, alpha, fastSignal,
            ButterworthFilterType.HIGH_PASS);

      reset();
   }

   public ButterworthFusedYoVariable(String name, YoVariableRegistry yoVariableRegistry, DoubleYoVariable alphaVariable, DoubleYoVariable slowSignal,
         DoubleYoVariable fastSignal)
   {
      super(name, yoVariableRegistry);

      lowPassFilteredSlowVariable = new ButterworthFilteredYoVariable(name + "lowPass", yoVariableRegistry, alphaVariable, slowSignal,
            ButterworthFilterType.LOW_PASS);
      highPassFilteredFastVariable = new ButterworthFilteredYoVariable(name + "highPass", yoVariableRegistry, alphaVariable, fastSignal,
            ButterworthFilterType.HIGH_PASS);
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
