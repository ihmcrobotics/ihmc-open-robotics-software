package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * A discrete-time second order filter using the bilinear transform
 *
 * ----------------------------------------------------------------
 * LOW PASS:
 *
 * Y(s)                omega^2
 * ---- =  ----------------------------------
 * X(s)    s^2 + 2 * xi * omega * s + omega^2
 *
 * ----------------------------------------------------------------
 * NOTCH:
 *
 * Y(s)            s^2 + omega^2
 * ---- =  ----------------------------------
 * X(s)    s^2 + 2 * xi * omega * s + omega^2
 *
 * ----------------------------------------------------------------
 * HIGH PASS:
 *
 * Y(s)                  s^2
 * ---- =  ----------------------------------
 * X(s)    s^2 + 2 * xi * omega * s + omega^2
 *
 * -----------------------------------------------------------------
 *
 * omega = 2 * PI * naturalFrequencyInHz
 * xi = dampingRatio
 */
public class SecondOrderFilteredYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final double dt;
   private final SecondOrderFilteredYoVariableParameters parameters;
   protected final YoBoolean hasBeenCalled;
   private final YoDouble inputVariable;
   private final YoDouble[] input;
   private final YoDouble[] output;
   private final double a[];
   private final double b[];

   public SecondOrderFilteredYoVariable(String name, YoVariableRegistry registry, double dt, double naturalFrequencyInHz, double dampingRatio,
         SecondOrderFilterType filterType)
   {
      this(name, registry, dt, new SecondOrderFilteredYoVariableParameters(name, registry, naturalFrequencyInHz, dampingRatio, filterType), null);
   }

   public SecondOrderFilteredYoVariable(String name, YoVariableRegistry registry, double dt, SecondOrderFilteredYoVariableParameters parameters)
   {
      this(name, registry, dt, parameters, null);
   }

   public SecondOrderFilteredYoVariable(String name, YoVariableRegistry registry, double dt, double naturalFrequencyInHz, double dampingRatio,
         SecondOrderFilterType filterType, YoDouble inputVariable)
   {
      this(name, registry, dt, new SecondOrderFilteredYoVariableParameters(name, registry, naturalFrequencyInHz, dampingRatio, filterType), inputVariable);
   }

   public SecondOrderFilteredYoVariable(String name, YoVariableRegistry registry, double dt, SecondOrderFilteredYoVariableParameters parameters,
         YoDouble inputVariable)
   {
      super(name, registry);
      this.dt = dt;
      this.parameters = parameters;
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);
      this.inputVariable = inputVariable;
      this.input = new YoDouble[3];
      this.output = new YoDouble[3];
      this.a = new double[3];
      this.b = new double[3];
      for (int i = 0; i < 3; i++)
      {
         this.input[i] = new YoDouble(name + "input" + i, registry);
         this.output[i] = new YoDouble(name + "output" + i, registry);
      }
      reset();
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
      computeCoefficients();
   }

   @Override
   public void update()
   {
      if (inputVariable == null)
      {
         throw new NullPointerException(
               "SecondOrderFilteredYoVariable must be constructed with a non null position variable to call update(), otherwise use update(double)");
      }

      update(inputVariable.getDoubleValue());
   }

   public void update(double currentInputValue)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(currentInputValue);
         for (int i = 0; i < 3; i++)
         {
            input[i].set(currentInputValue);
            output[i].set(currentInputValue);
         }
         return;
      }

      for (int i = 2; i > 0; i--)
      {
         input[i].set(input[i - 1].getDoubleValue());
         output[i].set(output[i - 1].getDoubleValue());
      }
      input[0].set(currentInputValue);

      double currentOutputValue = 0.0;
      currentOutputValue += b[2] * input[2].getDoubleValue();
      currentOutputValue += b[1] * input[1].getDoubleValue();
      currentOutputValue += b[0] * input[0].getDoubleValue();
      currentOutputValue -= a[2] * output[2].getDoubleValue();
      currentOutputValue -= a[1] * output[1].getDoubleValue();
      currentOutputValue /= a[0];
      output[0].set(currentOutputValue);

      set(currentOutputValue);
   }

   public void setNaturalFrequencyInHz(double naturalFrequencyInHz)
   {
      parameters.getNaturalFrequencyInHz().set(Math.min(Math.max(naturalFrequencyInHz, 0.0), 1.0 / (2.0 * dt)));
      computeCoefficients();
   }

   public void setDampingRatio(double dampingRatio)
   {
      parameters.getDampingRatio().set(Math.max(dampingRatio, 0.0));
      computeCoefficients();
   }

   public boolean getHasBeenCalled()
   {
      return hasBeenCalled.getBooleanValue();
   }

   public void getFilterCoefficients(double[] b, double[] a)
   {
      if (b.length < 3)
         throw new RuntimeException("b must be of length 3 or greater");

      if (a.length < 3)
         throw new RuntimeException("a must be of length 3 or greater");

      for (int i = 0; i < 3; i++)
         b[i] = this.b[i];
      for (int i = 3; i < b.length; i++)
         b[i] = 0.0;
      for (int i = 0; i < 3; i++)
         a[i] = this.a[i];
      for (int i = 3; i < a.length; i++)
         a[i] = 0.0;
   }

   private void computeCoefficients()
   {
      double omega = 2 * Math.PI * parameters.getNaturalFrequencyInHz().getDoubleValue();
      double xi = parameters.getDampingRatio().getDoubleValue();

      switch (parameters.getFilterType())
      {
      case LOW_PASS:
         b[0] = omega * omega;
         b[1] = 2.0 * omega * omega;
         b[2] = omega * omega;
         break;
      case NOTCH:
         b[0] = 4.0 / (dt * dt) + omega * omega;
         b[1] = 2.0 * omega * omega - 8.0 / (dt * dt);
         b[2] = 4.0 / (dt * dt) + omega * omega;
         break;
      case HIGH_PASS:
         b[0] = 4.0 / (dt * dt);
         b[1] = -8.0 / (dt * dt);
         b[2] = 4.0 / (dt * dt);
         break;
      }

      a[0] = 4.0 / (dt * dt) + 4.0 / dt * xi * omega + omega * omega;
      a[1] = 2.0 * omega * omega - 8.0 / (dt * dt);
      a[2] = 4.0 / (dt * dt) - 4.0 / dt * xi * omega + omega * omega;
   }
}
