package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * @author jrebula
 *         </p>
 *         <p>
 *         LittleDogVersion06: us.ihmc.LearningLocomotion.Version06.util.YoAlphaFilteredVariable,
 *         9:34:00 AM, Aug 29, 2006
 *         </p>
 *         <p>
 *         A YoAlphaFilteredVariable is a filtered version of an input YoVar. Either a YoVariable
 *         holding the unfiltered val is passed in to the constructor and update() is called every
 *         tick, or update(double) is called every tick. The YoAlphaFilteredVariable updates it's
 *         val with the current filtered version using
 *         </p>
 * 
 *         <pre>
 *            filtered_{n} = alpha * filtered_{n-1} + 1/2 * (1 - alpha) * (raw_{n} + raw{n-1}}
 *         </pre>
 */
public class ButterworthFilteredYoVariable extends YoDouble
{
   public enum ButterworthFilterType
   {
      LOW_PASS, HIGH_PASS
   }

   private final DoubleProvider alphaVariable;
   private final ButterworthFilterType butterworthFilterType;

   private final DoubleProvider position;
   private final YoDouble previousInput;

   private final YoBoolean hasBeenCalled;

   public ButterworthFilteredYoVariable(String name, YoRegistry registry, double alpha, ButterworthFilterType butterworthFilterType)
   {
      this(name, registry, AlphaFilteredYoVariable.createAlphaYoDouble(name, alpha, registry), butterworthFilterType);
   }

   public ButterworthFilteredYoVariable(String name,
                                        YoRegistry registry,
                                        double alpha,
                                        DoubleProvider positionVariable,
                                        ButterworthFilterType butterworthFilterType)
   {
      this(name, registry, AlphaFilteredYoVariable.createAlphaYoDouble(name, alpha, registry), positionVariable, butterworthFilterType);
   }

   public ButterworthFilteredYoVariable(String name, YoRegistry registry, DoubleProvider alphaVariable, ButterworthFilterType butterworthFilterType)
   {
      this(name, registry, alphaVariable, null, butterworthFilterType);
   }

   public ButterworthFilteredYoVariable(String name,
                                        YoRegistry registry,
                                        DoubleProvider alphaVariable,
                                        DoubleProvider positionVariable,
                                        ButterworthFilterType butterworthFilterType)
   {
      super(name, registry);

      this.alphaVariable = alphaVariable;
      this.butterworthFilterType = butterworthFilterType;

      this.position = positionVariable;
      this.previousInput = new YoDouble(name + "_prevIn", registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

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
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getValue());
   }

   public void update(double currentInput)
   {
      if (!hasBeenCalled.getValue())
      {
         hasBeenCalled.set(true);

         if (this.butterworthFilterType == ButterworthFilterType.HIGH_PASS)
         {
            set(0.0);
         }
         else
         {
            set(currentInput);
         }
      }
      else
      {
         double alpha = alphaVariable.getValue();

         switch (butterworthFilterType)
         {
            case LOW_PASS:
            {
               set(alpha * getValue() + 0.5 * (1.0 - alpha) * (currentInput + previousInput.getValue()));
               break;
            }
            case HIGH_PASS:
            {
               set(alpha * getValue() + 0.5 * (1.0 + alpha) * (currentInput - previousInput.getValue()));
               break;
            }
         }
      }

      previousInput.set(currentInput);
   }

   public static double computeAlphaGivenBreakFrequency(double breakFrequencyInHertz, double dt)
   {
      if (Double.isInfinite(breakFrequencyInHertz))
         return 0.0;

      double samplingFrequency = 1.0 / dt;

      if (breakFrequencyInHertz > 0.25 * samplingFrequency)
         return 0.0;

      double tanOmegaBreak = Math.tan(Math.PI * breakFrequencyInHertz * dt);
      return MathTools.clamp((1.0 - tanOmegaBreak) / (1.0 + tanOmegaBreak), 0.0, 1.0);
   }

   public static double computeBreakFrequencyGivenAlpha(double alpha, double dt)
   {
      double beta = (1.0 - alpha) / (1.0 + alpha);
      return Math.max(Math.atan(beta) / (dt * Math.PI), 0.0);
   }
}
