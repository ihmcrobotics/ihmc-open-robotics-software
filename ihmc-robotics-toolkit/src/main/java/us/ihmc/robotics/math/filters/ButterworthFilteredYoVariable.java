package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 *         <p>
 *         A YoAlphaFilteredVariable is a filtered version of an input YoVar.
 *         Either a YoVariable holding the unfiltered val is passed in to the
 *         constructor and update() is called every tick, or update(double) is
 *         called every tick. The YoAlphaFilteredVariable updates it's val
 *         with the current filtered version using
 *         </p>
 *         <pre>
 *            filtered_{n} = alpha * filtered_{n-1} + 1/2 * (1 - alpha) * (raw_{n} + raw{n-1}}
 *         </pre>
 */
public class ButterworthFilteredYoVariable extends YoDouble
{
   private final double alpha;
   private final YoDouble alphaVariable;

   private final YoDouble position;
   private final YoDouble previousInput;

   private final YoBoolean hasBeenCalled;

   private final ButterworthFilterType butterworthFilterType;

   public ButterworthFilteredYoVariable(String name, YoRegistry registry, double alpha, ButterworthFilterType butterworthFilterType)
   {
      super(name, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      this.alpha = alpha;
      this.alphaVariable = null;

      this.position = null;
      this.previousInput = new YoDouble(name + "_prevIn", registry);

      this.butterworthFilterType = butterworthFilterType;

      reset();
   }

   public ButterworthFilteredYoVariable(String name, YoRegistry registry, double alpha, YoDouble positionVariable,
         ButterworthFilterType butterworthFilterType)
   {
      super(name, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      this.alpha = alpha;
      this.alphaVariable = null;

      this.position = positionVariable;
      this.previousInput = new YoDouble(name + "_prevIn", registry);

      this.butterworthFilterType = butterworthFilterType;

      reset();
   }

   public ButterworthFilteredYoVariable(String name, YoRegistry registry, YoDouble alphaVariable, ButterworthFilterType butterworthFilterType)
   {
      super(name, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      this.alpha = 0.0;
      this.alphaVariable = alphaVariable;

      this.position = null;
      this.previousInput = new YoDouble(name + "_prevIn", registry);

      this.butterworthFilterType = butterworthFilterType;

      reset();
   }

   public ButterworthFilteredYoVariable(String name, YoRegistry registry, YoDouble alphaVariable, YoDouble positionVariable,
         ButterworthFilterType butterworthFilterType)
   {
      super(name, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      this.alpha = 0.0;
      this.alphaVariable = alphaVariable;

      this.position = positionVariable;
      this.previousInput = new YoDouble(name + "_prevIn", registry);

      this.butterworthFilterType = butterworthFilterType;

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
         throw new NullPointerException("YoButterworthFilteredVariable must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getDoubleValue());
   }

   public void update(double currentInput)
   {
      if (!hasBeenCalled.getBooleanValue())
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

         previousInput.set(currentInput);
      }

      double alphaToUse;

      if (alphaVariable == null)
      {
         alphaToUse = alpha;
      }
      else
      {
         alphaToUse = alphaVariable.getDoubleValue();
      }

      switch (butterworthFilterType)
      {
         case LOW_PASS:
         {
            set(alphaToUse * getDoubleValue() + 0.5 * (1.0 - alphaToUse) * (currentInput + previousInput.getDoubleValue()));

            break;
         }

         case HIGH_PASS:
         {
            set(alphaToUse * getDoubleValue() + 0.5 * (1.0 + alphaToUse) * (currentInput - previousInput.getDoubleValue()));

            break;
         }

      }

      previousInput.set(currentInput);
   }

   public enum ButterworthFilterType
   {
      LOW_PASS, HIGH_PASS
   }

}
