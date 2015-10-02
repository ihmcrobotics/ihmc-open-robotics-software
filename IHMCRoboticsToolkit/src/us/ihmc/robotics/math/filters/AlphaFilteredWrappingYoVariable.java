package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class AlphaFilteredWrappingYoVariable extends AlphaFilteredYoVariable
{
   public static final double EPSILON = 1e-10;

   private final DoubleYoVariable unfilteredVariable;
   private final DoubleYoVariable alphaVariable;

   private final DoubleYoVariable temporaryOutputVariable;
   private final DoubleYoVariable error;
   private final double upperLimit;
   private final double lowerLimit;
   private final double range;

   public AlphaFilteredWrappingYoVariable(String name, String description, YoVariableRegistry registry, final DoubleYoVariable unfilteredVariable,
         DoubleYoVariable alphaVariable, double lowerLimit, double upperLimit)
   {
      super(name, description, registry, alphaVariable);
      this.alphaVariable = alphaVariable;
      this.upperLimit = upperLimit;
      this.lowerLimit = lowerLimit;
      this.range = upperLimit - lowerLimit;

      this.unfilteredVariable = unfilteredVariable;
      this.unfilteredVariable.addVariableChangedListener(new VariableChangedListener()
      {
         double range = AlphaFilteredWrappingYoVariable.this.upperLimit - AlphaFilteredWrappingYoVariable.this.lowerLimit;

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            //calculate the error
            if (hasBeenCalled.getBooleanValue())
            {
               double tempError = unfilteredVariable.getDoubleValue() - getDoubleValue();
               tempError = tempError % range;
               tempError = shiftErrorToStartOfRange(tempError, AlphaFilteredWrappingYoVariable.this.lowerLimit);
               error.set(tempError);

               //determine if wrapping and move the input if necessary
               temporaryOutputVariable.set(getDoubleValue());
               if ((getDoubleValue() + tempError) > AlphaFilteredWrappingYoVariable.this.upperLimit)
               {
                  temporaryOutputVariable.set(getDoubleValue() - range);
               }
               if ((getDoubleValue() + tempError) <= AlphaFilteredWrappingYoVariable.this.lowerLimit)
               {
                  temporaryOutputVariable.set(getDoubleValue() + range);
               }
            }
         }

         private double shiftErrorToStartOfRange(double error, double startOfRange)
         {
            double ret = error;
            startOfRange = startOfRange - EPSILON;

            if (error < startOfRange)
            {
               ret = error + Math.ceil((startOfRange - error) / range) * range;
            }

            if (error >= (startOfRange + range))
            {
               ret = error - Math.floor((error - startOfRange) / range) * range;
            }

            return ret;
         }
      });

      temporaryOutputVariable = new DoubleYoVariable(name + "TemporaryOutputVariable", registry);
      error = new DoubleYoVariable(name + "Error", registry);
   }

   @Override
   public void update()
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         temporaryOutputVariable.set(unfilteredVariable.getDoubleValue());
         set(unfilteredVariable.getDoubleValue());
      }
      else
      {
         temporaryOutputVariable.set(alphaVariable.getDoubleValue() * temporaryOutputVariable.getDoubleValue() + (1.0 - alphaVariable.getDoubleValue()) * unfilteredVariable.getDoubleValue());

         set(temporaryOutputVariable.getDoubleValue());
         if (temporaryOutputVariable.getDoubleValue() > upperLimit)
         {
            set(temporaryOutputVariable.getDoubleValue() - range);
         }
         if (temporaryOutputVariable.getDoubleValue() <= lowerLimit)
         {
            set(temporaryOutputVariable.getDoubleValue() + range);
         }
      }
   }
}
