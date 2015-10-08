package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class AlphaFilteredWrappingYoVariable extends AlphaFilteredYoVariable
{
   public static final double EPSILON = 1e-10;

   private double previousUnfilteredVariable;
   private final DoubleYoVariable unfilteredVariable;
   private final DoubleYoVariable unfilteredInRangeVariable;
   private final DoubleYoVariable alphaVariable;
   
   private final DoubleYoVariable temporaryOutputVariable;
   private final DoubleYoVariable error;
   private final double upperLimit;
   private final double lowerLimit;
   private final double range;
   
   //wrap the values in [lowerLimit ; upperLimit[
   
   public AlphaFilteredWrappingYoVariable(String name, String description, YoVariableRegistry registry, final DoubleYoVariable unfilteredVariable, DoubleYoVariable alphaVariable, double lowerLimit, double upperLimit)
   {
      super(name, description, registry, alphaVariable);
      this.alphaVariable = alphaVariable;
      this.upperLimit = upperLimit;
      this.lowerLimit = lowerLimit;
      this.range = upperLimit - lowerLimit;
      this.unfilteredVariable = unfilteredVariable;

      unfilteredInRangeVariable = new DoubleYoVariable(name + "UnfilteredInRangeVariable", registry);
      temporaryOutputVariable = new DoubleYoVariable(name + "TemporaryOutputVariable", registry);
      error = new DoubleYoVariable(name + "Error", registry);
      
   }
   
   @Override
   public void update()
   {
      if(!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         temporaryOutputVariable.set(unfilteredVariable.getDoubleValue());
         previousUnfilteredVariable = unfilteredVariable.getDoubleValue();
         
         unfilteredVariableModulo();
         set(unfilteredInRangeVariable.getDoubleValue());
      }
      else
      {
         if (!MathTools.epsilonEquals(unfilteredVariable.getDoubleValue(), previousUnfilteredVariable, EPSILON))
         {
            previousUnfilteredVariable = unfilteredVariable.getDoubleValue();
            
            unfilteredVariableModulo();
            
            //calculate the error
            double tempError = unfilteredInRangeVariable.getDoubleValue() - getDoubleValue();
            tempError = tempError % range;
            tempError = shiftErrorToStartOfRange(tempError, lowerLimit);
            error.set(tempError);

            //determine if wrapping and move the input if necessary
            temporaryOutputVariable.set(getDoubleValue());
            if ((getDoubleValue() + tempError) >= upperLimit)
            {
               temporaryOutputVariable.set(getDoubleValue() - range);
            }
            if ((getDoubleValue() + tempError) < lowerLimit)
            {
               temporaryOutputVariable.set(getDoubleValue() + range);
            }
         }
         
         temporaryOutputVariable.set(alphaVariable.getDoubleValue() * temporaryOutputVariable.getDoubleValue() + (1.0 - alphaVariable.getDoubleValue())
               * unfilteredInRangeVariable.getDoubleValue());

         if (temporaryOutputVariable.getDoubleValue() > upperLimit + EPSILON)
         {
            set(temporaryOutputVariable.getDoubleValue() - range);
         }
         else if (temporaryOutputVariable.getDoubleValue() <= lowerLimit - EPSILON)
         {
            set(temporaryOutputVariable.getDoubleValue() + range);
         }
         else
         {
            set(temporaryOutputVariable.getDoubleValue());
         }
      }
   }

   private void unfilteredVariableModulo()
   {
      //handle if the input is out of range
      boolean rangeNeedsToBeChecked = true;
      unfilteredInRangeVariable.set(unfilteredVariable.getDoubleValue());
      
      while(rangeNeedsToBeChecked)
      {
         rangeNeedsToBeChecked = false;
         if (unfilteredInRangeVariable.getDoubleValue() >= upperLimit)
         {
            unfilteredInRangeVariable.set(unfilteredInRangeVariable.getDoubleValue() - range);
            rangeNeedsToBeChecked = true;
         }
         if (unfilteredInRangeVariable.getDoubleValue() < lowerLimit)
         {
            unfilteredInRangeVariable.set(unfilteredInRangeVariable.getDoubleValue() + range);
            rangeNeedsToBeChecked = true;
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
}
