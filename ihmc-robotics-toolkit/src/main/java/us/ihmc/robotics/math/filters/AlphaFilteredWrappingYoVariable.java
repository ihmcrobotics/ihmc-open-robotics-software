package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlphaFilteredWrappingYoVariable extends AlphaFilteredYoVariable
{
   public static final double EPSILON = 1e-10;

   private double previousUnfilteredVariable;
   private final YoDouble unfilteredVariable;
   private final YoDouble unfilteredInRangeVariable;
   private final DoubleProvider alphaVariable;
   
   private final YoDouble temporaryOutputVariable;
   private final YoDouble error;
   private final double upperLimit;
   private final double lowerLimit;
   private final double range;
   
   //wrap the values in [lowerLimit ; upperLimit[
   
   public AlphaFilteredWrappingYoVariable(String name, String description, YoVariableRegistry registry, final YoDouble unfilteredVariable, DoubleProvider alphaVariable, double lowerLimit, double upperLimit)
   {
      super(name, description, registry, alphaVariable);
      this.alphaVariable = alphaVariable;
      this.upperLimit = upperLimit;
      this.lowerLimit = lowerLimit;
      this.range = upperLimit - lowerLimit;
      this.unfilteredVariable = unfilteredVariable;

      unfilteredInRangeVariable = new YoDouble(name + "UnfilteredInRangeVariable", registry);
      temporaryOutputVariable = new YoDouble(name + "TemporaryOutputVariable", registry);
      error = new YoDouble(name + "Error", registry);
      
   }
   
   @Override
   public void update()
   {
      update(unfilteredVariable.getDoubleValue());
   }

   
   @Override
   public void update(double currentPosition)
   {
      if(!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         previousUnfilteredVariable = unfilteredVariable.getDoubleValue();
         
         unfilteredVariableModulo(currentPosition);
         
         temporaryOutputVariable.set(unfilteredInRangeVariable.getDoubleValue());
         set(unfilteredInRangeVariable.getDoubleValue());
      }
      else
      {
         if (!MathTools.epsilonEquals(currentPosition, previousUnfilteredVariable, EPSILON))
         {
            previousUnfilteredVariable = currentPosition;
            
            unfilteredVariableModulo(currentPosition);
            
            //calculate the error
            double standardError = unfilteredInRangeVariable.getDoubleValue() - getDoubleValue();
            double wrappingError;
            if(unfilteredInRangeVariable.getDoubleValue() > getDoubleValue())
            {
               wrappingError = lowerLimit - getDoubleValue() + unfilteredInRangeVariable.getDoubleValue() - upperLimit;
            }
            else
            {
               wrappingError = upperLimit - getDoubleValue() + unfilteredInRangeVariable.getDoubleValue() - lowerLimit;
            }
            if(Math.abs(standardError) < Math.abs(wrappingError))
            {
               error.set(standardError);
            }
            else
            {
               error.set(wrappingError);
            }
            
            //determine if wrapping and move the input if necessary
            temporaryOutputVariable.set(getDoubleValue());
            if ((getDoubleValue() + error.getDoubleValue()) >= upperLimit)
            {
               temporaryOutputVariable.set(getDoubleValue() - range);
            }
            if ((getDoubleValue() + error.getDoubleValue()) < lowerLimit)
            {
               temporaryOutputVariable.set(getDoubleValue() + range);
            }
         }
         
         temporaryOutputVariable.set(alphaVariable.getValue() * temporaryOutputVariable.getDoubleValue() + (1.0 - alphaVariable.getValue())
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
   
   private void unfilteredVariableModulo(double currentPosition)
   {
      //handle if the input is out of range
      boolean rangeNeedsToBeChecked = true;
      unfilteredInRangeVariable.set(currentPosition);
      
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
}
