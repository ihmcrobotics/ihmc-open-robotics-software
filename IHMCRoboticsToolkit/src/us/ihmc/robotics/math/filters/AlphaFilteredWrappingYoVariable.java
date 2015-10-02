package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.AngleTools;

public class AlphaFilteredWrappingYoVariable extends AlphaFilteredYoVariable
{
   public static final double EPSILON = 1e-10;

   private final DoubleYoVariable unfilteredVariable;
   private final DoubleYoVariable alphaVariable;
   
   private final DoubleYoVariable shiftedVariable;
   private final DoubleYoVariable error;
   private final double upperLimit;
   private final double lowerLimit;
   
   public AlphaFilteredWrappingYoVariable(String name, String description, YoVariableRegistry registry, final DoubleYoVariable unfilteredVariable, DoubleYoVariable alphaVariable, double lowerLimit, double upperLimit)
   {
      super(name, description, registry, alphaVariable);
      this.alphaVariable = alphaVariable;
      this.upperLimit = upperLimit;
      this.lowerLimit = lowerLimit;
      
      this.unfilteredVariable = unfilteredVariable;
      this.unfilteredVariable.addVariableChangedListener(new VariableChangedListener()
      {
         double range = AlphaFilteredWrappingYoVariable.this.upperLimit - AlphaFilteredWrappingYoVariable.this.lowerLimit;
         
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            //calculate the error
            double tempError = unfilteredVariable.getDoubleValue() - getDoubleValue();
            tempError = tempError % range;
            tempError = shiftErrorToStartOfRange(tempError, AlphaFilteredWrappingYoVariable.this.lowerLimit);
            error.set(tempError);
            
            //determine if wrapping and move the input if necessary
            shiftedVariable.set(getDoubleValue());
            if((getDoubleValue() + tempError) > AlphaFilteredWrappingYoVariable.this.upperLimit)
            {
               shiftedVariable.set(getDoubleValue() - range);
               System.out.println("over");
            }
            if((getDoubleValue() + tempError) <= AlphaFilteredWrappingYoVariable.this.lowerLimit)
            {
               shiftedVariable.set(getDoubleValue() + range);
               System.out.println("under");
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
      
      shiftedVariable = new DoubleYoVariable(name + "shiftedVariable", registry);
      error = new DoubleYoVariable(name + "Error", registry);
   }

   @Override
   public void update()
   {
      
   }
   
   public double getError()
   {
      return error.getDoubleValue();
   }
   
   public double getShiftedVariable()
   {
      return shiftedVariable.getDoubleValue();
   }
   
   public static void main (String args[])
   {
      System.out.println(AngleTools.computeAngleDifferenceMinusPiToPi(Math.toRadians(45.0 + 90.0), Math.toRadians(-45.0 + 90.0)));
      System.out.println(AngleTools.computeAngleDifferenceMinusPiToPi(Math.toRadians(-45.0 + 90.0), Math.toRadians(45.0 + 90.0)));
   }
   
   //listener on the unfiltered to calculate the error, check if there will be a wrapping, and move the correctedUnfiltered to the proper spot
}
