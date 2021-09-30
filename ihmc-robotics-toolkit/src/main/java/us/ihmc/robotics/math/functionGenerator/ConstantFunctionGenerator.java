package us.ihmc.robotics.math.functionGenerator;

public class ConstantFunctionGenerator extends BaseFunctionGenerator
{
   @Override
   protected double computeValue()
   {
      return getOffset() + getAmplitude();
   }

   @Override
   protected double computeValueDot()
   {
      return 0;
   }

   @Override
   protected double computeValueDDot()
   {
      return 0;
   }
}
