package us.ihmc.robotics.math.functionGenerator;

public class SquareWaveFunctionGenerator extends BaseFunctionGenerator
{
   @Override
   protected double computeValue()
   {
      double angle = getAngle();
      double offset = getOffset();
      double amplitude = getAmplitude();
      return offset + (angle <= Math.PI ? amplitude : -amplitude);
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
