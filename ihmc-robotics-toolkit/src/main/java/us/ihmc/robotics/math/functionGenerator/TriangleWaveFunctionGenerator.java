package us.ihmc.robotics.math.functionGenerator;

import us.ihmc.euclid.tools.EuclidCoreTools;

public class TriangleWaveFunctionGenerator extends BaseFunctionGenerator
{
   public TriangleWaveFunctionGenerator()
   {
   }

   @Override
   protected double computeValue()
   {
      double angle = getAngle();
      double offset = getOffset();
      double amplitude = getAmplitude();

      if (angle <= Math.PI)
      { // Going up
         double alpha = angle / Math.PI;
         return offset + EuclidCoreTools.interpolate(-amplitude, amplitude, alpha);
      }
      else
      { // Going down
         double alpha = (angle - Math.PI) / Math.PI;
         return offset + EuclidCoreTools.interpolate(amplitude, -amplitude, alpha);
      }
   }

   @Override
   protected double computeValueDot()
   {
      double angle = getAngle();
      double amplitude = getAmplitude();
      double frequency = getFrequency();
      return (angle <= Math.PI ? 1.0 : -1.0) * (4.0 * amplitude * frequency);
   }

   @Override
   protected double computeValueDDot()
   {
      return 0;
   }
}
