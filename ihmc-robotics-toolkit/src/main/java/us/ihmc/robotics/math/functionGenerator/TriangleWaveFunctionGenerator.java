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
      double frequency = getFrequency();
      double offset = getOffset();

      if (frequency == 0.0)
         return offset;

      double angle = getAngle();
      double amplitude = getAmplitude();

      if (angle <= Math.PI)
      { // Positive
         double alpha = angle / Math.PI;
         if (alpha < 0.5) // Going up
            return offset + EuclidCoreTools.interpolate(0, amplitude, alpha * 2.0);
         else // Going down
            return offset + EuclidCoreTools.interpolate(amplitude, 0, 2.0 * (alpha - 0.5));
      }
      else
      { // Negative
         double alpha = (angle - Math.PI) / Math.PI;
         if (alpha < 0.5) // Going down
            return offset + EuclidCoreTools.interpolate(0.0, -amplitude, alpha * 2.0);
         else // Going up
            return offset + EuclidCoreTools.interpolate(-amplitude, 0, 2.0 * (alpha - 0.5));
      }
   }

   @Override
   protected double computeValueDot()
   {
      double frequency = getFrequency();

      if (frequency == 0.0)
         return 0.0;

      double angle = getAngle();
      double amplitude = getAmplitude();
      return (angle <= Math.PI ? 1.0 : -1.0) * (4.0 * amplitude * frequency);
   }

   @Override
   protected double computeValueDDot()
   {
      return 0;
   }
}
