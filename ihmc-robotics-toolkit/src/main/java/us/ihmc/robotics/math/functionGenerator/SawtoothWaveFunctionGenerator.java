package us.ihmc.robotics.math.functionGenerator;

import us.ihmc.euclid.tools.EuclidCoreTools;

public class SawtoothWaveFunctionGenerator extends BaseFunctionGenerator
{

   @Override
   protected double computeValue()
   {
      double alpha = getAngle() / twoPi;
      double amplitude = getAmplitude();
      double offset = getOffset();
      return offset + EuclidCoreTools.interpolate(-amplitude, amplitude, alpha);
   }

   @Override
   protected double computeValueDot()
   {
      double amplitude = getAmplitude();
      double frequency = getFrequency();
      return 2.0 * amplitude * frequency;
   }

   @Override
   protected double computeValueDDot()
   {
      return 0;
   }
}
