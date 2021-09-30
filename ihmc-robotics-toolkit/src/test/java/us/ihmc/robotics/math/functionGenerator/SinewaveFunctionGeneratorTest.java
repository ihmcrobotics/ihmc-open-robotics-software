package us.ihmc.robotics.math.functionGenerator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;

public class SinewaveFunctionGeneratorTest
{
   private static final int ITERATIONS = 10000;
   private static final double EPSILON = 1.0e-12;
   private static final double FD_EPSILON = 1.0e-6;

   @Test
   public void test()
   {
      Random random = new Random(34805734);

      for (int i = 0; i < ITERATIONS; i++)
      {
         SineWaveFunctionGenerator sinewave = new SineWaveFunctionGenerator();
         double offset = RandomNumbers.nextDouble(random, 5.0);
         double amplitude = RandomNumbers.nextDouble(random, 0.0, 5.0);
         double frequency = RandomNumbers.nextDouble(random, 0.0, 5.0);
         double phase = 0.0;
         sinewave.setOffset(offset);
         sinewave.setAmplitude(amplitude);
         sinewave.setFrequency(frequency);
         sinewave.setPhase(phase);

         sinewave.resetAngle();
         assertEquals(offset, sinewave.getValue(), EPSILON);
         assertEquals(2.0 * Math.PI * frequency * amplitude, sinewave.getValueDot(), EPSILON);
         assertEquals(0.0, sinewave.getValueDDot(), EPSILON);

         double dt = 1.0e-6;
         double angle = RandomNumbers.nextDouble(random, Math.PI);
         sinewave.setAngle(angle);
         double valueDotCurrent = sinewave.getValueDot();
         double valueDDotCurrent = sinewave.getValueDDot();

         sinewave.setAngle(angle - sinewave.getAngleDot() * dt);
         double valuePrevious = sinewave.getValue();
         double valueDotPrevious = sinewave.getValueDot();

         sinewave.setAngle(angle + sinewave.getAngleDot() * dt);
         double valueNext = sinewave.getValue();
         double valueDotNext = sinewave.getValueDot();

         double valueDotCurrentFD = (valueNext - valuePrevious) / (2.0 * dt);
         double valueDDotCurrentFD = (valueDotNext - valueDotPrevious) / (2.0 * dt);
         assertEquals(valueDotCurrentFD, valueDotCurrent, FD_EPSILON, "Difference: " + Math.abs(valueDotCurrentFD - valueDotCurrent));
         assertEquals(valueDDotCurrentFD, valueDDotCurrent, FD_EPSILON, "Difference: " + Math.abs(valueDDotCurrentFD - valueDDotCurrent));
      }
   }

}
