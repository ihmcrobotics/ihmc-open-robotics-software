package us.ihmc.robotics.math.functionGenerator;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;

public class TriangleWaveFunctionGeneratorTest
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
         TriangleWaveFunctionGenerator triangleWave = new TriangleWaveFunctionGenerator();
         double offset = RandomNumbers.nextDouble(random, 5.0);
         double amplitude = RandomNumbers.nextDouble(random, 0.0, 5.0);
         double frequency = RandomNumbers.nextDouble(random, 0.0, 5.0);
         double phase = 0.0;
         triangleWave.setOffset(offset);
         triangleWave.setAmplitude(amplitude);
         triangleWave.setFrequency(frequency);
         triangleWave.setPhase(phase);

         triangleWave.resetAngle();
         assertEquals(offset - amplitude, triangleWave.getValue(), EPSILON);
//         assertEquals(8.0 * Math.PI * frequency * amplitude, triangleWave.getValueDot(), EPSILON);
         assertEquals(0.0, triangleWave.getValueDDot(), EPSILON);

         double dt = 1.0e-6;
         double angle = RandomNumbers.nextDouble(random, Math.PI - 0.01);
         triangleWave.setAngle(angle);
         double valueDotCurrent = triangleWave.getValueDot();
         double valueDDotCurrent = triangleWave.getValueDDot();

         triangleWave.setAngle(angle - triangleWave.getAngleDot() * dt);
         double valuePrevious = triangleWave.getValue();
         double valueDotPrevious = triangleWave.getValueDot();

         triangleWave.setAngle(angle + triangleWave.getAngleDot() * dt);
         double valueNext = triangleWave.getValue();
         double valueDotNext = triangleWave.getValueDot();

         double valueDotCurrentFD = (valueNext - valuePrevious) / (2.0 * dt);
         double valueDDotCurrentFD = (valueDotNext - valueDotPrevious) / (2.0 * dt);
         assertEquals(valueDotCurrentFD, valueDotCurrent, FD_EPSILON, "Difference: " + Math.abs(valueDotCurrentFD - valueDotCurrent));
         assertEquals(valueDDotCurrentFD, valueDDotCurrent, FD_EPSILON, "Difference: " + Math.abs(valueDDotCurrentFD - valueDDotCurrent));
      }
   }

}
