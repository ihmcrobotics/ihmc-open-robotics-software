package us.ihmc.sensorProcessing.diagnostic;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.DelayedDoubleYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class DelayEstimatorBetweenTwoSignalsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testPerfectSignal() throws Exception
   {
      double epsilon = 1.0e-15;

      YoVariableRegistry registry = new YoVariableRegistry("Blop");
      double numberOfTicks = 10000;
      double dt = 0.001;
      DoubleYoVariable yoTime = new DoubleYoVariable("time", registry);

      YoFunctionGenerator functionGenerator = new YoFunctionGenerator("foo", yoTime, registry);
      functionGenerator.setChirpFrequencyMaxHz(40.0);
      functionGenerator.setAmplitude(1.0);
      functionGenerator.setResetTime(numberOfTicks * dt);
      functionGenerator.setMode(YoFunctionGeneratorMode.CHIRP_LINEAR);

      DoubleYoVariable referenceSignal = new DoubleYoVariable("referenceSignal", registry);
      int expectedDelayInTicks = 12;
      DelayedDoubleYoVariable delayedSignal = new DelayedDoubleYoVariable("delayedSignal", "", referenceSignal, expectedDelayInTicks, registry);

      DelayEstimatorBetweenTwoSignals delayEstimatorBetweenTwoSignals = new DelayEstimatorBetweenTwoSignals("delayedSignal", referenceSignal, delayedSignal, dt, registry);
      delayEstimatorBetweenTwoSignals.setEstimationParameters(25, 25, 100);
      delayEstimatorBetweenTwoSignals.enable();

      for (int i = 0; i < numberOfTicks; i++)
      {
         double sine = functionGenerator.getValue();
         referenceSignal.set(sine);
         delayedSignal.update();
         delayEstimatorBetweenTwoSignals.update();

         yoTime.add(dt);

         if (delayEstimatorBetweenTwoSignals.isEstimatingDelay())
         {
            assertEquals(1.0, delayEstimatorBetweenTwoSignals.getCorrelationCoefficient(), epsilon);
            assertEquals(expectedDelayInTicks * dt, delayEstimatorBetweenTwoSignals.getEstimatedDelay(), epsilon);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testWithShiftedSignal() throws Exception
   {
      double epsilon = 1.0e-15;

      YoVariableRegistry registry = new YoVariableRegistry("Blop");
      double numberOfTicks = 10000;
      double dt = 0.001;
      DoubleYoVariable yoTime = new DoubleYoVariable("time", registry);

      YoFunctionGenerator functionGenerator = new YoFunctionGenerator("foo", yoTime, registry);
      functionGenerator.setChirpFrequencyMaxHz(40.0);
      functionGenerator.setAmplitude(1.0);
      functionGenerator.setResetTime(numberOfTicks * dt);
      functionGenerator.setMode(YoFunctionGeneratorMode.CHIRP_LINEAR);

      DoubleYoVariable referenceSignal = new DoubleYoVariable("referenceSignal", registry);
      int expectedDelayInTicks = 12;
      DelayedDoubleYoVariable delayedSignal = new DelayedDoubleYoVariable("delayedSignal", "", referenceSignal, expectedDelayInTicks, registry);
      DoubleYoVariable shiftedDelayedSignal = new DoubleYoVariable("shiftedDelayedSignal", registry);

      DelayEstimatorBetweenTwoSignals delayEstimatorBetweenTwoSignals = new DelayEstimatorBetweenTwoSignals("delayedSignal", referenceSignal, shiftedDelayedSignal, dt, registry);
      delayEstimatorBetweenTwoSignals.setEstimationParameters(25, 25, 100);
      delayEstimatorBetweenTwoSignals.enable();

      for (int i = 0; i < numberOfTicks; i++)
      {
         double sine = functionGenerator.getValue();
         referenceSignal.set(sine);
         delayedSignal.update();
         shiftedDelayedSignal.set(delayedSignal.getDoubleValue() + 2.5);
         delayEstimatorBetweenTwoSignals.update();

         yoTime.add(dt);

         if (delayEstimatorBetweenTwoSignals.isEstimatingDelay())
         {
            assertEquals(1.0, delayEstimatorBetweenTwoSignals.getCorrelationCoefficient(), epsilon);
            assertEquals(expectedDelayInTicks * dt, delayEstimatorBetweenTwoSignals.getEstimatedDelay(), epsilon);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testNoisySignal() throws Exception
   {
      Random random = new Random(5616515L);

      YoVariableRegistry registry = new YoVariableRegistry("Blop");
      double numberOfTicks = 10000;
      double dt = 0.001;
      DoubleYoVariable yoTime = new DoubleYoVariable("time", registry);

      DoubleYoVariable referenceSignal = new DoubleYoVariable("referenceSignal", registry);
      int expectedDelayInTicks = 12;
      DelayedDoubleYoVariable delayedSignal = new DelayedDoubleYoVariable("delayedSignal", "", referenceSignal, expectedDelayInTicks, registry);
      DoubleYoVariable noisyDelayedSignal = new DoubleYoVariable("noisyDelayedSignal", registry);

      DelayEstimatorBetweenTwoSignals delayEstimatorBetweenTwoSignals = new DelayEstimatorBetweenTwoSignals("delayedSignal", referenceSignal, noisyDelayedSignal, dt, registry);
      delayEstimatorBetweenTwoSignals.setEstimationParameters(25, 25, 100);
      delayEstimatorBetweenTwoSignals.enable();

      for (int i = 0; i < numberOfTicks; i++)
      {
         double sine = 0.5 * Math.sin(2.0 * Math.PI * 16.0 * i * dt);
         referenceSignal.set(sine);
         delayedSignal.update();
         noisyDelayedSignal.set(delayedSignal.getDoubleValue());
         noisyDelayedSignal.add(0.5 * (random.nextDouble() - 0.5) + 1.5);
         delayEstimatorBetweenTwoSignals.update();

         yoTime.add(dt);

         if (delayEstimatorBetweenTwoSignals.isEstimatingDelay())
         {
            assertTrue("Correlation seems abnormally low", delayEstimatorBetweenTwoSignals.getCorrelationCoefficient() > 0.85);
            assertTrue("Bad delay estimation", (int)(delayEstimatorBetweenTwoSignals.getEstimatedDelay() / dt) <= expectedDelayInTicks + 2);
            assertTrue("Bad delay estimation", (int)(delayEstimatorBetweenTwoSignals.getEstimatedDelay() / dt) >= expectedDelayInTicks - 2);
         }
      }
   }
}
