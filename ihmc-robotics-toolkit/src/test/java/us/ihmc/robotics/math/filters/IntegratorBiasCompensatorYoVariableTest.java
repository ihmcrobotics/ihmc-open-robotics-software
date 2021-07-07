package us.ihmc.robotics.math.filters;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.yoVariables.variable.YoDouble;

public class IntegratorBiasCompensatorYoVariableTest
{
   @Test
   public void test_PerfectMeasurements_Sinewave()
   {
      double epsilon = 1.0e-3;
      Random random = new Random(23524);

      double kp = 0.1;
      double ki = 0.01;
      YoDouble position = new YoDouble("position", null);
      position.set(RandomNumbers.nextDouble(random, 100.0));
      YoDouble rate = new YoDouble("rate", null);
      double dt = 0.001;
      IntegratorBiasCompensatorYoVariable filter = new IntegratorBiasCompensatorYoVariable("blop", null, kp, ki, position, rate, dt);

      double angle = 0.0;
      double angleRate = 0.0;
      double amp = 10.0;
      double freq = 1.0;

      // First loop is warmup to let filter estimate bias.
      for (int i = 0; i < 100000; i++)
      {
         angleRate = 2.0 * Math.PI * freq;
         angle += angleRate * dt;

         rate.set(angleRate * amp * Math.cos(angle));
         position.set(amp * Math.sin(angle));
         filter.update();
      }

      for (int i = 0; i < 5000; i++)
      {
         angleRate = 2.0 * Math.PI * freq;
         angle += angleRate * dt;

         rate.set(angleRate * amp * Math.cos(angle));
         position.set(amp * Math.sin(angle));
         filter.update();

         assertEquals(0.0, filter.getBiasEstimation().getValue(), epsilon);
         assertEquals(rate.getValue(), filter.getRateEstimation().getValue(), epsilon);
         assertEquals(position.getValue(), filter.getPositionEstimation().getValue(), Math.max(Math.abs(position.getValue()), 1.0) * epsilon);
      }
   }

   @Test
   public void test_RateConstantBias_Sinewave()
   {
      double epsilon = 1.0e-3;
      Random random = new Random(23524);

      double kp = 0.1;
      double ki = 0.01;
      double rateBias = RandomNumbers.nextDouble(random, 1.0);
      YoDouble position = new YoDouble("position", null);
      position.set(RandomNumbers.nextDouble(random, 100.0));
      YoDouble rate = new YoDouble("rate", null);
      YoDouble biasedRate = new YoDouble("biasedRate", null);
      double dt = 0.001;
      IntegratorBiasCompensatorYoVariable filter = new IntegratorBiasCompensatorYoVariable("blop", null, kp, ki, position, biasedRate, dt);

      double angle = 0.0;
      double angleRate = 0.0;
      double amp = 10.0;
      double freq = 1.0;

      // First loop is warmup to let filter estimate bias.
      for (int i = 0; i < 100000; i++)
      {
         angleRate = 2.0 * Math.PI * freq;
         angle += angleRate * dt;

         rate.set(angleRate * amp * Math.cos(angle));
         biasedRate.set(rate.getValue() + rateBias);
         position.set(amp * Math.sin(angle));
         filter.update();
      }

      for (int i = 0; i < 5000; i++)
      {
         angleRate = 2.0 * Math.PI * freq;
         angle += angleRate * dt;

         rate.set(angleRate * amp * Math.cos(angle));
         biasedRate.set(rate.getValue() + rateBias);
         position.set(amp * Math.sin(angle));
         filter.update();

         assertEquals(-rateBias, filter.getBiasEstimation().getValue(), epsilon);
         assertEquals(rate.getValue(), filter.getRateEstimation().getValue(), epsilon);
         assertEquals(position.getValue(), filter.getPositionEstimation().getValue(), Math.max(Math.abs(position.getValue()), 1.0) * epsilon);
      }
   }

   @Test
   public void test_PositionGaussianNoise_Sinewave()
   {
      Random random = new Random(23540);

      double kp = 0.1;
      double ki = 0.01;
      YoDouble position = new YoDouble("position", null);
      position.set(RandomNumbers.nextDouble(random, 100.0));
      YoDouble noisyPosition = new YoDouble("noisyPosition", null);
      YoDouble rate = new YoDouble("rate", null);
      double dt = 0.001;
      IntegratorBiasCompensatorYoVariable filter = new IntegratorBiasCompensatorYoVariable("blop", null, kp, ki, noisyPosition, rate, dt);

      double noiseAmplitude = RandomNumbers.nextDouble(random, 0.1);

      double angle = 0.0;
      double angleRate = 0.0;
      double amp = 10.0;
      double freq = 1.0;

      double epsilon = 10.0 * Math.abs(kp * noiseAmplitude);
      double biasEpsilon = 2.0 * Math.abs(kp * noiseAmplitude);

      for (int i = 0; i < 5000; i++)
      {
         angleRate = 2.0 * Math.PI * freq;
         angle += angleRate * dt;

         rate.set(angleRate * amp * Math.cos(angle));
         position.set(amp * Math.sin(angle));
         double noise = noiseAmplitude * random.nextGaussian();
         noisyPosition.set(position.getValue() + noise);
         filter.update();

         assertEquals(0.0, filter.getBiasEstimation().getValue(), biasEpsilon);
         assertEquals(rate.getValue(), filter.getRateEstimation().getValue(), biasEpsilon);
         assertEquals(position.getValue(), filter.getPositionEstimation().getValue(), epsilon);
      }
   }
}
