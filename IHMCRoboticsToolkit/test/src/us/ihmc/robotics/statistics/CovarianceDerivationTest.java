package us.ihmc.robotics.statistics;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.apache.commons.math3.stat.descriptive.SummaryStatistics;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class CovarianceDerivationTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testLawOfLargeNumbers()
   {
      Random random = new Random(12352351L);
      int sampleSize = 100000;

      double variance = 2.5;
      double stdDev = Math.sqrt(variance);
      SummaryStatistics statistics = new SummaryStatistics();
      for (int i = 0; i < sampleSize; i++)
      {
         statistics.addValue(random.nextGaussian() * stdDev);
      }

      double epsilon = variance / 1e-2;
      assertEquals(variance, statistics.getVariance(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 8.4)
	@Test(timeout = 42000)
   public void testRandomWalkDiscretization()
   {
      Random random = new Random(1252L);
      double qContinuous = 2.5;

      double dtSmall = 1e-3;
      double qSmall = qContinuous * dtSmall;
      double stdDevSmall = Math.sqrt(qSmall);
      SummaryStatistics smallStatistics = new SummaryStatistics();

      double dtBig = 1.0;
      double qBig = qContinuous * dtBig;
      double stdDevBig = Math.sqrt(qBig);
      SummaryStatistics bigStatistics = new SummaryStatistics();

      int sampleSize = 100000;

      for (int i = 0; i < sampleSize; i++)
      {
         bigStatistics.addValue(random.nextGaussian() * stdDevBig);

         double x = 0.0;
         for (double t = 0.0; t < dtBig; t += dtSmall)
         {
            x += random.nextGaussian() * stdDevSmall;
         }
         smallStatistics.addValue(x);
      }

      double epsilon = qBig / 1e2;
      assertEquals(bigStatistics.getVariance(), qBig, epsilon);
      assertEquals(bigStatistics.getVariance(), smallStatistics.getVariance(), epsilon);
   }
}
