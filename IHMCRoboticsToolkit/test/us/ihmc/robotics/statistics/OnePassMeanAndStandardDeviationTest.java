package us.ihmc.robotics.statistics;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.robotics.statistics.OnePassMeanAndStandardDeviation.InsufficientMeasurementsException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import static org.junit.Assert.assertEquals;

public class OnePassMeanAndStandardDeviationTest
{
   private OnePassMeanAndStandardDeviation meanAndStandardDeviation;

   @Before
   public void setUp() throws Exception
   {
      meanAndStandardDeviation = new OnePassMeanAndStandardDeviation();
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testCalculateMeanAndStandardDeviationExample()
   {
      double[] data =
      {
         65.0, 66.0, 67.0, 69.0, 70.0, 70.0, 70.0, 71.0, 71.0, 72.0, 73.0, 74.0, 76.0
      };

      for (double measurement : data)
      {
         meanAndStandardDeviation.compute(measurement);
      }

      double delta = 1e-10;
      assertEquals(70.3076923076923, meanAndStandardDeviation.getAverage(), delta);
      assertEquals(8.982248520710021, meanAndStandardDeviation.getVariance(), delta);
      assertEquals(2.997039959812018, meanAndStandardDeviation.getStandardDeviation(), delta);
      assertEquals(9.730769230769189, meanAndStandardDeviation.getSampleVariance(), delta);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testCalculateMeanAndStandardDeviationSingleValue()
   {
      meanAndStandardDeviation.compute(65.0);

      double delta = 1e-10;
      assertEquals(65.0, meanAndStandardDeviation.getAverage(), delta);
      assertEquals(0.0, meanAndStandardDeviation.getVariance(), delta);
      assertEquals(0.0, meanAndStandardDeviation.getStandardDeviation(), delta);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = InsufficientMeasurementsException.class)
   public void testCalculateSampleVarianceSingleValue()
   {
      meanAndStandardDeviation.compute(65.0);
      double delta = 1e-10;
      assertEquals(0.0, meanAndStandardDeviation.getSampleVariance(), delta);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = InsufficientMeasurementsException.class)
   public void testCalculateAverageNoValue()
   {
      double delta = 1e-10;
      assertEquals(0.0, meanAndStandardDeviation.getAverage(), delta);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = InsufficientMeasurementsException.class)
   public void testCalculateVarianceNoValue()
   {
      double delta = 1e-10;
      assertEquals(0.0, meanAndStandardDeviation.getVariance(), delta);
   }

}
