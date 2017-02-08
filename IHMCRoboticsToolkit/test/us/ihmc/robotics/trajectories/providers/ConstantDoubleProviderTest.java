package us.ihmc.robotics.trajectories.providers;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomTools;

public class ConstantDoubleProviderTest
{

   private static final double EPS = 1e-12;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random();
      double expectedValue = RandomTools.generateRandomDouble(random, Double.MIN_VALUE, Double.MAX_VALUE);
      ConstantDoubleProvider constantDoubleProvider = new ConstantDoubleProvider(expectedValue);
      double actualValue = constantDoubleProvider.getValue();
      
      assertEquals(expectedValue, actualValue, EPS);
   }

}
