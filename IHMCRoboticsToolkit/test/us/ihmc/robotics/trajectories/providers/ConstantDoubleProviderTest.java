package us.ihmc.robotics.trajectories.providers;

import org.junit.Test;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.random.RandomTools;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class ConstantDoubleProviderTest
{

   private static final double EPS = 1e-12;

	@EstimatedDuration(duration = 0.0)
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
