package us.ihmc.robotics.math.trajectories.providers;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class YoVariableDoubleProviderTest
{
   private YoVariableRegistry registry;
   private DoubleYoVariable yoValue;
   private static double value1 = Math.random();
   private static double value2 = Math.random();
   private static final double EPSILON = 1e-14;
   
   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("registry");
      yoValue = new DoubleYoVariable("value", "a DoubleYoVariable in paradise", registry);
      yoValue.set(value1);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor1()
   {
      YoVariableDoubleProvider provider1 = new YoVariableDoubleProvider("provider1", registry);
      assertEquals(0.0, provider1.getValue(), EPSILON);
      
      provider1.set(value2);
      assertEquals(value2, provider1.getValue(), EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor2()
   {
      YoVariableDoubleProvider provider2 = new YoVariableDoubleProvider(yoValue);
      assertEquals(value1, provider2.getValue(), EPSILON);

      provider2.set(value2);
      assertEquals(value2, provider2.getValue(), EPSILON);
   }
}
