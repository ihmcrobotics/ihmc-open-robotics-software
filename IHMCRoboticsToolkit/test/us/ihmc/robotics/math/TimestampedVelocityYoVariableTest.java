package us.ihmc.robotics.math;


import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class TimestampedVelocityYoVariableTest
{
   private DoubleYoVariable position;
   private DoubleYoVariable timestamp;
   private TimestampedVelocityYoVariable velocityYoVariable;
   
   @Before
   public void setUp() throws Exception
   {
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      position = new DoubleYoVariable("testPosition", registry);
      timestamp = new DoubleYoVariable("testTimestamp", registry);
      velocityYoVariable = new TimestampedVelocityYoVariable("testVelVar", "", position, timestamp, registry, 1e-9);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasNotBeenUpdated()
   {
      double val = velocityYoVariable.getDoubleValue();
      assertEquals(0.0, val, 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasBeenUpdatedOnce()
   {
      position.set(1.0);
      timestamp.set(2.0);
      velocityYoVariable.update();
      double val = velocityYoVariable.getDoubleValue();
      assertEquals(0.0, val, 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasBeenUpdatedTwice()
   {
      velocityYoVariable.update();
      position.set(1.0);
      timestamp.set(2.0);
      velocityYoVariable.update();
      double val = velocityYoVariable.getDoubleValue();
      assertEquals(position.getDoubleValue() / timestamp.getDoubleValue(), val, 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasBeenUpdatedThreeTimes()
   {
      velocityYoVariable.update();
      position.set(1.0);
      timestamp.set(2.0);
      velocityYoVariable.update();
      position.set(2.0);
      timestamp.set(3.0);
      velocityYoVariable.update();
      double val = velocityYoVariable.getDoubleValue();
      assertEquals(1.0, val, 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testHasBeenUpdatedThreeTimesNoChange()
   {
      velocityYoVariable.update();
      position.set(1.0);
      timestamp.set(2.0);
      velocityYoVariable.update();
      timestamp.set(4.0);
      velocityYoVariable.update();
      double val = velocityYoVariable.getDoubleValue();
      assertEquals(0.5, val, 0.0);
   }
}
