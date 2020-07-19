package us.ihmc.robotics.math;


import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TimestampedVelocityYoVariableTest
{
   private YoDouble position;
   private YoDouble timestamp;
   private TimestampedVelocityYoVariable velocityYoVariable;
   
   @BeforeEach
   public void setUp() throws Exception
   {
      YoRegistry registry = new YoRegistry("testRegistry");
      position = new YoDouble("testPosition", registry);
      timestamp = new YoDouble("testTimestamp", registry);
      velocityYoVariable = new TimestampedVelocityYoVariable("testVelVar", "", position, timestamp, registry, 1e-9);
   }

	@Test
   public void testHasNotBeenUpdated()
   {
      double val = velocityYoVariable.getDoubleValue();
      assertEquals(0.0, val, 0.0);
   }

	@Test
   public void testHasBeenUpdatedOnce()
   {
      position.set(1.0);
      timestamp.set(2.0);
      velocityYoVariable.update();
      double val = velocityYoVariable.getDoubleValue();
      assertEquals(0.0, val, 0.0);
   }

	@Test
   public void testHasBeenUpdatedTwice()
   {
      velocityYoVariable.update();
      position.set(1.0);
      timestamp.set(2.0);
      velocityYoVariable.update();
      double val = velocityYoVariable.getDoubleValue();
      assertEquals(position.getDoubleValue() / timestamp.getDoubleValue(), val, 0.0);
   }

	@Test
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

	@Test
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
