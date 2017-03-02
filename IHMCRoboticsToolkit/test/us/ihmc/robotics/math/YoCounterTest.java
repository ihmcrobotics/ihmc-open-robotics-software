package us.ihmc.robotics.math;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class YoCounterTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testCounter()
   {
      YoCounter counter = new YoCounter("numberOfThings", new YoVariableRegistry("countReg"));
      counter.setMaxCount(5);
      
      assertFalse(counter.maxCountReached());
      counter.countOne();
      assertFalse(counter.maxCountReached());
      counter.countOne();
      assertFalse(counter.maxCountReached());
      counter.countOne();
      assertFalse(counter.maxCountReached());
      counter.countOne();
      assertFalse(counter.maxCountReached());
      counter.countOne();
      assertTrue(counter.maxCountReached());
      counter.countOne();
      assertTrue(counter.maxCountReached());
      
      counter.resetCount();
      assertFalse(counter.maxCountReached());
      counter.countN(4);
      assertFalse(counter.maxCountReached());
      counter.countOne();
      assertTrue(counter.maxCountReached());
      
      counter.resetCount();
      counter.setMaxCount(3);
      assertFalse(counter.maxCountReached());
      counter.countN(3);
      assertTrue(counter.maxCountReached());
   }
}
