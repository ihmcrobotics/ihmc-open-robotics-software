package us.ihmc.atlas.behaviorTests;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations;

public class FooTest
{
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @TestPlanAnnotations.DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 5000)
   public void fooTest() throws InterruptedException
   {
      while(true)
      {
         Thread.sleep(1000L);
      }
   }
}
