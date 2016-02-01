package us.ihmc.atlas.behaviorTests;

import static us.ihmc.tools.testing.TestPlanTarget.Exclude;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = Exclude)
public class FooTest
{
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 5000)
   public void fooTest() throws InterruptedException
   {
      while(true)
      {
         Thread.sleep(1000L);
      }
   }
}
