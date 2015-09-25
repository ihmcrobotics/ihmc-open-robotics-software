package us.ihmc.atlas.behaviorTests;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.MemoryTools;

public class FooTest
{
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
@Test
public void fooTest() throws InterruptedException
{
   while(true)
   {
      Thread.sleep(1000L);
   }
}
}
