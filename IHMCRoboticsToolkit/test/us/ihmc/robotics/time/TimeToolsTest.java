package us.ihmc.robotics.time;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class TimeToolsTest
{

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testToSeconds()
   {
     long timestamp = 1500000000;
     
     assertEquals(1.5, TimeTools.nanoSecondstoSeconds(timestamp), 1e-22);
     
     assertEquals(-1.5, TimeTools.nanoSecondstoSeconds(-timestamp), 1e-22);
     
     
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testToNanoSeconds()
   {
      double time = 1.5;
      
      assertEquals(1500000000, TimeTools.secondsToNanoSeconds(time));
      assertEquals(-1500000000, TimeTools.secondsToNanoSeconds(-time));
   }

}
