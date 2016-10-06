package us.ihmc.tools.inputDevices.keyboard.linux;

import org.junit.Test;

import us.ihmc.tools.inputDevices.keyboard.linux.RepeatingReleasedEventsFixer;
import us.ihmc.tools.testing.TestPlanAnnotations;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.TestPlanTarget;

@ContinuousIntegrationPlan(targets ={TestPlanTarget.InDevelopment})
public class RepeatingReleasedEventsFixerTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testInstallAndRemove()
   {
      RepeatingReleasedEventsFixer repeatingReleasedEventsFixer = new RepeatingReleasedEventsFixer();
      repeatingReleasedEventsFixer.install();
      
      // TODO test some dispatched events here
      
      repeatingReleasedEventsFixer.remove();
   }
}
