package us.ihmc.tools.inputDevices.keyboard.linux;

import org.junit.Test;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.inputDevices.keyboard.linux.RepeatingReleasedEventsFixer;

@ContinuousIntegrationPlan(categories ={IntegrationCategory.IN_DEVELOPMENT})
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
