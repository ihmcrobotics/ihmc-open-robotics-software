package us.ihmc.tools.inputDevices.keyboard.linux;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;

public class RepeatingReleasedEventsFixerTest
{
   @Test
   public void testInstallAndRemove()
   {
      RepeatingReleasedEventsFixer repeatingReleasedEventsFixer = new RepeatingReleasedEventsFixer();
      repeatingReleasedEventsFixer.install();
      
      // TODO test some dispatched events here
      
      repeatingReleasedEventsFixer.remove();
   }
}
