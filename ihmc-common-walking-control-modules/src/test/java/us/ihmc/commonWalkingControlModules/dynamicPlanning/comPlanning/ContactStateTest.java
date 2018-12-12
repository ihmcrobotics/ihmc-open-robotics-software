package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;

import static junit.framework.Assert.assertFalse;
import static junit.framework.TestCase.assertTrue;

public class ContactStateTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsLoadBearing()
   {
      assertTrue(ContactState.IN_CONTACT.isLoadBearing());
      assertFalse(ContactState.FLIGHT.isLoadBearing());
   }
}
