package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ContactStateTest
{
   @Test
   public void testIsLoadBearing()
   {
      assertTrue(ContactState.IN_CONTACT.isLoadBearing());
      assertFalse(ContactState.FLIGHT.isLoadBearing());
   }
}
