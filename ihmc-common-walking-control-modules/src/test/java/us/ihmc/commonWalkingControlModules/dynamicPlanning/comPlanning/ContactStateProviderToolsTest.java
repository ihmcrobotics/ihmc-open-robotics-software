package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ContactStateProviderToolsTest
{
   @Test
   public void testCheckContactSequenceDoesNotEndInFlight()
   {
      SettableContactStateProvider contactState1 = new SettableContactStateProvider();
      SettableContactStateProvider contactState2 = new SettableContactStateProvider();
      SettableContactStateProvider contactState3 = new SettableContactStateProvider();

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      contactSequence.add(contactState1);
      contactSequence.add(contactState2);
      contactSequence.add(contactState3);

      contactState1.setContactState(ContactState.FLIGHT);
      contactState2.setContactState(ContactState.FLIGHT);
      contactState3.setContactState(ContactState.IN_CONTACT);

      assertTrue(ContactStateProviderTools.checkContactSequenceDoesNotEndInFlight(contactSequence));

      contactState3.setContactState(ContactState.FLIGHT);
      assertFalse(ContactStateProviderTools.checkContactSequenceDoesNotEndInFlight(contactSequence));

      contactState3.setContactState(ContactState.IN_CONTACT);
      assertTrue(ContactStateProviderTools.checkContactSequenceDoesNotEndInFlight(contactSequence));
   }

   @Test
   public void testCheckContactSequenceIsContinuous()
   {
      SettableContactStateProvider contactState1 = new SettableContactStateProvider();
      SettableContactStateProvider contactState2 = new SettableContactStateProvider();
      SettableContactStateProvider contactState3 = new SettableContactStateProvider();
      SettableContactStateProvider contactState4 = new SettableContactStateProvider();

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      contactSequence.add(contactState1);
      contactSequence.add(contactState2);
      contactSequence.add(contactState3);
      contactSequence.add(contactState4);

      Random random = new Random(1738L);

      double time1 = RandomNumbers.nextDouble(random, -100.0);
      double time2 = RandomNumbers.nextDouble(random, time1, 100.0);
      double time3 = RandomNumbers.nextDouble(random, time2, 100.0);
      double time4 = RandomNumbers.nextDouble(random, time3, 100.0);
      double time5 = RandomNumbers.nextDouble(random, time4, 100.0);

      contactState1.getTimeInterval().setInterval(time1, time2);
      contactState2.getTimeInterval().setInterval(time2, time3);
      contactState3.getTimeInterval().setInterval(time3, time4);
      contactState4.getTimeInterval().setInterval(time4, time5);

      assertTrue(ContactStateProviderTools.checkContactSequenceIsContinuous(contactSequence));

      contactState1.getTimeInterval().setInterval(time1, time2 - 0.9 * ContactStateProviderTools.epsilonForContinuity);
      assertTrue(ContactStateProviderTools.checkContactSequenceIsContinuous(contactSequence));

      contactState2.getTimeInterval().setInterval(time2 + 0.9 * ContactStateProviderTools.epsilonForContinuity, time3);

      assertFalse(ContactStateProviderTools.checkContactSequenceIsContinuous(contactSequence));

      contactState2.getTimeInterval().setInterval(time2 - 0.9 * ContactStateProviderTools.epsilonForContinuity, time3);

      assertTrue(ContactStateProviderTools.checkContactSequenceIsContinuous(contactSequence));
   }
}
