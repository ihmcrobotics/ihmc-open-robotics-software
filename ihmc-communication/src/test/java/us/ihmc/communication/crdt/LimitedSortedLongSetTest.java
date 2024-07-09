package us.ihmc.communication.crdt;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class LimitedSortedLongSetTest
{
   @Test
   public void test()
   {
      LimitedSortedLongSet set = new LimitedSortedLongSet(10);

      for (int i = 0; i < 20; i++)
      {
         set.addWithLimit(i);
      }

      for (long i = 10; i < 20; i++)
      {
         Assertions.assertTrue(set.contains(i));
      }
   }
}
