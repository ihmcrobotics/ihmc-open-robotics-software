package us.ihmc.tools.time;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;

public class MovingAverageDurationCalculatorTest
{
   @Test
   public void test()
   {
      int windowSize = 3;
      MovingAverageDurationCalculator durationCalculator = new MovingAverageDurationCalculator(3);

      for (int i = 0; i < 10; i++)
      {
         ThreadTools.sleep(10);

         durationCalculator.ping();
         Assertions.assertEquals(0.01, durationCalculator.getDuration(), 0.01);
      }
   }

   @Test
   public void testPause()
   {
      int windowSize = 3;
      MovingAverageDurationCalculator durationCalculator = new MovingAverageDurationCalculator(3);

      for (int i = 0; i < 10; i++)
      {
         ThreadTools.sleep(10);

         durationCalculator.pause();

         ThreadTools.sleep(20);

         durationCalculator.ping();
         Assertions.assertEquals(0.01, durationCalculator.getDuration(), 0.01);
      }
   }

   @Test
   public void testReset()
   {
      int windowSize = 3;
      MovingAverageDurationCalculator durationCalculator = new MovingAverageDurationCalculator(3);

      ThreadTools.sleep(200);

      durationCalculator.reset();

      for (int i = 0; i < 10; i++)
      {
         ThreadTools.sleep(10);

         durationCalculator.ping();
         Assertions.assertEquals(0.01, durationCalculator.getDuration(), 0.01);
      }
   }
}
