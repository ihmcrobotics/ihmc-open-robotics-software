package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.TestTools;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.time.FrequencyCalculator;

public class ThrottlerAndFrequencyCounterTest
{
   private static void testFrequencyCounter(double targetFrequency, double epsilon)
   {
      LogTools.info("testFrequencyCounter targetFrequency=" + targetFrequency + " epsilon=" + epsilon);

      long start = System.currentTimeMillis();
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator(true);

      // Run for 5 seconds
      while (System.currentTimeMillis() - start < 5000)
      {
         frequencyCalculator.ping();

         double sleepTimeSeconds = UnitConversions.hertzToSeconds(targetFrequency);
         long sleepTimeMillis = (long) (sleepTimeSeconds * 1000);

         ThreadTools.sleep(sleepTimeMillis);
      }

      frequencyCalculator.destroy();

      TestTools.assertEpsilonEquals(targetFrequency, frequencyCalculator.getFrequency(), epsilon, "Frequency not correct");
   }

   private static void testFrequencyCounterDecaying(double targetFrequency, double decayTimeSeconds, double epsilon)
   {
      LogTools.info("testFrequencyCounter (decaying) targetFrequency=" + targetFrequency + " decayTimeSeconds=" + decayTimeSeconds + " epsilon=" + epsilon);

      long start = System.currentTimeMillis();
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator(true);

      // Run for 5 seconds
      while (System.currentTimeMillis() - start < 5000)
      {
         frequencyCalculator.ping();

         double sleepTimeSeconds = UnitConversions.hertzToSeconds(targetFrequency);
         long sleepTimeMillis = (long) (sleepTimeSeconds * 1000);

         ThreadTools.sleep(sleepTimeMillis);
      }

      frequencyCalculator.destroy();

      // Decaying sleep
      ThreadTools.sleep((long) (decayTimeSeconds * 1000));

      double targetDecayFrequency = frequencyCalculator.getFrequency() / Math.sqrt(decayTimeSeconds);
      TestTools.assertEpsilonEquals(frequencyCalculator.getFrequencyDecaying(), targetDecayFrequency, epsilon);
   }

   private static void testThrottlerAndFrequencyCounter(double targetFrequency, double epsilon)
   {
      LogTools.info("testThrottlerAndFrequencyCounter (using us.ihmc.tools.thread.Throttler) targetFrequency=" + targetFrequency + " epsilon=" + epsilon);

      Throttler throttler = new Throttler();
      throttler.setFrequency(targetFrequency);

      long start = System.currentTimeMillis();
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator(true);

      // Run for 5 seconds
      while (System.currentTimeMillis() - start < 5000)
      {
         frequencyCalculator.ping();

         throttler.waitAndRun();
      }

      frequencyCalculator.destroy();

      TestTools.assertEpsilonEquals(targetFrequency, frequencyCalculator.getFrequency(), epsilon, "Frequency not correct");
   }

   @Test
   public void testFrequencyCounter100Hz()
   {
      testFrequencyCounter(100, 1);
   }

   @Test
   public void testThrottlerAndFrequencyCounter100Hz()
   {
      testThrottlerAndFrequencyCounter(100, 1);
   }

   @Test
   public void testFrequencyCounter10Hz()
   {
      testFrequencyCounter(10, 0.1);
   }

   @Test
   public void testThrottlerAndFrequencyCounter10Hz()
   {
      testThrottlerAndFrequencyCounter(10, 0.1);
   }

   @Test
   public void testFrequencyCounter1Hz()
   {
      testFrequencyCounter(1, 0.01);
   }

   @Test
   public void testThrottlerAndFrequencyCounter1Hz()
   {
      testThrottlerAndFrequencyCounter(1, 0.01);
   }

   @Test
   public void testFrequencyCounter0_5Hz()
   {
      testFrequencyCounter(0.5, 0.005);
   }

   @Test
   public void testThrottlerAndFrequencyCounter0_5Hz()
   {
      testThrottlerAndFrequencyCounter(0.5, 0.005);
   }

   @Test
   public void testThrottlerAndFrequencyCounter100HzDecaying()
   {
      testFrequencyCounterDecaying(100, 5, 10);
   }
}
