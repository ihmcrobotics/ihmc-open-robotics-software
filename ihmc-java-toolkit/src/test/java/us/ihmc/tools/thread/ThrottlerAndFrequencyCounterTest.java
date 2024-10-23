package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.commons.UnitConversions;
import us.ihmc.tools.time.FrequencyCalculator;

import static org.junit.jupiter.api.Assertions.*;

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
         MissingThreadTools.sleep(sleepTimeSeconds);
      }

      frequencyCalculator.destroy();

      assertEquals(targetFrequency, frequencyCalculator.getFrequency(), epsilon, "Frequency not correct");
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
         MissingThreadTools.sleep(sleepTimeSeconds);
      }

      frequencyCalculator.destroy();

      // Decaying sleep
      MissingThreadTools.sleep(decayTimeSeconds);

      double targetDecayFrequency = frequencyCalculator.getFrequency() / Math.exp(decayTimeSeconds);
      assertEquals(frequencyCalculator.getFrequencyDecaying(), targetDecayFrequency, epsilon);
   }

   private static void testThrottlerAndFrequencyCounter(double targetFrequency, double epsilon)
   {
      LogTools.info("testThrottlerAndFrequencyCounter (using us.ihmc.tools.thread.Throttler) targetFrequency=" + targetFrequency + " epsilon=" + epsilon);

      Throttler throttler = new Throttler();
      throttler.setFrequency(targetFrequency);
      throttler.waitAndRun();

      long start = System.currentTimeMillis();
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator(true);

      // Run for 5 seconds
      while (System.currentTimeMillis() - start < 5000)
      {
         frequencyCalculator.ping();

         throttler.waitAndRun();
      }

      frequencyCalculator.destroy();

      assertEquals(targetFrequency, frequencyCalculator.getFrequency(), epsilon, "Frequency not correct");
   }

   @Test
   public void testFrequencyCounter100Hz()
   {
      testFrequencyCounter(100, 2);
   }

   @Test
   public void testThrottlerAndFrequencyCounter100Hz()
   {
      testThrottlerAndFrequencyCounter(100, 2);
   }

   @Test
   public void testFrequencyCounter10Hz()
   {
      testFrequencyCounter(10, 0.2);
   }

   @Test
   public void testThrottlerAndFrequencyCounter10Hz()
   {
      testThrottlerAndFrequencyCounter(10, 0.2);
   }

   @Test
   public void testFrequencyCounter1Hz()
   {
      testFrequencyCounter(1, 0.002);
   }

   @Test
   public void testThrottlerAndFrequencyCounter1Hz()
   {
      testThrottlerAndFrequencyCounter(1, 0.002);
   }

   @Test
   public void testFrequencyCounter0_5Hz()
   {
      testFrequencyCounter(0.5, 0.001);
   }

   @Test
   public void testThrottlerAndFrequencyCounter0_5Hz()
   {
      testThrottlerAndFrequencyCounter(0.5, 0.001);
   }

   @Test
   public void testThrottlerAndFrequencyCounter100HzDecaying()
   {
      testFrequencyCounterDecaying(100, 5, 2);
   }
}
