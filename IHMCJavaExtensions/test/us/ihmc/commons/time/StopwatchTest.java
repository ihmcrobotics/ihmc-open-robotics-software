package us.ihmc.commons.time;

import static org.junit.Assert.assertEquals;

import java.util.Random;
import java.util.function.DoubleSupplier;

import org.junit.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

@SuppressWarnings(value = "unused")
public class StopwatchTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testConstructor()
   {
      Stopwatch stopwatch = new Stopwatch();

      assertEquals("didnt NaN", Double.NaN, stopwatch.averageLap(), 0.0);
      assertEquals("didnt NaN", Double.NaN, stopwatch.lapElapsed(), 0.0);
      assertEquals("didnt NaN", Double.NaN, stopwatch.totalElapsed(), 0.0);
      assertEquals("didnt NaN", Double.NaN, stopwatch.lap(), 0.0);
      
      stopwatch = new Stopwatch(new FakeTimeProvider());

      assertEquals("didnt NaN", Double.NaN, stopwatch.averageLap(), 0.0);
      assertEquals("didnt NaN", Double.NaN, stopwatch.lapElapsed(), 0.0);
      assertEquals("didnt NaN", Double.NaN, stopwatch.totalElapsed(), 0.0);
      assertEquals("didnt NaN", Double.NaN, stopwatch.lap(), 0.0);
   }

   public class FakeTimeProvider implements DoubleSupplier
   {
      public double clock = 0.0;

      public void incrementClock(double amount)
      {
         clock += amount;
      }

      @Override
      public double getAsDouble()
      {
         return clock;
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.8)
   @Test(timeout = 30000)
   public void testStopwatchWithRealTime()
   {
      Stopwatch stopwatch = new Stopwatch();
      
      double averageLap = stopwatch.averageLap();
      PrintTools.debug(this, "Lap: " + stopwatch.lap());
      assertEquals("averageLap incorrect", Double.NaN, averageLap, Epsilons.ONE_HUNDREDTH);
      
      assertEquals("return ref not equal", stopwatch, stopwatch.start());
      
      double lapElapsed = stopwatch.lapElapsed();
      double totalElapsed = stopwatch.totalElapsed();
      averageLap = stopwatch.averageLap();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, Epsilons.ONE_HUNDREDTH);
      assertEquals("totalElapsed incorrect", 0.0, totalElapsed, Epsilons.ONE_HUNDREDTH);
      assertEquals("averageLap incorrect", Double.NaN, averageLap, Epsilons.ONE_HUNDREDTH);
      
      double sleepTime1 = 0.5;
      ThreadTools.sleepSeconds(sleepTime1);
      
      double lap = stopwatch.lap();
      averageLap = stopwatch.averageLap();
      assertEquals("lap incorrect", sleepTime1, lap, Epsilons.ONE_HUNDREDTH);
      assertEquals("averageLap incorrect", sleepTime1, averageLap, Epsilons.ONE_HUNDREDTH);
      
      double sleepTime2 = 1.0;
      ThreadTools.sleepSeconds(sleepTime2);
      
      lap = stopwatch.lap();
      averageLap = stopwatch.averageLap();
      assertEquals("lap incorrect", sleepTime2, lap, Epsilons.ONE_HUNDREDTH);
      assertEquals("averageLap incorrect", (sleepTime1 + sleepTime2) / 2.0, averageLap, Epsilons.ONE_HUNDREDTH);
      
      stopwatch.resetLap();
      lapElapsed = stopwatch.lapElapsed();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, Epsilons.ONE_HUNDREDTH);
      
      lap = stopwatch.lap();
      averageLap = stopwatch.averageLap();
      assertEquals("lap incorrect", 0.0, lap, Epsilons.ONE_HUNDREDTH);
      assertEquals("averageLap incorrect", (sleepTime1 + sleepTime2) / 3.0, averageLap, Epsilons.ONE_HUNDREDTH);
      
      double sleepTime3 = 0.3;
      ThreadTools.sleepSeconds(sleepTime3);
      
      lapElapsed = stopwatch.lapElapsed();
      totalElapsed = stopwatch.totalElapsed();
      assertEquals("lapElapsed incorrect", sleepTime3, lapElapsed, Epsilons.ONE_HUNDREDTH);
      assertEquals("totalElapsed incorrect", sleepTime1 + sleepTime2 + sleepTime3, totalElapsed, Epsilons.ONE_HUNDREDTH);
      
      stopwatch.reset();
      lapElapsed = stopwatch.lapElapsed();
      totalElapsed = stopwatch.totalElapsed();
      averageLap = stopwatch.averageLap();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, Epsilons.ONE_HUNDREDTH);
      assertEquals("totalElapsed incorrect", 0.0, totalElapsed, Epsilons.ONE_HUNDREDTH);
      assertEquals("averageLap incorrect", Double.NaN, averageLap, Epsilons.ONE_HUNDREDTH);
      
      double sleepTime4 = 0.3;
      ThreadTools.sleepSeconds(sleepTime4);
      
      stopwatch.resetLap();
      lapElapsed = stopwatch.lapElapsed();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, Epsilons.ONE_HUNDREDTH);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.8)
   @Test(timeout = 30000)
   public void testStopwatch()
   {
      FakeTimeProvider fakeTimeProvider = new FakeTimeProvider();
      Stopwatch stopwatch = new Stopwatch(fakeTimeProvider);
      
      double averageLap = stopwatch.averageLap();
      PrintTools.debug(this, "Lap: " + stopwatch.lap());
      assertEquals("averageLap incorrect", Double.NaN, averageLap, Epsilons.ONE_TEN_BILLIONTH);
      
      assertEquals("return ref not equal", stopwatch, stopwatch.start());
      
      double lapElapsed = stopwatch.lapElapsed();
      double totalElapsed = stopwatch.totalElapsed();
      averageLap = stopwatch.averageLap();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, Epsilons.ONE_TEN_BILLIONTH);
      assertEquals("totalElapsed incorrect", 0.0, totalElapsed, Epsilons.ONE_TEN_BILLIONTH);
      assertEquals("averageLap incorrect", Double.NaN, averageLap, Epsilons.ONE_TEN_BILLIONTH);
      
      double sleepTime1 = 0.5;
      fakeTimeProvider.incrementClock(sleepTime1);
      
      double lap = stopwatch.lap();
      averageLap = stopwatch.averageLap();
      assertEquals("lap incorrect", sleepTime1, lap, Epsilons.ONE_TEN_BILLIONTH);
      assertEquals("averageLap incorrect", sleepTime1, averageLap, Epsilons.ONE_TEN_BILLIONTH);
      
      double sleepTime2 = 1.0;
      fakeTimeProvider.incrementClock(sleepTime2);
      
      lap = stopwatch.lap();
      averageLap = stopwatch.averageLap();
      assertEquals("lap incorrect", sleepTime2, lap, Epsilons.ONE_TEN_BILLIONTH);
      assertEquals("averageLap incorrect", (sleepTime1 + sleepTime2) / 2.0, averageLap, Epsilons.ONE_TEN_BILLIONTH);
      
      stopwatch.resetLap();
      lapElapsed = stopwatch.lapElapsed();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, Epsilons.ONE_TEN_BILLIONTH);
      
      lap = stopwatch.lap();
      averageLap = stopwatch.averageLap();
      assertEquals("lap incorrect", 0.0, lap, Epsilons.ONE_TEN_BILLIONTH);
      assertEquals("averageLap incorrect", (sleepTime1 + sleepTime2) / 3.0, averageLap, Epsilons.ONE_TEN_BILLIONTH);
      
      double sleepTime3 = 0.3;
      fakeTimeProvider.incrementClock(sleepTime3);
      
      lapElapsed = stopwatch.lapElapsed();
      totalElapsed = stopwatch.totalElapsed();
      assertEquals("lapElapsed incorrect", sleepTime3, lapElapsed, Epsilons.ONE_TEN_BILLIONTH);
      assertEquals("totalElapsed incorrect", sleepTime1 + sleepTime2 + sleepTime3, totalElapsed, Epsilons.ONE_TEN_BILLIONTH);
      
      stopwatch.reset();
      lapElapsed = stopwatch.lapElapsed();
      totalElapsed = stopwatch.totalElapsed();
      averageLap = stopwatch.averageLap();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, Epsilons.ONE_TEN_BILLIONTH);
      assertEquals("totalElapsed incorrect", 0.0, totalElapsed, Epsilons.ONE_TEN_BILLIONTH);
      assertEquals("averageLap incorrect", Double.NaN, averageLap, Epsilons.ONE_TEN_BILLIONTH);
      
      double sleepTime4 = 0.3;
      fakeTimeProvider.incrementClock(sleepTime4);
      
      stopwatch.resetLap();
      lapElapsed = stopwatch.lapElapsed();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, Epsilons.ONE_TEN_BILLIONTH);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSuspendAndResume()
   {
      Random random = new Random(12389L);
      FakeTimeProvider fakeTimeProvider = new FakeTimeProvider();
      Stopwatch stopwatch = new Stopwatch(fakeTimeProvider);

      for (int i = 0; i < 10; i++)
      {
         stopwatch.start();
         double randomDuration1 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.suspend();
         double randomDuration2 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.resume();
         double randomDuration3 = sleep(fakeTimeProvider, randomTime(random));
         assertTimeEquals(randomDuration1 + randomDuration3, stopwatch.lapElapsed());
         assertTimeEquals(randomDuration1 + randomDuration3, stopwatch.totalElapsed());
         assertTimeEquals(randomDuration1 + randomDuration3, stopwatch.lap());
      }

      for (int i = 0; i < 10; i++)
      {
         stopwatch.start();
         double randomDuration1 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.resume();
         double randomDuration2 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.suspend();
         double randomDuration3 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.resume();
         double randomDuration4 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.suspend();
         double randomDuration5 = sleep(fakeTimeProvider, randomTime(random));
         double expectedElapsed = randomDuration1 + randomDuration2 + randomDuration4;
         assertTimeEquals(expectedElapsed, stopwatch.lapElapsed());
         assertTimeEquals(expectedElapsed, stopwatch.totalElapsed());
         assertTimeEquals(expectedElapsed, stopwatch.lap());
      }

      for (int i = 0; i < 10; i++)
      {
         stopwatch.reset();
         double randomDuration1 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.suspend();
         double randomDuration2 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.resume();
         double randomDuration3 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.lap();
         double randomDuration4 = sleep(fakeTimeProvider, randomTime(random));
         assertTimeEquals(randomDuration4, stopwatch.lapElapsed());
         assertTimeEquals(randomDuration1 + randomDuration3 + randomDuration4, stopwatch.totalElapsed());
         assertTimeEquals(randomDuration4, stopwatch.lap());
      }

      for (int i = 0; i < 10; i++)
      {
         stopwatch.reset();
         double randomDuration1 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.suspend();
         double randomDuration2 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.resume();
         double randomDuration3 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.lap();
         double randomDuration4 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.suspend();
         double randomDuration5 = sleep(fakeTimeProvider, randomTime(random));
         assertTimeEquals(randomDuration4, stopwatch.lapElapsed());
         assertTimeEquals(randomDuration1 + randomDuration3 + randomDuration4, stopwatch.totalElapsed());
         assertTimeEquals(randomDuration4, stopwatch.lap());
      }

      for (int i = 0; i < 10; i++)
      {
         stopwatch.reset();
         double randomDuration1 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.suspend();
         double randomDuration2 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.resume();
         double randomDuration3 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.lap();
         double randomDuration4 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.suspend();
         double randomDuration5 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.reset();
         double randomDuration6 = sleep(fakeTimeProvider, randomTime(random));
         assertTimeEquals(randomDuration6, stopwatch.lapElapsed());
         assertTimeEquals(randomDuration6, stopwatch.totalElapsed());
         assertTimeEquals(randomDuration6, stopwatch.lap());
         stopwatch.suspend();
         double randomDuration7 = sleep(fakeTimeProvider, randomTime(random));
         stopwatch.resetLap();
         double randomDuration8 = sleep(fakeTimeProvider, randomTime(random));
         assertTimeEquals(randomDuration8, stopwatch.lapElapsed());
         assertTimeEquals(randomDuration8 + randomDuration6, stopwatch.totalElapsed());
         assertTimeEquals(randomDuration8, stopwatch.lap());
      }

      stopwatch.reset();
      {
         double randomDuration1 = randomTime(random);
         double randomDuration2 = randomTime(random);
         double randomDuration3 = randomTime(random);
         for (int i = 0; i < 10; i++)
         {
            sleep(fakeTimeProvider, randomDuration1);
            stopwatch.suspend();
            sleep(fakeTimeProvider, randomDuration2);
            stopwatch.resume();
            sleep(fakeTimeProvider, randomDuration3);
            stopwatch.resetLap();
            sleep(fakeTimeProvider, randomDuration1);
            stopwatch.suspend();
            sleep(fakeTimeProvider, randomDuration1);
            stopwatch.suspend();
            sleep(fakeTimeProvider, randomDuration2);
            stopwatch.resume();
            stopwatch.resume();
            sleep(fakeTimeProvider, randomDuration3);
            assertTimeEquals(randomDuration1 + randomDuration3, stopwatch.lapElapsed());
            stopwatch.lap();
            assertTimeEquals((randomDuration1 + randomDuration3) * (i + 1), stopwatch.totalElapsed());
         }
      }
   }

   private void assertTimeEquals(double expected, double actual)
   {
      PrintTools.info("Expected: " + expected + " (s)  Actual: " + actual + " (s)");
      assertEquals("Expected time incorrect", expected, actual, Epsilons.ONE_TEN_BILLIONTH);
      PrintTools.info("Assertions passed!");
   }

   private double randomTime(Random random)
   {
      return Conversions.millisecondsToSeconds(Math.abs(random.nextInt()) % 100);
   }

   private double sleep(FakeTimeProvider fakeTimeProvider, double duration)
   {
      fakeTimeProvider.incrementClock(duration);
      return duration;
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(Stopwatch.class, StopwatchTest.class);
   }
}
