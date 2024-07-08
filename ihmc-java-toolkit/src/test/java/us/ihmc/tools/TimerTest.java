package us.ihmc.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;

import static org.junit.jupiter.api.Assertions.*;

public class TimerTest
{
   @Test
   public void testInitialState()
   {
      Timer timer = new Timer();

      double now = Conversions.nanosecondsToSeconds(System.nanoTime());

      assertTrue(Double.isNaN(timer.getElapsedTime()));
      assertFalse(timer.isRunning(now));
      assertFalse(timer.isExpired(now));
      assertFalse(timer.hasBeenSet());

      TimerSnapshot snapshot = timer.createSnapshot();
      assertTrue(Double.isNaN(snapshot.getTimePassedSinceReset()));
      assertFalse(snapshot.isRunning(now));
      assertFalse(snapshot.isExpired(now));
      assertFalse(snapshot.hasBeenSet());

      TimerSnapshotWithExpiration snapshotWithExpiration = timer.createSnapshot(now + 1.0);
      assertTrue(Double.isNaN(snapshotWithExpiration.getTimePassedSinceReset()));
      assertFalse(snapshotWithExpiration.isRunning());
      assertFalse(snapshotWithExpiration.isExpired());
      assertFalse(snapshotWithExpiration.hasBeenSet());
   }

   @Test
   public void testTimer()
   {
      testTimer(0.001);
      testTimer(0.2);
      testTimer(0.000042487234);
      testTimer(1.0);
      testTimer(2.342);
   }

   private static void testTimer(double expirationTime)
   {
      Timer timer = new Timer();

      assertFalse(timer.isRunning(expirationTime));
      assertFalse(timer.isExpired(expirationTime));

      timer.reset();

      assertTrue(timer.isRunning(expirationTime));
      assertFalse(timer.isExpired(expirationTime));

      MissingThreadTools.sleepAtLeast(expirationTime);

      assertFalse(timer.isRunning(expirationTime));
      assertTrue(timer.isExpired(expirationTime));
   }

   @Test
   public void testTimerSleep()
   {
      testTimerSleep(0.00000231);
      testTimerSleep(0.0002);
      testTimerSleep(0.12);
      testTimerSleep(0.3);
      testTimerSleep(1.0);
      testTimerSleep(1.78);
   }

   private static void testTimerSleep(double expirationTime)
   {
      Timer timer = new Timer();

      assertFalse(timer.isRunning(expirationTime));
      assertFalse(timer.isExpired(expirationTime));

      timer.reset();

      assertTrue(timer.isRunning(expirationTime));
      assertFalse(timer.isExpired(expirationTime));

      double before = Conversions.nanosecondsToSeconds(System.nanoTime());

      timer.sleepUntilExpiration(expirationTime);

      double after = Conversions.nanosecondsToSeconds(System.nanoTime());

      boolean isRunning = timer.isRunning(expirationTime);
      boolean isExpired = timer.isExpired(expirationTime);

      LogTools.info("Slept for {} s", after - before);

      assertFalse(isRunning);
      assertTrue(isExpired);
   }
}
