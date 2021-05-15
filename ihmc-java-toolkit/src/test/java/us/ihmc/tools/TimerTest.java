package us.ihmc.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;

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
}
