package us.ihmc.robotics.time;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoStopwatchTest
{
   @Test
   public void testStopwatch()
   {
      YoRegistry registry = new YoRegistry("test");
      YoDouble yoTime = new YoDouble("t", registry);
      
      testStopwatch(yoTime, new YoStopwatch(yoTime));
      testStopwatch(yoTime, new YoStopwatch("timerTest", yoTime, registry));
   }

   private void testStopwatch(YoDouble yoTime, YoStopwatch stopwatch)
   {
      yoTime.set(0.0);
      
      double averageLap = stopwatch.averageLap();
      assertEquals("averageLap incorrect", Double.NaN, averageLap, 1e-2);
      
      stopwatch.start();
      
      double lapElapsed = stopwatch.lapElapsed();
      double totalElapsed = stopwatch.totalElapsed();
      averageLap = stopwatch.averageLap();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, 1e-2);
      assertEquals("totalElapsed incorrect", 0.0, totalElapsed, 1e-2);
      assertEquals("averageLap incorrect", Double.NaN, averageLap, 1e-2);
      
      double sleepTime1 = 0.5;
      yoTime.add(sleepTime1);
      
      double lap = stopwatch.lap();
      averageLap = stopwatch.averageLap();
      assertEquals("lap incorrect", sleepTime1, lap, 1e-2);
      assertEquals("averageLap incorrect", sleepTime1, averageLap, 1e-2);
      
      double sleepTime2 = 1.0;
      yoTime.add(sleepTime2);
      
      lap = stopwatch.lap();
      averageLap = stopwatch.averageLap();
      assertEquals("lap incorrect", sleepTime2, lap, 1e-2);
      assertEquals("averageLap incorrect", (sleepTime1 + sleepTime2) / 2.0, averageLap, 1e-2);
      
      stopwatch.resetLap();
      lapElapsed = stopwatch.lapElapsed();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, 1e-2);
      
      lap = stopwatch.lap();
      averageLap = stopwatch.averageLap();
      assertEquals("lap incorrect", 0.0, lap, 1e-2);
      assertEquals("averageLap incorrect", (sleepTime1 + sleepTime2) / 3.0, averageLap, 1e-2);
      
      double sleepTime3 = 0.3;
      yoTime.add(sleepTime3);
      
      stopwatch.suspend();
      yoTime.add(1000.0);
      stopwatch.resume();
      
      lapElapsed = stopwatch.lapElapsed();
      totalElapsed = stopwatch.totalElapsed();
      assertEquals("lapElapsed incorrect", sleepTime3, lapElapsed, 1e-2);
      assertEquals("totalElapsed incorrect", sleepTime1 + sleepTime2 + sleepTime3, totalElapsed, 1e-2);
      
      stopwatch.reset();
      lapElapsed = stopwatch.lapElapsed();
      totalElapsed = stopwatch.totalElapsed();
      averageLap = stopwatch.averageLap();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, 1e-2);
      assertEquals("totalElapsed incorrect", 0.0, totalElapsed, 1e-2);
      assertEquals("averageLap incorrect", Double.NaN, averageLap, 1e-2);
      
      double sleepTime4 = 0.3;
      yoTime.add(sleepTime4);
      
      stopwatch.resetLap();
      lapElapsed = stopwatch.lapElapsed();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, 1e-2);
   }
   
   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(YoStopwatch.class, YoStopwatchTest.class);
   }
}
