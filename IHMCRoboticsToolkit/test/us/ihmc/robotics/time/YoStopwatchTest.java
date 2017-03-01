package us.ihmc.robotics.time;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoStopwatchTest
{
   @ContinuousIntegrationTest(estimatedDuration = 1.8)
   @Test(timeout = 30000)
   public void testStopwatch()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      DoubleYoVariable yoTime = new DoubleYoVariable("t", registry);
      
      testStopwatch(yoTime, new YoStopwatch(yoTime));
      testStopwatch(yoTime, new YoStopwatch("timerTest", yoTime, registry));
   }

   private void testStopwatch(DoubleYoVariable yoTime, YoStopwatch stopwatch)
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
