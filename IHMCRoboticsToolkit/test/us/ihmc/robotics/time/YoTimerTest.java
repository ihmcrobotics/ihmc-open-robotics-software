package us.ihmc.robotics.time;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.MutationTestingTools;

public class YoTimerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 1.8)
   @Test(timeout = 30000)
   public void testTimer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      DoubleYoVariable yoTime = new DoubleYoVariable("t", registry);
      
      testTimer(yoTime, new YoTimer(yoTime));
      testTimer(yoTime, new YoTimer("timerTest", yoTime, registry));
   }

   private void testTimer(DoubleYoVariable yoTime, YoTimer timer)
   {
      yoTime.set(0.0);
      
      double averageLap = timer.averageLap();
      assertEquals("averageLap incorrect", Double.NaN, averageLap, 1e-2);
      
      timer.start();
      
      double lapElapsed = timer.lapElapsed();
      double totalElapsed = timer.totalElapsed();
      averageLap = timer.averageLap();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, 1e-2);
      assertEquals("totalElapsed incorrect", 0.0, totalElapsed, 1e-2);
      assertEquals("averageLap incorrect", Double.NaN, averageLap, 1e-2);
      
      double sleepTime1 = 0.5;
      yoTime.add(sleepTime1);
      
      double lap = timer.lap();
      averageLap = timer.averageLap();
      assertEquals("lap incorrect", sleepTime1, lap, 1e-2);
      assertEquals("averageLap incorrect", sleepTime1, averageLap, 1e-2);
      
      double sleepTime2 = 1.0;
      yoTime.add(sleepTime2);
      
      lap = timer.lap();
      averageLap = timer.averageLap();
      assertEquals("lap incorrect", sleepTime2, lap, 1e-2);
      assertEquals("averageLap incorrect", (sleepTime1 + sleepTime2) / 2.0, averageLap, 1e-2);
      
      timer.resetLap();
      lapElapsed = timer.lapElapsed();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, 1e-2);
      
      lap = timer.lap();
      averageLap = timer.averageLap();
      assertEquals("lap incorrect", 0.0, lap, 1e-2);
      assertEquals("averageLap incorrect", (sleepTime1 + sleepTime2) / 3.0, averageLap, 1e-2);
      
      double sleepTime3 = 0.3;
      yoTime.add(sleepTime3);
      
      lapElapsed = timer.lapElapsed();
      totalElapsed = timer.totalElapsed();
      assertEquals("lapElapsed incorrect", sleepTime3, lapElapsed, 1e-2);
      assertEquals("totalElapsed incorrect", sleepTime1 + sleepTime2 + sleepTime3, totalElapsed, 1e-2);
      
      timer.reset();
      lapElapsed = timer.lapElapsed();
      totalElapsed = timer.totalElapsed();
      averageLap = timer.averageLap();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, 1e-2);
      assertEquals("totalElapsed incorrect", 0.0, totalElapsed, 1e-2);
      assertEquals("averageLap incorrect", Double.NaN, averageLap, 1e-2);
      
      double sleepTime4 = 0.3;
      yoTime.add(sleepTime4);
      
      timer.resetLap();
      lapElapsed = timer.lapElapsed();
      assertEquals("lapElapsed incorrect", 0.0, lapElapsed, 1e-2);
   }
   
   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(YoTimerTest.class);
   }
}
