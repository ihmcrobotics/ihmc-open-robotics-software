package us.ihmc.robotics.time;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class GlobalTimerTest
{
   private static final long RANDOM_SEED = 1976L;
   @ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testgetElapsedTime()
   {
      GlobalTimer globalTimer = new GlobalTimer("timer", new YoVariableRegistry("testRegistry"));

      Random random = new Random(RANDOM_SEED);
      for (int i = 0; i < 10; i++)
      {
         long delay = (long) (random.nextDouble() * 50.0 + 50.0);

         globalTimer.startTimer();

         try
         {
            Thread.sleep(delay);
         }
         catch (InterruptedException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }

         globalTimer.stopTimer();

         assertEquals(delay, globalTimer.getElapsedTime(), 10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 5.2)
   @Test(timeout = 30000)
   public void testgetElapsedTime2()
   {
      GlobalTimer.clearTimers();
      GlobalTimer globalTimer = null;
      String timerName = "timer";
      
      Random random = new Random(RANDOM_SEED);
      for (int i = 0; i < 5; i++)
      {
         globalTimer = new GlobalTimer(timerName + i, new YoVariableRegistry("testRegistry"));

         long delay = (long) (random.nextDouble() * 50.0 + 1000.0);

         globalTimer.startTimer();

         try
         {
            Thread.sleep(delay);
         }
         catch (InterruptedException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }

         globalTimer.stopTimer();
         assertEquals(delay, globalTimer.getElapsedTime(), 10);
         assertTrue(globalTimer.getTimerName().contentEquals("timer" + i));
      }

      ArrayList<GlobalTimer> listOfTimers = new ArrayList<>();
      globalTimer.getAlltimers(listOfTimers);
      assertEquals(5, listOfTimers.size());
      
   }

}
