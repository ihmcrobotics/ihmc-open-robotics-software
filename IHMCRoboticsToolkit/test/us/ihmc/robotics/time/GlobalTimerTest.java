package us.ihmc.robotics.time;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class GlobalTimerTest
{
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testgetElapsedTime()
   {
      GlobalTimer globalTimer = new GlobalTimer("timer", new YoVariableRegistry("testRegistry"));

      Random random = new Random();
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
         assertEquals(delay, globalTimer.getElapsedTime(), 1);
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testgetElapsedTime2()
   {
      Random random = new Random();
      for (int i = 0; i < 5; i++)
      {
         GlobalTimer globalTimer = new GlobalTimer("timer", new YoVariableRegistry("testRegistry"));

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
         assertEquals(delay, globalTimer.getElapsedTime(), 1);
      }
   }

}
