package us.ihmc.robotics.time;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.tools.testing.TestPlanAnnotations;

public class ExecutionTimerTest
{
   @TestPlanAnnotations.DeployableTestMethod(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void test()
   {
      ExecutionTimer executionTimer = new ExecutionTimer("executionTimer", 0.0, new YoVariableRegistry("testRegistry"));
      
      long max = 0;
      
      Random random = new Random();
      
      for (int i = 0; i < 10; i++)
      {
         long delay = (long) (random.nextDouble() * 50.0 + 50.0);
         
         if(delay > max)
         {
        	 max = delay;
         }

         executionTimer.startMeasurement();

         try
         {
            Thread.sleep(delay);
         }
         catch (InterruptedException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }

         executionTimer.stopMeasurement();
         assertEquals(delay, executionTimer.getCurrentTime().getDoubleValue()*1000.0, 1);
      }
      
      assertEquals(max, executionTimer.getMaxTime().getDoubleValue()*1000.0, 1);

   }

}
