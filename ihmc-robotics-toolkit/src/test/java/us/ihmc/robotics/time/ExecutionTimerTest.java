package us.ihmc.robotics.time;

import java.util.Random;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ExecutionTimerTest
{
   private static final long RANDOM_SEED = 1976L;

   @Test
   public void test()
   {
      ExecutionTimer executionTimer = new ExecutionTimer("executionTimer", 0.0, new YoRegistry("testRegistry"));
      
      long max = 0;
      
      Random random = new Random(RANDOM_SEED);
      
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
         Assertions.assertEquals(delay, executionTimer.getCurrentTime().getDoubleValue() * 1000.0, 10);
      }
      
      Assertions.assertEquals(max, executionTimer.getMaxTime().getDoubleValue() * 1000.0, 10);
   }
   
   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(ExecutionTimer.class, ExecutionTimerTest.class);
   }
}
