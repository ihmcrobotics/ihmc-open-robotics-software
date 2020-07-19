package us.ihmc.robotics.time;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CallFrequencyCalculatorTest
{
   @Test
   public void testDetermineCallFrequency()
   {
      CallFrequencyCalculator callFrequencyCalculator = new CallFrequencyCalculator(new YoRegistry("test"), "");
      callFrequencyCalculator.setNumberOfSamples(12);

      int desiredFreq = 20;
      long delay = (long) (1000.0 / desiredFreq);

      for (int i = 0; i < 100; i++)
      {
         try
         {
            Thread.sleep(delay);
         }
         catch (InterruptedException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }

         double freq = callFrequencyCalculator.determineCallFrequency();

         if (freq != 0.0)
         {
            assertEquals(desiredFreq, freq, 1);
         }
      }
   }

}
