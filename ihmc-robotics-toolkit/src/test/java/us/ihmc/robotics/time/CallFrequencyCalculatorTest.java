package us.ihmc.robotics.time;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CallFrequencyCalculatorTest
{
   @Test
   public void testDetermineCallFrequency()
   {
      CallFrequencyCalculator callFrequencyCalculator = new CallFrequencyCalculator(new YoVariableRegistry("test"), "");
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
