package us.ihmc.robotics.controllers;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class PDControllerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testComputerForAngles()
   {
      PDController pdController = new PDController("suffix", new YoVariableRegistry("testRegistry"));

      pdController.setDerivativeGain(0.0);
      pdController.setProportionalGain(0.0);

      double value = pdController.computeForAngles(0, Math.toRadians(90), 0, 0);
      assertEquals(0.0, value, 1e-6);

      pdController.setDerivativeGain(1.0);
      pdController.setProportionalGain(1.0);

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         double current = 0;
         double desired = random.nextDouble() * Math.PI;
         value = pdController.computeForAngles(current, desired, 0, 0);
         assertEquals(desired, value, 1e-6);
      }

      for (int i = 0; i < 100; i++)
      {
         double current = Math.PI;
         double desired = random.nextDouble() * Math.PI + Math.PI;
         value = pdController.computeForAngles(current, desired, 0, 0);
         assertEquals(desired - Math.PI, value, 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAngleBoundaries()
   {
      PDController pdController = new PDController("suffix", new YoVariableRegistry("testRegistry"));
      pdController.setDerivativeGain(1.0);
      pdController.setProportionalGain(1.0);
      
      Random random = new Random();
      
      for(int i = 0; i < 1000; i++)
      {
          double current = Math.PI - random.nextDouble()*Math.toRadians(89);
          double desired = -(Math.PI - random.nextDouble()*Math.toRadians(89));
          double value = pdController.computeForAngles(current, desired, 0, 0);
          double error = (Math.PI - current) + (Math.PI - Math.abs(desired));
          
          assertEquals((Math.PI - current) + (Math.PI - Math.abs(desired)), value, 1e-6);
      }
   }
}
