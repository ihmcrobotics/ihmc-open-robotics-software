package us.ihmc.robotics.controllers;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class YoPDGainsTest
{
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testParameters()
   {
      YoPDGains gains = new YoPDGains("pdGains", new YoVariableRegistry("testRegistry"));

      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
          double kp = rand.nextDouble() * 100;
          double kd = rand.nextDouble() * 100;
          double maxAcc = rand.nextDouble() * 100;
          double maxJerk = rand.nextDouble() * 100;
          double zeta = rand.nextDouble()*100;

          gains.setKd(kd);
          gains.setKp(kp);
          gains.setMaximumAcceleration(maxAcc);
          gains.setMaximumJerk(maxJerk);
          gains.setZeta(zeta);

          assertEquals(kp, gains.getKp(), 1e-6);
          assertEquals(kp, gains.getYoKp().getDoubleValue(), 1e-6);
          assertEquals(kd, gains.getKd(), 1e-6);
          assertEquals(kd, gains.getYoKd().getDoubleValue(), 1e-6);
          assertEquals(maxAcc, gains.getMaximumAcceleration(), 1e-6);
          assertEquals(maxAcc, gains.getYoMaximumAcceleration().getDoubleValue(), 1e-6);
          assertEquals(maxJerk, gains.getMaximumJerk(), 1e-6);
          assertEquals(maxJerk, gains.getYoMaximumJerk().getDoubleValue(), 1e-6);
          assertEquals(zeta, gains.getZeta(), 1e-6);
          assertEquals(zeta, gains.getYoZeta().getDoubleValue(), 1e-6);
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testParameters_2()
   {
      YoPDGains gains = new YoPDGains("pdGains", new YoVariableRegistry("testRegistry"));

      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
          double kp = rand.nextDouble() * 100;
          double maxAcc = rand.nextDouble() * 100;
          double maxJerk = rand.nextDouble() * 100;
          double zeta = rand.nextDouble()*100;

          gains.setPDGains(kp, zeta);
          gains.setMaximumAccelerationAndMaximumJerk(maxAcc, maxJerk);
          
          assertEquals(kp, gains.getKp(), 1e-6);
          assertEquals(0.0, gains.getKd(), 1e-6);
          assertEquals(maxAcc, gains.getMaximumAcceleration(), 1e-6);
          assertEquals(maxJerk, gains.getMaximumJerk(), 1e-6);
          assertEquals(zeta, gains.getZeta(), 1e-6);
      }
   }

}
