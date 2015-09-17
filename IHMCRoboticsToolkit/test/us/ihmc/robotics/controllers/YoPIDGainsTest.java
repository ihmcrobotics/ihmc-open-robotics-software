package us.ihmc.robotics.controllers;

import static org.junit.Assert.assertEquals;
import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class YoPIDGainsTest
{
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test()
   {
      YoPIDGains pidGains = new YoPIDGains("test", new YoVariableRegistry("PIDGainsRegistry"));

      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double kp = random.nextDouble() * 100;
         double kd = random.nextDouble() * 100;
         double ki = random.nextDouble() * 100;
         double maxAcc = random.nextDouble() * 100;
         double maxJerk = random.nextDouble() * 100;
         double maxIntegralError = random.nextDouble() * 100;
         double zeta = random.nextDouble() * 100;

         pidGains.setKp(kp);
         pidGains.setKd(kd);
         pidGains.setKi(ki);
         pidGains.setMaximumAcceleration(maxAcc);
         pidGains.setMaximumJerk(maxJerk);
         pidGains.setMaximumIntegralError(maxIntegralError);
         pidGains.setZeta(zeta);
         
         
         assertEquals(kp, pidGains.getKp(), 1e-6);
         assertEquals(kd, pidGains.getKd(), 1e-6);
         assertEquals(kp, pidGains.getYoKp().getDoubleValue(), 1e-6);
         assertEquals(kd, pidGains.getYoKd().getDoubleValue(), 1e-6);
         assertEquals(ki, pidGains.getYoKi().getDoubleValue(), 1e-6);
         assertEquals(maxAcc, pidGains.getMaximumAcceleration(), 1e-6);
         assertEquals(maxJerk, pidGains.getMaximumJerk(), 1e-6);
         assertEquals(maxAcc, pidGains.getYoMaximumAcceleration().getDoubleValue(), 1e-6);
         assertEquals(maxJerk, pidGains.getYoMaximumJerk().getDoubleValue(), 1e-6);
         assertEquals(zeta, pidGains.getZeta(), 1e-6);
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testParameters_2()
   {
      YoPIDGains pidGains = new YoPIDGains("test", new YoVariableRegistry("PIDGainsRegistry"));

      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double kp = random.nextDouble() * 100;
         double kd = random.nextDouble() * 100;
         double ki = random.nextDouble() * 100;
         double maxAcc = random.nextDouble() * 100;
         double maxJerk = random.nextDouble() * 100;
         double maxIntegralError = random.nextDouble() * 100;
         double zeta = random.nextDouble() * 100;
         
         pidGains.setPDGains(kp, zeta);
         pidGains.setKd(kd);
         pidGains.setKi(ki);
         pidGains.setMaximumAcceleration(maxAcc);
         pidGains.setMaximumJerk(maxJerk);
         pidGains.setMaximumIntegralError(maxIntegralError);
         
         
         assertEquals(kp, pidGains.getKp(), 1e-6);
         assertEquals(kd, pidGains.getKd(), 1e-6);
         assertEquals(kp, pidGains.getYoKp().getDoubleValue(), 1e-6);
         assertEquals(kd, pidGains.getYoKd().getDoubleValue(), 1e-6);
         assertEquals(ki, pidGains.getYoKi().getDoubleValue(), 1e-6);
         assertEquals(maxAcc, pidGains.getMaximumAcceleration(), 1e-6);
         assertEquals(maxJerk, pidGains.getMaximumJerk(), 1e-6);
         assertEquals(maxAcc, pidGains.getYoMaximumAcceleration().getDoubleValue(), 1e-6);
         assertEquals(maxJerk, pidGains.getYoMaximumJerk().getDoubleValue(), 1e-6);
         assertEquals(zeta, pidGains.getZeta(), 1e-6);
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testParameters_3()
   {
      YoPIDGains pidGains = new YoPIDGains("test", new YoVariableRegistry("PIDGainsRegistry"));

      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double kp = random.nextDouble() * 100;
         double kd = random.nextDouble() * 100;
         double ki = random.nextDouble() * 100;
         double maxAcc = random.nextDouble() * 100;
         double maxJerk = random.nextDouble() * 100;
         double maxIntegralError = random.nextDouble() * 100;
         double zeta = random.nextDouble() * 100;
         
         pidGains.setPIDGains(kp, zeta, ki, maxIntegralError);
         pidGains.setKd(kd);
         pidGains.setMaximumAcceleration(maxAcc);
         pidGains.setMaximumJerk(maxJerk);
         
         assertEquals(kp, pidGains.getKp(), 1e-6);
         assertEquals(kd, pidGains.getKd(), 1e-6);
         assertEquals(kp, pidGains.getYoKp().getDoubleValue(), 1e-6);
         assertEquals(kd, pidGains.getYoKd().getDoubleValue(), 1e-6);
         assertEquals(ki, pidGains.getYoKi().getDoubleValue(), 1e-6);
         assertEquals(maxAcc, pidGains.getMaximumAcceleration(), 1e-6);
         assertEquals(maxJerk, pidGains.getMaximumJerk(), 1e-6);
         assertEquals(maxAcc, pidGains.getYoMaximumAcceleration().getDoubleValue(), 1e-6);
         assertEquals(maxJerk, pidGains.getYoMaximumJerk().getDoubleValue(), 1e-6);
         assertEquals(zeta, pidGains.getZeta(), 1e-6);
         assertEquals(maxIntegralError, pidGains.getYoMaxIntegralError().getDoubleValue(), 1e-6);
      }
   }

}
