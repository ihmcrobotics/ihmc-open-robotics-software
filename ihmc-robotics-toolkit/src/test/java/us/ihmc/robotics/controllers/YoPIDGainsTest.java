package us.ihmc.robotics.controllers;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoPIDGainsTest
{
   @Test
   public void test()
   {
      YoPIDGains pidGains = new YoPIDGains("test", new YoRegistry("PIDGainsRegistry"));

      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double kp = random.nextDouble() * 100;
         double kd = random.nextDouble() * 100;
         double ki = random.nextDouble() * 100;
         double maxAcc = random.nextDouble() * 100;
         double maxJerk = random.nextDouble() * 100;
         double maxIntegralError = random.nextDouble() * 100;
         double integratorLeakRatio = random.nextDouble();
         double zeta = random.nextDouble() * 100;

         pidGains.setKp(kp);
         pidGains.setKd(kd);
         pidGains.setKi(ki);
         pidGains.setMaximumFeedback(maxAcc);
         pidGains.setMaximumFeedbackRate(maxJerk);
         pidGains.setMaximumIntegralError(maxIntegralError);
         pidGains.setIntegralLeakRatio(integratorLeakRatio);
         pidGains.setZeta(zeta);


         assertEquals(kp, pidGains.getKp(), 1e-6);
         assertEquals(kd, pidGains.getKd(), 1e-6);
         assertEquals(kp, pidGains.getYoKp().getDoubleValue(), 1e-6);
         assertEquals(kd, pidGains.getYoKd().getDoubleValue(), 1e-6);
         assertEquals(ki, pidGains.getYoKi().getDoubleValue(), 1e-6);
         assertEquals(maxAcc, pidGains.getMaximumFeedback(), 1e-6);
         assertEquals(maxJerk, pidGains.getMaximumFeedbackRate(), 1e-6);
         assertEquals(maxAcc, pidGains.getYoMaximumFeedback().getDoubleValue(), 1e-6);
         assertEquals(maxJerk, pidGains.getYoMaximumFeedbackRate().getDoubleValue(), 1e-6);
         assertEquals(integratorLeakRatio, pidGains.getYoIntegralLeakRatio().getDoubleValue(), 1e-6);
         assertEquals(zeta, pidGains.getZeta(), 1e-6);
      }
   }

   @Test
   public void testParameters_2()
   {
      YoPIDGains pidGains = new YoPIDGains("test", new YoRegistry("PIDGainsRegistry"));

      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double kp = random.nextDouble() * 100;
         double kd = random.nextDouble() * 100;
         double ki = random.nextDouble() * 100;
         double maxAcc = random.nextDouble() * 100;
         double maxJerk = random.nextDouble() * 100;
         double maxIntegralError = random.nextDouble() * 100;
         double integratorLeakRatio = random.nextDouble();
         double zeta = random.nextDouble() * 100;

         pidGains.setPDGains(kp, zeta);
         pidGains.setKd(kd);
         pidGains.setKi(ki);
         pidGains.setMaximumFeedback(maxAcc);
         pidGains.setMaximumFeedbackRate(maxJerk);
         pidGains.setMaximumIntegralError(maxIntegralError);
         pidGains.setIntegralLeakRatio(integratorLeakRatio);


         assertEquals(kp, pidGains.getKp(), 1e-6);
         assertEquals(kd, pidGains.getKd(), 1e-6);
         assertEquals(kp, pidGains.getYoKp().getDoubleValue(), 1e-6);
         assertEquals(kd, pidGains.getYoKd().getDoubleValue(), 1e-6);
         assertEquals(ki, pidGains.getYoKi().getDoubleValue(), 1e-6);
         assertEquals(maxAcc, pidGains.getMaximumFeedback(), 1e-6);
         assertEquals(maxJerk, pidGains.getMaximumFeedbackRate(), 1e-6);
         assertEquals(maxAcc, pidGains.getYoMaximumFeedback().getDoubleValue(), 1e-6);
         assertEquals(maxJerk, pidGains.getYoMaximumFeedbackRate().getDoubleValue(), 1e-6);
         assertEquals(integratorLeakRatio, pidGains.getYoIntegralLeakRatio().getDoubleValue(), 1e-6);
         assertEquals(zeta, pidGains.getZeta(), 1e-6);
      }
   }

   @Test
   public void testParameters_3()
   {
      YoPIDGains pidGains = new YoPIDGains("test", new YoRegistry("PIDGainsRegistry"));

      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double kp = random.nextDouble() * 100;
         double kd = random.nextDouble() * 100;
         double ki = random.nextDouble() * 100;
         double maxAcc = random.nextDouble() * 100;
         double maxJerk = random.nextDouble() * 100;
         double maxIntegralError = random.nextDouble() * 100;
         double integratorLeakRatio = random.nextDouble();
         double zeta = random.nextDouble() * 100;

         pidGains.setPIDGains(kp, zeta, ki, maxIntegralError);
         pidGains.setKd(kd);
         pidGains.setMaximumFeedback(maxAcc);
         pidGains.setMaximumFeedbackRate(maxJerk);
         pidGains.setIntegralLeakRatio(integratorLeakRatio);

         assertEquals(kp, pidGains.getKp(), 1e-6);
         assertEquals(kd, pidGains.getKd(), 1e-6);
         assertEquals(kp, pidGains.getYoKp().getDoubleValue(), 1e-6);
         assertEquals(kd, pidGains.getYoKd().getDoubleValue(), 1e-6);
         assertEquals(ki, pidGains.getYoKi().getDoubleValue(), 1e-6);
         assertEquals(maxAcc, pidGains.getMaximumFeedback(), 1e-6);
         assertEquals(maxJerk, pidGains.getMaximumFeedbackRate(), 1e-6);
         assertEquals(maxAcc, pidGains.getYoMaximumFeedback().getDoubleValue(), 1e-6);
         assertEquals(maxJerk, pidGains.getYoMaximumFeedbackRate().getDoubleValue(), 1e-6);
         assertEquals(zeta, pidGains.getZeta(), 1e-6);
         assertEquals(integratorLeakRatio, pidGains.getYoIntegralLeakRatio().getDoubleValue(), 1e-6);
         assertEquals(maxIntegralError, pidGains.getYoMaxIntegralError().getDoubleValue(), 1e-6);
      }
   }

   @Test
   public void testClippingLeakRate()
   {
      YoPIDGains pidGains = new YoPIDGains("test", new YoRegistry("PIDGainsRegistry"));

      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double integratorLeakRatio = random.nextDouble() * 100;
         pidGains.setIntegralLeakRatio(integratorLeakRatio);

         assertTrue(pidGains.getYoIntegralLeakRatio().getDoubleValue() <= 1.0);

         integratorLeakRatio = -random.nextDouble() * 100;
         pidGains.setIntegralLeakRatio(integratorLeakRatio);

         assertTrue(pidGains.getYoIntegralLeakRatio().getDoubleValue() >= 0.0);
      }
   }

}
