package us.ihmc.robotics.controllers;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class YoPDGainsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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
          double deadband = rand.nextDouble() * 100;


          gains.setKd(kd);
          gains.setKp(kp);
          gains.setMaximumFeedback(maxAcc);
          gains.setMaximumFeedbackRate(maxJerk);
          gains.setZeta(zeta);
          gains.setPositionDeadband(deadband);

          assertEquals(kp, gains.getKp(), 1e-6);
          assertEquals(kp, gains.getYoKp().getDoubleValue(), 1e-6);
          assertEquals(kd, gains.getKd(), 1e-6);
          assertEquals(kd, gains.getYoKd().getDoubleValue(), 1e-6);
          assertEquals(maxAcc, gains.getMaximumFeedback(), 1e-6);
          assertEquals(maxAcc, gains.getYoMaximumFeedback().getDoubleValue(), 1e-6);
          assertEquals(maxJerk, gains.getMaximumFeedbackRate(), 1e-6);
          assertEquals(maxJerk, gains.getYoMaximumFeedbackRate().getDoubleValue(), 1e-6);
          assertEquals(zeta, gains.getZeta(), 1e-6);
          assertEquals(zeta, gains.getYoZeta().getDoubleValue(), 1e-6);
          assertEquals(deadband, gains.getPositionDeadband().getDoubleValue(), 1e-6);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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
          gains.setMaximumFeedbackAndMaximumFeedbackRate(maxAcc, maxJerk);
          
          assertEquals(kp, gains.getKp(), 1e-6);
          assertEquals(0.0, gains.getKd(), 1e-6);
          assertEquals(maxAcc, gains.getMaximumFeedback(), 1e-6);
          assertEquals(maxJerk, gains.getMaximumFeedbackRate(), 1e-6);
          assertEquals(zeta, gains.getZeta(), 1e-6);
          assertEquals(0.0, gains.getPositionDeadband().getDoubleValue(), 1e-6);
      }
   }

}
