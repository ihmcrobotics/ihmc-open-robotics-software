package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.Random;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.fail;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.LinearCoMTrajectoryPlannerTools.sufficientlyLarge;

public class LinearCoMTrajectoryPlannerToolsTest
{
   private static final double epsilon = 1e-8;
   private static final int iters = 1000;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFirstCoefficientCoMPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier) || expectedMultiplier > sufficientlyLarge)
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = timeInPhase;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSecondCoefficientCoMPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetThirdCoefficientCoMPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double multiplier = LinearCoMTrajectoryPlannerTools.getThirdCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR, timeInPhase);

         double expectedMultiplier = timeInPhase;
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getThirdCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.CONSTANT, timeInPhase);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFourthCoefficientCoMPositionMultiplier()
   {
      double multiplier = LinearCoMTrajectoryPlannerTools.getFourthCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      double expectedMultiplier = 1.0;
      assertTrue(Double.isFinite(multiplier));
      assertEquals(expectedMultiplier, multiplier, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFirstCoefficientCoMVelocityMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = omega * Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier) || expectedMultiplier > sufficientlyLarge)
            expectedMultiplier = sufficientlyLarge;

         assertTrue("time = " + timeInPhase, Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(Math.min(omega * LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega),
                               sufficientlyLarge), multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMVelocityMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSecondCoefficientCoMVelocityMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = -omega * Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(Math.min(-omega * LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega),
                               sufficientlyLarge), multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMVelocityMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 0.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetThirdCoefficientCoMVelocityMultiplier()
   {
      double multiplier = LinearCoMTrajectoryPlannerTools.getThirdCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(1.0, multiplier, epsilon);

      multiplier = LinearCoMTrajectoryPlannerTools.getThirdCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, ContactMotion.CONSTANT);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFourthCoefficientCoMVelocityMultiplier()
   {
      double multiplier = LinearCoMTrajectoryPlannerTools.getFourthCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFirstCoefficientCoMAccelerationMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = omega * omega * Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier)  || expectedMultiplier > sufficientlyLarge)
            expectedMultiplier = sufficientlyLarge;

         assertTrue("time = " + timeInPhase, Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(
               Math.min(omega * omega * LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega),
                        sufficientlyLarge), multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMAccelerationMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSecondCoefficientAccelerationMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = omega * omega * Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(
               Math.min(omega * omega * LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega),
                        sufficientlyLarge), multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMAccelerationMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 0.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetThirdCoefficientCoMAccelerationMultiplier()
   {
      double multiplier = LinearCoMTrajectoryPlannerTools.getThirdCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);

      multiplier = LinearCoMTrajectoryPlannerTools.getThirdCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFourthCoefficientCoMAccelerationMultiplier()
   {
      double multiplier = LinearCoMTrajectoryPlannerTools.getFourthCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFirstCoefficientVRPPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier) || expectedMultiplier > sufficientlyLarge)
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = timeInPhase;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSecondCoefficientVRPPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetThirdCoefficientVRPPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double multiplier = LinearCoMTrajectoryPlannerTools.getThirdCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR, timeInPhase);

         double expectedMultiplier = timeInPhase;
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getThirdCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.CONSTANT, timeInPhase);

         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFourthCoefficientVRPPositionMultiplier()
   {
      double multiplier = LinearCoMTrajectoryPlannerTools.getFourthCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      double expectedMultiplier = 1.0;
      assertTrue(Double.isFinite(multiplier));
      assertEquals(expectedMultiplier, multiplier, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetGravityPositionEffect()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double gravity = RandomNumbers.nextDouble(random, 8.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getGravityPositionEffect(ContactState.IN_CONTACT, timeInPhase, gravity);
         assertEquals(0.0, multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getGravityPositionEffect(ContactState.FLIGHT, timeInPhase, gravity);
         double expectedMultiplier = -0.5 * gravity * timeInPhase * timeInPhase;

         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, 100 * epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetGravityVelocityEffect()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double gravity = RandomNumbers.nextDouble(random, 8.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getGravityVelocityEffect(ContactState.IN_CONTACT, timeInPhase, gravity);
         assertEquals(0.0, multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getGravityVelocityEffect(ContactState.FLIGHT, timeInPhase, gravity);
         double expectedMultiplier = -gravity * timeInPhase;

         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetGravityAccelerationEffect()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double gravity = RandomNumbers.nextDouble(random, 8.0, 10.0);
         double multiplier = LinearCoMTrajectoryPlannerTools.getGravityAccelerationEffect(ContactState.IN_CONTACT, gravity);
         assertEquals(0.0, multiplier, epsilon);

         multiplier = LinearCoMTrajectoryPlannerTools.getGravityAccelerationEffect(ContactState.FLIGHT, gravity);
         double expectedMultiplier = -gravity;

         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
      }
   }
}
