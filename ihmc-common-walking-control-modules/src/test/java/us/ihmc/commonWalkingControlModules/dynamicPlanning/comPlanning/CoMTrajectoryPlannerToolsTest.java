package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

import static junit.framework.TestCase.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLarge;

public class CoMTrajectoryPlannerToolsTest
{
   private static final double epsilon = 1e-8;
   private static final int iters = 1000;

   @Test
   public void testGetFirstCoefficientCoMPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier) || expectedMultiplier > sufficientlyLarge)
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = timeInPhase;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @Test
   public void testGetSecondCoefficientCoMPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @Test
   public void testGetThirdCoefficientCoMPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double multiplier = CoMTrajectoryPlannerTools.getThirdCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR, timeInPhase);

         double expectedMultiplier = timeInPhase;
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getThirdCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.CONSTANT, timeInPhase);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
      }
   }

   @Test
   public void testGetFourthCoefficientCoMPositionMultiplier()
   {
      double multiplier = CoMTrajectoryPlannerTools.getFourthCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      double expectedMultiplier = 1.0;
      assertTrue(Double.isFinite(multiplier));
      assertEquals(expectedMultiplier, multiplier, epsilon);
   }

   @Test
   public void testGetFirstCoefficientCoMVelocityMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = omega * Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier) || expectedMultiplier > sufficientlyLarge)
            expectedMultiplier = sufficientlyLarge;

         assertTrue("time = " + timeInPhase, Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(Math.min(omega * CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega),
                               sufficientlyLarge), multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMVelocityMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @Test
   public void testGetSecondCoefficientCoMVelocityMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = -omega * Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(Math.min(-omega * CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega),
                               sufficientlyLarge), multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMVelocityMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 0.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @Test
   public void testGetThirdCoefficientCoMVelocityMultiplier()
   {
      double multiplier = CoMTrajectoryPlannerTools.getThirdCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(1.0, multiplier, epsilon);

      multiplier = CoMTrajectoryPlannerTools.getThirdCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, ContactMotion.CONSTANT);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);
   }

   @Test
   public void testGetFourthCoefficientCoMVelocityMultiplier()
   {
      double multiplier = CoMTrajectoryPlannerTools.getFourthCoefficientCoMVelocityMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);
   }

   @Test
   public void testGetFirstCoefficientCoMAccelerationMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = omega * omega * Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier)  || expectedMultiplier > sufficientlyLarge)
            expectedMultiplier = sufficientlyLarge;

         assertTrue("time = " + timeInPhase, Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(
               Math.min(omega * omega * CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega),
                        sufficientlyLarge), multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMAccelerationMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @Test
   public void testGetSecondCoefficientAccelerationMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = omega * omega * Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(
               Math.min(omega * omega * CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega),
                        sufficientlyLarge), multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMAccelerationMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 0.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @Test
   public void testGetThirdCoefficientCoMAccelerationMultiplier()
   {
      double multiplier = CoMTrajectoryPlannerTools.getThirdCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);

      multiplier = CoMTrajectoryPlannerTools.getThirdCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);
   }

   @Test
   public void testGetFourthCoefficientCoMAccelerationMultiplier()
   {
      double multiplier = CoMTrajectoryPlannerTools.getFourthCoefficientCoMAccelerationMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      assertTrue(Double.isFinite(multiplier));
      assertEquals(0.0, multiplier, epsilon);
   }

   @Test
   public void testGetFirstCoefficientVRPPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier) || expectedMultiplier > sufficientlyLarge)
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = timeInPhase;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @Test
   public void testGetSecondCoefficientVRPPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(ContactState.FLIGHT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @Test
   public void testGetThirdCoefficientVRPPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double multiplier = CoMTrajectoryPlannerTools.getThirdCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR, timeInPhase);

         double expectedMultiplier = timeInPhase;
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = sufficientlyLarge;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getThirdCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.CONSTANT, timeInPhase);

         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
      }
   }

   @Test
   public void testGetFourthCoefficientVRPPositionMultiplier()
   {
      double multiplier = CoMTrajectoryPlannerTools.getFourthCoefficientCoMPositionMultiplier(ContactState.IN_CONTACT, ContactMotion.LINEAR);

      double expectedMultiplier = 1.0;
      assertTrue(Double.isFinite(multiplier));
      assertEquals(expectedMultiplier, multiplier, epsilon);
   }

   @Test
   public void testGetGravityPositionEffect()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double gravity = RandomNumbers.nextDouble(random, 8.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getGravityPositionEffect(ContactState.IN_CONTACT, timeInPhase, gravity);
         assertEquals(0.0, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getGravityPositionEffect(ContactState.FLIGHT, timeInPhase, gravity);
         double expectedMultiplier = -0.5 * gravity * timeInPhase * timeInPhase;

         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, 100 * epsilon);
      }
   }

   @Test
   public void testGetGravityVelocityEffect()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double gravity = RandomNumbers.nextDouble(random, 8.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getGravityVelocityEffect(ContactState.IN_CONTACT, timeInPhase, gravity);
         assertEquals(0.0, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getGravityVelocityEffect(ContactState.FLIGHT, timeInPhase, gravity);
         double expectedMultiplier = -gravity * timeInPhase;

         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
      }
   }

   @Test
   public void testGetGravityAccelerationEffect()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double gravity = RandomNumbers.nextDouble(random, 8.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getGravityAccelerationEffect(ContactState.IN_CONTACT, gravity);
         assertEquals(0.0, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getGravityAccelerationEffect(ContactState.FLIGHT, gravity);
         double expectedMultiplier = -gravity;

         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
      }
   }
}
