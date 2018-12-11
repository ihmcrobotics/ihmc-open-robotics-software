package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.planning.ContactState;

import java.util.Random;

import static junit.framework.TestCase.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class CoMTrajectoryPlannerToolsTest
{
   private static final double epsilon = 1e-8;
   private static final int iters = 1000;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFirstCoefficientPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MAX_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientPositionMultiplier(ContactState.NO_CONTACT, timeInPhase, omega);
         expectedMultiplier = timeInPhase;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSecondCoefficientPositionMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MAX_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientPositionMultiplier(ContactState.NO_CONTACT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFirstCoefficientVelocityMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientVelocityMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = omega * Math.exp(omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MAX_VALUE;

         assertTrue("time = " + timeInPhase, Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(Math.min(omega * CoMTrajectoryPlannerTools.getFirstCoefficientPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega), Double.MAX_VALUE), multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getFirstCoefficientVelocityMultiplier(ContactState.NO_CONTACT, timeInPhase, omega);
         expectedMultiplier = 1.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSecondCoefficientVelocityMultiplier()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double timeInPhase = RandomNumbers.nextDouble(random, 0.0, 10000);
         double omega = RandomNumbers.nextDouble(random, 1.0, 10.0);
         double multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientVelocityMultiplier(ContactState.IN_CONTACT, timeInPhase, omega);

         double expectedMultiplier = -omega * Math.exp(-omega * timeInPhase);
         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
         assertEquals(Math.min(-omega * CoMTrajectoryPlannerTools.getSecondCoefficientPositionMultiplier(ContactState.IN_CONTACT, timeInPhase, omega), Double.MAX_VALUE), multiplier, epsilon);


         multiplier = CoMTrajectoryPlannerTools.getSecondCoefficientVelocityMultiplier(ContactState.NO_CONTACT, timeInPhase, omega);
         expectedMultiplier = 0.0;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier);
      }
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
         double multiplier = CoMTrajectoryPlannerTools.getGravityPositionEffect(ContactState.IN_CONTACT, timeInPhase, gravity);
         assertEquals(0.0, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getGravityPositionEffect(ContactState.NO_CONTACT, timeInPhase, gravity);
         double expectedMultiplier =  -gravity * timeInPhase * timeInPhase;

         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
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
         double multiplier = CoMTrajectoryPlannerTools.getGravityVelocityEffect(ContactState.IN_CONTACT, timeInPhase, gravity);
         assertEquals(0.0, multiplier, epsilon);

         multiplier = CoMTrajectoryPlannerTools.getGravityVelocityEffect(ContactState.NO_CONTACT, timeInPhase, gravity);
         double expectedMultiplier =  -2.0 * gravity * timeInPhase;

         if (!Double.isFinite(expectedMultiplier))
            expectedMultiplier = Double.MIN_VALUE;

         assertTrue(Double.isFinite(multiplier));
         assertEquals(expectedMultiplier, multiplier, epsilon);
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFirstCoefficientIndex()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int sequence = RandomNumbers.nextInt(random, 0, 10000);
         int index = CoMTrajectoryPlannerTools.getFirstCoefficientIndex(sequence);
         int expectedIndex = 2 * sequence;

         assertEquals(expectedIndex, index);

      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSecondCoefficientIndex()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int sequence = RandomNumbers.nextInt(random, 0, 10000);
         int index = CoMTrajectoryPlannerTools.getSecondCoefficientIndex(sequence);
         int expectedIndex = 2 * sequence + 1;

         assertEquals(expectedIndex, index);

      }
   }
}
