package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;

class HeuristicICPControllerHelperTest
{

   @Test
   void testNoAdjustmentExamples()
   {
      double adjustedICP = 0.3;
      double firstIntersection = -0.2;
      double secondIntersection = 0.2;
      double firstPerfect = -0.1;
      double secondPerfect = 0.1;
      double minICPPushDelta = 0.1;
      double copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      assertEquals(copAdjustment, 0.0, 1e-7);
   }

   @Test
   void testAdjustToMidpointExamples()
   {
      double adjustedICP = 0.5;
      double firstIntersection = 0.1;
      double secondIntersection = 0.4;
      double firstPerfect = 0.2;
      double secondPerfect = 0.3;
      double minICPPushDelta = 0.1;
      double copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      double expectedAdjustment = 0.25;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      // As you shift to the midpoint, just keep adjusting to the midpoint till you get there. 
      // These will all slow things down, which should add robustness.
      double shiftTowardICP = 0.01;

      for (int i = 0; i < 25; i++)
      {
         adjustedICP -= shiftTowardICP;
         firstIntersection -= shiftTowardICP;
         secondIntersection -= shiftTowardICP;
         firstPerfect -= shiftTowardICP;
         secondPerfect -= shiftTowardICP;
         copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

         expectedAdjustment -= shiftTowardICP;
         assertEquals(expectedAdjustment, copAdjustment, 1e-7);
      }

      //TODO: When you want to slow down instead of speed up, then not sure if you should keep on adjusting.

   }

   @Test
   void testMinICPPushDeltaExamples()
   {
      double adjustedICP = 0.5;
      double firstIntersection = 0.1;
      double secondIntersection = 0.4;
      double firstPerfect = 0.2;
      double secondPerfect = 0.3;
      double minICPPushDelta = 0.0;
      double copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      double expectedAdjustment = 0.25;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      // As the icpPushDelta grows, make sure not to project into the push region, unless you must to get to the support polygon.
      double increaseMinICPPushDelta = 0.01;

      for (int i = 0; i < 25; i++)
      {
         minICPPushDelta += increaseMinICPPushDelta;
         copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

         assertEquals(expectedAdjustment, copAdjustment, 1e-7);
      }

      // Now as you increase it further, you cannot adjust past the minPushDelta.
      for (int i = 0; i < 15; i++)
      {
         minICPPushDelta += increaseMinICPPushDelta;
         copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

         expectedAdjustment -= increaseMinICPPushDelta;
         assertEquals(expectedAdjustment, copAdjustment, 1e-7);
      }

      // Now as you increase it further, you need to make sure to stay inside the polygon at least:
      for (int i = 0; i < 25; i++)
      {
         minICPPushDelta += increaseMinICPPushDelta;
         copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

         expectedAdjustment = firstIntersection;
         assertEquals(expectedAdjustment, copAdjustment, 1e-7);
      }
   }

   @Test
   void testPushingBackwardSoJustGoOnEdge()
   {
      double adjustedICP = 0.35;
      double firstIntersection = 0.2;
      double secondIntersection = 0.5;
      double firstPerfect = 0.3;
      double secondPerfect = 0.4;
      double minICPPushDelta = 0.15;
      double copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      double expectedAdjustment = 0.2;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      for (int i = 0; i < 50; i++)
      {
         adjustedICP -= 0.01;

         copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
         assertEquals(expectedAdjustment, copAdjustment, 1e-7);
      }
   }

   @Test
   void testPushingToEdgeOfPerfectRegion()
   {
      double adjustedICP = 0.5;
      double firstIntersection = 0.1;
      double secondIntersection = 0.4;
      double firstPerfect = 0.15;
      double secondPerfect = 0.2;
      double minICPPushDelta = 0.1;
      double copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      double expectedAdjustment = 0.2;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      firstPerfect = 0.3;
      secondPerfect = 0.35;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      expectedAdjustment = 0.3;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      firstPerfect = 0.1;
      secondPerfect = 0.15;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      expectedAdjustment = 0.15;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      firstPerfect = 0.05;
      secondPerfect = 0.1;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      expectedAdjustment = 0.1;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);
   }

   @Test
   void testPushingToEdgeOfIntersectionRegion()
   {
      double adjustedICP = 0.5;
      double firstIntersection = 0.1;
      double secondIntersection = 0.4;
      double firstPerfect = 0.05;
      double secondPerfect = 0.1;
      double minICPPushDelta = 0.01;
      double copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      double expectedAdjustment = 0.1;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      for (int i = 0; i < 50; i++)
      {
         firstPerfect -= 0.01;
         secondPerfect -= 0.01;

         copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
         assertEquals(expectedAdjustment, copAdjustment, 1e-7);
      }

      adjustedICP = 0.5;
      firstIntersection = 0.1;
      secondIntersection = 0.4;
      firstPerfect = 0.3;
      secondPerfect = 0.35;
      minICPPushDelta = 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      expectedAdjustment = 0.3;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      for (int i = 0; i < 10; i++)
      {
         firstPerfect += 0.01;
         secondPerfect += 0.01;

         expectedAdjustment += 0.01;

         copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
         assertEquals(expectedAdjustment, copAdjustment, 1e-7);
      }

      for (int i = 0; i < 9; i++)
      {
         firstPerfect += 0.01;
         secondPerfect += 0.01;

         copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
         assertEquals(expectedAdjustment, copAdjustment, 1e-7);
      }

   }

   @Test
   void testRandom()
   {
      Random random = new Random(1776L);

      int numberOfTests = 100000;

      for (int i = 0; i < numberOfTests; i++)
      {

         double adjustedICP = 0.01 + 0.50 * random.nextDouble();
         double firstIntersection = randomInInterval(random, -0.5, 0.5);
         double secondIntersection = firstIntersection + 0.01 + 0.5 * random.nextDouble();
         double firstPerfect = randomInInterval(random, -0.5, 0.5);
         double secondPerfect = firstPerfect + 0.01 + 0.5 * random.nextDouble();
         double minICPPushDelta = 0.01 + 0.50 * random.nextDouble();
         double copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

         assertTrue(copAdjustment >= firstIntersection);
         assertTrue(copAdjustment <= secondIntersection);

         if ((firstPerfect > firstIntersection) && (firstPerfect < secondIntersection) && (firstPerfect < adjustedICP - minICPPushDelta))
            assertTrue(copAdjustment >= firstPerfect);

         if ((secondPerfect < secondIntersection) && (secondPerfect > firstIntersection))
            assertTrue(copAdjustment <= secondPerfect);
      }
   }

   private double randomInInterval(Random random, double first, double second)
   {
      return first + (second - first) * random.nextDouble();
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(HeuristicICPControllerHelper.class, HeuristicICPControllerHelperTest.class);
   }
}
