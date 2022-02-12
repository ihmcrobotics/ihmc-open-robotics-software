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
   void testDoNotMoveWhenMinICPPushDeltaIsLarge()
   {
      double adjustedICP = 0.05;
      double firstIntersection = -0.05;
      double secondIntersection = 0.3;
      double firstPerfect = -0.05;
      double secondPerfect = 0.3;
      double minICPPushDelta = 1.0;
      double copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      double expectedAdjustment = 0.0;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

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

   @Test
   void testTheTwentyNineCombosIgnoringPerfect()
   {
      // First intersection is in front of ICP. Gotta push backward. Best is stay on that edge. Does not matter what minICPPushDelta is...
      double adjustedICP = 1.0;
      double firstIntersection = 1.1;
      double secondIntersection = 1.2;
      double firstPerfect = firstIntersection;
      double secondPerfect = secondIntersection;
      double minICPPushDelta = 0.0;
      double copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      double expectedAdjustment = firstIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = 1.2;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      // First intersection is behind ICP. Midpoint is in front of ICP. minICPPushDelta is important to consider. 
      // Move as close to the ICP as allowed, considering both the first intersection and the minICPPushDelta
      firstIntersection = 0.9;
      secondIntersection = 1.3;
      firstPerfect = firstIntersection;
      secondPerfect = secondIntersection;

      minICPPushDelta = 0.05;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      expectedAdjustment = adjustedICP - minICPPushDelta;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = 0.2;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      expectedAdjustment = firstIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = 1.2;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      expectedAdjustment = firstIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      // First intersection is behind ICP. Midpoint is now behind ICP. minICPPushDelta is important to consider. 
      // Move as close to the midpoint as allowed, considering both the first intersection and the minICPPushDelta

      firstIntersection = 0.7;
      secondIntersection = 1.1;
      firstPerfect = firstIntersection;
      secondPerfect = secondIntersection;

      double midPoint = 0.5 * (firstIntersection + secondIntersection);

      minICPPushDelta = 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);

      expectedAdjustment = midPoint;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - midPoint + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = adjustedICP - minICPPushDelta;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - 0.05;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = firstIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP + 0.05;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = firstIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      // Both intersections and midpoint are behind ICP. minICPPushDelta is important to consider. 
      // Move as close to the midpoint as allowed, considering both the first intersection and the minICPPushDelta
      firstIntersection = 0.4;
      secondIntersection = 0.8;
      firstPerfect = firstIntersection;
      secondPerfect = secondIntersection;

      midPoint = 0.5 * (firstIntersection + secondIntersection);

      minICPPushDelta = 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = midPoint;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - secondIntersection + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = midPoint;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - midPoint + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = adjustedICP - minICPPushDelta;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - firstIntersection + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = firstIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = firstIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      // First intersection is behind unprojected CoP. Second intersection and midpoint are in front of CoP.
      // minICPPushDelta is important to consider. 
      // Move as close to the midpoint as allowed, considering the minICPPushDelta

      firstIntersection = -0.1;
      secondIntersection = 0.5;
      firstPerfect = firstIntersection;
      secondPerfect = secondIntersection;

      midPoint = 0.5 * (firstIntersection + secondIntersection);

      minICPPushDelta = 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = midPoint;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - secondIntersection + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = midPoint;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - midPoint + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = adjustedICP - minICPPushDelta;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = 0.0;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - firstIntersection + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = 0.0;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      //TODO: Not sure if this is the behavior we want! This will speed things up a lot, which reduces robustness...

      // First intersection and midpoint are behind unprojected CoP. Second intersection is in front of CoP.
      // minICPPushDelta is important to consider. 
      // Move as close to the midpoint as allowed, considering the minICPPushDelta
      
      // Or: Don't move at all!!??

      firstIntersection = -0.5;
      secondIntersection = 0.1;
      firstPerfect = firstIntersection;
      secondPerfect = secondIntersection;

      midPoint = 0.5 * (firstIntersection + secondIntersection);

      minICPPushDelta = 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = 0.0; //midPoint;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - secondIntersection + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = 0.0; //midPoint;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = 0.0; //midPoint;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - midPoint + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = 0.0;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - firstIntersection + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = 0.0;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      //    TODO: Not sure if this is the behavior we want! This will speed things up a lot, which reduces robustness...

      // Both intersections and midpoint are behind unprojected CoP.
      // minICPPushDelta is important to consider. 
      // Move as close to the midpoint as allowed, considering the minICPPushDelta

      firstIntersection = -0.8;
      secondIntersection = -0.2;
      firstPerfect = firstIntersection;
      secondPerfect = secondIntersection;

      midPoint = 0.5 * (firstIntersection + secondIntersection);

      minICPPushDelta = 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = secondIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = secondIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - secondIntersection + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = secondIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - midPoint + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = secondIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);

      minICPPushDelta = adjustedICP - firstIntersection + 0.01;
      copAdjustment = HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP, firstIntersection, secondIntersection, firstPerfect, secondPerfect, minICPPushDelta);
      expectedAdjustment = secondIntersection;
      assertEquals(expectedAdjustment, copAdjustment, 1e-7);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(HeuristicICPControllerHelper.class, HeuristicICPControllerHelperTest.class);
   }
}
