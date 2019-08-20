package us.ihmc.quadrupedFootstepPlanning.pawPlanning.turnWalkTurn;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class QuadrupedTurnWalkTurnPathPlannerTest
{
   private static final double epsilon = 1e-10;
   private static final int iters = 1000;

   @Test
   public void testComputeTimeToAccelerateToAchieveValueWithNoMaxRate()
   {
      double delta = 0.25;
      double maxAcceleration = 1.0;
      double time = QuadrupedTurnWalkTurnPathPlanner.computeTimeToAccelerateToAchieveValueWithNoMaxRate(0.0, 0.0, delta, 0.0, maxAcceleration);
      double timeExpected = Math.sqrt(delta / maxAcceleration);

      assertEquals(timeExpected, time, epsilon);

      // flip the sign, and see what happens
      time = QuadrupedTurnWalkTurnPathPlanner.computeTimeToAccelerateToAchieveValueWithNoMaxRate(0.0, 0.0, -delta, 0.0, maxAcceleration);

      assertEquals(timeExpected, time, epsilon);
   }

   @Test
   public void testLargestQuadraticSolution()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         double a1 = RandomNumbers.nextDouble(random, 100);
         double a2 = RandomNumbers.nextDouble(random, 100);
         double b1 = RandomNumbers.nextDouble(random, 100);
         double b2 = RandomNumbers.nextDouble(random, 100);

         double a = a1 * a2;
         double b = (a1 * b2 + a2 * b1);
         double c = b1 * b2;

         double expectedValue = Math.max(positiveQuadraticSolution(a, b, c), negativeQuadraticSolution(a, b, c));

         assertEquals(expectedValue, QuadrupedTurnWalkTurnPathPlanner.largestQuadraticSolution(a, b, c), epsilon);
      }
   }

   private static double positiveQuadraticSolution(double a, double b, double c)
   {
      double radical = Math.sqrt(MathTools.square(b) - 4.0 * a * c);
      return (-b + radical) / (2.0 * a);
   }

   private static double negativeQuadraticSolution(double a, double b, double c)
   {
      double radical = Math.sqrt(MathTools.square(b) - 4.0 * a * c);
      return (-b - radical) / (2.0 * a);
   }

}
