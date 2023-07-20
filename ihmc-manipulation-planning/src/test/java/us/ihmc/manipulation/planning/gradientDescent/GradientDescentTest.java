package us.ihmc.manipulation.planning.gradientDescent;

import gnu.trove.list.array.TDoubleArrayList;
import org.junit.jupiter.api.Test;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

import static us.ihmc.robotics.Assert.assertTrue;
public class GradientDescentTest
{
   @Test
   public void testGradientDescent()
   {
      System.out.println("Hello Test");

      double initialInput = 35.0;
      double desiredQuery = 5.0;
      double expectedOptimalInput = 10.0;

      TDoubleArrayList initial = new TDoubleArrayList();
      initial.add(initialInput);
      initial.add(initialInput);
      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            // power function.
            double query = desiredQuery;
            for (int i = 0; i < values.size(); i++)
            {
               query += Math.pow((values.get(i) - expectedOptimalInput) * 10, 2.0);
            }
            return query;
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initial);
      solver.setUnboundedStepSize(20.0);
      solver.setReducingStepSizeRatio(1.5);
      solver.setVerbose(true);

      TDoubleArrayList upperLimit = new TDoubleArrayList();
      upperLimit.add(35.0);
      upperLimit.add(35.0);
      solver.setInputUpperLimit(upperLimit);
      TDoubleArrayList lowerLimit = new TDoubleArrayList();
      lowerLimit.add(-40.0);
      lowerLimit.add(-40.0);
      solver.setInputLowerLimit(lowerLimit);

      System.out.println("iteration is " + solver.run());
      TDoubleArrayList optimalSolution = solver.getOptimalInput();
      for (int i = 0; i < optimalSolution.size(); i++)
         System.out.println("solution is " + optimalSolution.get(i));

      System.out.println("optimal query is " + solver.getOptimalQuery());

      double error = Math.abs(solver.getOptimalQuery() - desiredQuery);
      double expectedInputError = Math.abs(optimalSolution.get(0) - expectedOptimalInput);

      assertTrue("query arrived on desired value", error < 10E-5);
      assertTrue("input arrived on expected value", expectedInputError < 10E-4);

      System.out.println("Good Bye Test");
   }

   @Test
   public void testGradientDescentBounded()
   {
      System.out.println("Hello Test");

      double initialInput = 35.0;
      double desiredQuery = 5.0;
      double unboundedOptimalInput = 10.0;
      double lowerBoundAboveOptimal = 5.0;
      double boundedOptimalInput = unboundedOptimalInput + lowerBoundAboveOptimal;

      TDoubleArrayList initial = new TDoubleArrayList();
      initial.add(initialInput);
      initial.add(initialInput);
      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            // power function.
            double query = desiredQuery;
            for (int i = 0; i < values.size(); i++)
            {
               query += Math.pow((values.get(i) - unboundedOptimalInput) * 10, 2.0);
            }
            return query;
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initial);
      solver.setUnboundedStepSize(20.0);
      solver.setStepSizeRatio(0.5);
      solver.setReducingStepSizeRatio(1.5);
      solver.setVerbose(true);

      TDoubleArrayList upperLimit = new TDoubleArrayList();
      upperLimit.add(35.0);
      upperLimit.add(35.0);
      solver.setInputUpperLimit(upperLimit);
      TDoubleArrayList lowerLimit = new TDoubleArrayList();
      lowerLimit.add(boundedOptimalInput);
      lowerLimit.add(boundedOptimalInput);
      solver.setInputLowerLimit(lowerLimit);

      System.out.println("iteration is " + solver.run());
      TDoubleArrayList optimalSolution = solver.getOptimalInput();
      for (int i = 0; i < optimalSolution.size(); i++)
         System.out.println("solution is " + optimalSolution.get(i));

      System.out.println("optimal query is " + solver.getOptimalQuery());

      double error = Math.abs(solver.getOptimalQuery() - desiredQuery);
      double expectedInputError = Math.abs(optimalSolution.get(0) - boundedOptimalInput);

//      assertTrue("query arrived on desired value", error < 10E-5);
      assertTrue("input arrived on expected value", expectedInputError < 10E-4);

      System.out.println("Good Bye Test");
   }

   @Test
   public void testGradientDescentStartingLowerBound()
   {
      System.out.println("Hello Test");

      double initialInput = -40.0;
      double desiredQuery = 5.0;
      double expectedOptimalInput = 10.0;

      TDoubleArrayList initial = new TDoubleArrayList();
      initial.add(initialInput);
      initial.add(initialInput);
      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            // power function.
            double query = desiredQuery;
            for (int i = 0; i < values.size(); i++)
            {
               query += Math.pow((values.get(i) - expectedOptimalInput) * 10, 2.0);
            }
            return query;
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initial);
      solver.setUnboundedStepSize(20.0);
      solver.setReducingStepSizeRatio(1.5);
      solver.setVerbose(true);

      TDoubleArrayList upperLimit = new TDoubleArrayList();
      upperLimit.add(35.0);
      upperLimit.add(35.0);
      solver.setInputUpperLimit(upperLimit);
      TDoubleArrayList lowerLimit = new TDoubleArrayList();
      lowerLimit.add(-40.0);
      lowerLimit.add(-40.0);
      solver.setInputLowerLimit(lowerLimit);

      System.out.println("iteration is " + solver.run());
      TDoubleArrayList optimalSolution = solver.getOptimalInput();
      for (int i = 0; i < optimalSolution.size(); i++)
         System.out.println("solution is " + optimalSolution.get(i));

      System.out.println("optimal query is " + solver.getOptimalQuery());

      double error = Math.abs(solver.getOptimalQuery() - desiredQuery);
      double expectedInputError = Math.abs(optimalSolution.get(0) - expectedOptimalInput);

      assertTrue("query arrived on desired value", error < 10E-5);
      assertTrue("input arrived on expected value", expectedInputError < 10E-4);

      System.out.println("Good Bye Test");
   }


}