package us.ihmc.manipulation.planning.gradientDescent;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
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
      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            // power function.
            return Math.pow((values.get(0) - expectedOptimalInput) * 10, 2.0) + desiredQuery;
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initial);

      TDoubleArrayList upperLimit = new TDoubleArrayList();
      upperLimit.add(35.0);
      solver.setInputUpperLimit(upperLimit);

      System.out.println("iteration is " + solver.run());
      TDoubleArrayList optimalSolution = solver.getOptimalInput();
      for (int i = 0; i < optimalSolution.size(); i++)
         System.out.println("solution is " + optimalSolution.get(i));

      System.out.println("optimal query is " + solver.getOptimalQuery());

      double error = Math.abs(solver.getOptimalQuery() - desiredQuery);
      double expectedInputError = Math.abs(optimalSolution.get(0) - expectedOptimalInput);

      assertTrue("query arrived on desired value", error < 10E-5);
      assertTrue("input arrived on expected value", expectedInputError < 10E-5);

      System.out.println("Good Bye Test");
   }
}