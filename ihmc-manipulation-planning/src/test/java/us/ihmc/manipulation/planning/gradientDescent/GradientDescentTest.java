package us.ihmc.manipulation.planning.gradientDescent;

import gnu.trove.list.array.TDoubleArrayList;
import org.junit.jupiter.api.Test;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

import java.util.function.ToDoubleFunction;

import static org.junit.jupiter.api.Assertions.*;

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
      ToDoubleFunction<TDoubleArrayList> function = new ToDoubleFunction<>()
      {
         @Override
         public double applyAsDouble(TDoubleArrayList values)
         {
            // power function.
            return Math.pow((values.get(0) - expectedOptimalInput) * 10, 2.0) + desiredQuery;
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initial);
      solver.setLearningRate(20.0);
      solver.setReducingLearningRateRatio(1.5);

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

      assertTrue(error < 10E-5, "query arrived on desired value");
      assertTrue(expectedInputError < 10E-4, "input arrived on expected value");

      System.out.println("Good Bye Test");
   }
}