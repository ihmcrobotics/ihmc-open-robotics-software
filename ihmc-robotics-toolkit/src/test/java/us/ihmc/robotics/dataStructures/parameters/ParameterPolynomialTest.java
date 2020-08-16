package us.ihmc.robotics.dataStructures.parameters;

import java.util.Random;

import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;

import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ParameterPolynomialTest
{
   @Test
   public void testAgainstYoPolynomial()
   {
      Random random = new Random(438218L);

      for (int test = 0; test < 1000; test++)
      {
         YoRegistry registry = new YoRegistry("Test");
         double[] coefficients = new double[random.nextInt(10) + 1];
         for (int i = 0; i < coefficients.length; i++)
         {
            coefficients[i] = 10.0 * (random.nextDouble() - 0.5);
         }

         YoPolynomial yoPolynomial = new YoPolynomial("Yo", coefficients.length, registry);
         yoPolynomial.setDirectly(coefficients);
         ParameterPolynomial parameterPolynomial = new ParameterPolynomial("Parameter", coefficients, registry);

         DefaultParameterReader reader = new DefaultParameterReader();
         reader.readParametersInRegistry(registry);

         for (int valueIdx = 0; valueIdx < 100; valueIdx++)
         {
            double x = 100.0 * (random.nextDouble() - 0.5);

            yoPolynomial.compute(x);
            double yoResult = yoPolynomial.getPosition();

            parameterPolynomial.compute(x);
            double parameterResult = parameterPolynomial.getPosition();

            Assert.assertEquals(yoResult, parameterResult, Double.MIN_VALUE);
         }
      }
   }
}
