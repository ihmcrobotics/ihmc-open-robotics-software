package us.ihmc.robotics.dataStructures.parameters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.robotics.dataStructures.PolynomialReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ParameterPolynomial implements PolynomialReadOnly
{
   private final List<DoubleProvider> coefficients;

   private double value;

   public ParameterPolynomial(String prefix, int numberOfCoefficients, YoRegistry registry)
   {
      List<DoubleProvider> coefficients = new ArrayList<>();
      for (int index = 0; index < numberOfCoefficients; index++)
      {
         coefficients.add(new DoubleParameter(prefix + "Coefficient" + index, registry));
      }
      this.coefficients = Collections.unmodifiableList(coefficients);
   }

   public ParameterPolynomial(String prefix, double[] defaultCoefficients, YoRegistry registry)
   {
      List<DoubleProvider> coefficients = new ArrayList<>();
      for (int index = 0; index < defaultCoefficients.length; index++)
      {
         coefficients.add(new DoubleParameter(prefix + "Coefficient" + index, registry, defaultCoefficients[index]));
      }
      this.coefficients = Collections.unmodifiableList(coefficients);
   }

   @Override
   public void compute(double x)
   {
      double x_n = 1.0;
      value = 0.0;

      for (int index = 0; index < coefficients.size(); index++)
      {
         value = value + coefficients.get(index).getValue() * x_n;
         x_n = x_n * x;
      }
   }

   @Override
   public double getPosition()
   {
      return value;
   }

}
