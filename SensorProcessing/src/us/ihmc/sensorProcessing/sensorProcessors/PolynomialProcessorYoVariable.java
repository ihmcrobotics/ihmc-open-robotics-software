package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

/**
 * Processing YoVariable that computes the output by applying a {@link YoPolynomial} to an input variable.
 */
public class PolynomialProcessorYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final YoDouble input;
   private final YoPolynomial polynomial;

   public PolynomialProcessorYoVariable(String name, YoDouble input, YoPolynomial polynomial, YoVariableRegistry registry)
   {
      super(name, registry);

      this.input = input;
      this.polynomial = polynomial;
   }

   @Override
   public void update()
   {
      polynomial.compute(input.getDoubleValue());
      this.set(polynomial.getPosition());
   }
}
