package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.trajectories.yoVariables.YoPolynomial;
import us.ihmc.yoVariables.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Processing YoVariable that computes the output by applying a {@link YoPolynomial} to an input variable.
 */
public class PolynomialProcessorYoVariable extends YoDouble implements OffsettingProcessorVariable
{
   private final YoDouble input;
   private final PolynomialReadOnly polynomial;

   public PolynomialProcessorYoVariable(String name, YoDouble input, PolynomialReadOnly polynomial, YoRegistry registry)
   {
      super(name, registry);

      this.input = input;
      this.polynomial = polynomial;
   }

   @Override
   public void update()
   {
      polynomial.compute(input.getDoubleValue());
      this.set(polynomial.getValue());
   }

   @Override
   public double getOffset()
   {
      return input.getDoubleValue() - this.getDoubleValue();
   }
}
