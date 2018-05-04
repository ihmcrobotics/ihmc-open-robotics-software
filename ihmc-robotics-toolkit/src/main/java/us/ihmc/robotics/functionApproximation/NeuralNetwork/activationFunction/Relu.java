package us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction;

/**
 * The output = the input if the input > 0 otherwise output = 0
 */
public class Relu implements ActivationFunction
{
   @Override
   public double computeOutput(double input)
   {
      return Math.max(0, input);
   }
}
