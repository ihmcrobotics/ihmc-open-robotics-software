package us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction;

/**
 * The "transfer function" used to process the output of a neuron
 *
 */
public interface ActivationFunction
{
   public double computeOutput(double input);
}
