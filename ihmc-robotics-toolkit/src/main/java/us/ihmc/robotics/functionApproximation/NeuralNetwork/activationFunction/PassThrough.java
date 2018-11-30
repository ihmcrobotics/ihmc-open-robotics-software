package us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction;

/**
 * The output = the input 
 * Useful for the input Neurons
 */
public class PassThrough implements ActivationFunction
{

   @Override
   public double computeOutput(double input)
   {
      return input;
   }
}
