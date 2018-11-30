package us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction;

public class Sigmoid implements ActivationFunction
{

   @Override
   public double computeOutput(double input)
   {
      
      return 1.0 / (1.0 + Math.exp(-input));
   }

}
