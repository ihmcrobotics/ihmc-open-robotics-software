package us.ihmc.robotics.functionApproximation.NeuralNetwork.importing;

import us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction.ActivationFunction;
import us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction.PassThrough;
import us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction.Relu;
import us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction.Sigmoid;

public class NeuralNetworkConfiguration
{
   private String[] inputVariableNames;
   private String[] activationFunctionsPerLayer;
   private int[] numberOfNeuronsPerLayer;
   private double[][] bias;
   private double[][][] weights;

   public NeuralNetworkConfiguration()
   {

   }

   public String[] getActivationFunctionsPerLayer()
   {
      return activationFunctionsPerLayer;
   }

   public void setActivationFunctionsPerLayer(String[] activationFunctionsPerLayer)
   {
      this.activationFunctionsPerLayer = activationFunctionsPerLayer;
   }

   public int[] getNumberOfNeuronsPerLayer()
   {
      return numberOfNeuronsPerLayer;
   }

   public void setNumberOfNeuronsPerLayer(int[] numberOfNeuronsPerLayer)
   {
      this.numberOfNeuronsPerLayer = numberOfNeuronsPerLayer;
   }

   public double[][] getBias()
   {
      return bias;
   }

   public void setBias(double[][] bias)
   {
      this.bias = bias;
   }

   public double[][][] getWeights()
   {
      return weights;
   }

   public void setWeights(double[][][] weights)
   {
      this.weights = weights;
   }

   public ActivationFunction[] getActivationFunctions()
   {
      ActivationFunction[] activationFunctions = new ActivationFunction[activationFunctionsPerLayer.length];
      for(int i = 0; i < activationFunctionsPerLayer.length; i++)
      {
         switch(activationFunctionsPerLayer[i])
         {
         case "RELU":
            activationFunctions[i] = new Relu();
            break;
         case "SIGMOID":
            activationFunctions[i] = new Sigmoid();
            break;
         default:
            activationFunctions[i] = new PassThrough();
            break;
         }
      }
      return activationFunctions;
   }

   public String[] getInputVariableNames()
   {
      return inputVariableNames;
   }

   public void setInputVariableNames(String[] inputVariableNames)
   {
      this.inputVariableNames = inputVariableNames;
   }
}
