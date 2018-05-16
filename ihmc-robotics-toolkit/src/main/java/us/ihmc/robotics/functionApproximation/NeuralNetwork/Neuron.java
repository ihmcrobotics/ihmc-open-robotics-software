package us.ihmc.robotics.functionApproximation.NeuralNetwork;

import java.util.ArrayList;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction.ActivationFunction;

/**
 * Processing unit in a Neural Network
 */
public class Neuron
{
   private final ArrayList<Neuron> inputs = new ArrayList<>();
   private final TObjectDoubleHashMap<Neuron> weights = new TObjectDoubleHashMap<Neuron>();
   private final ActivationFunction activationFunction;
   private double output = 0.0;
   private double bias = 0.0;

   /**
    * Creates a Neuron
    * @param activationFunction the function used to process the output of this Neuron
    * @param bias the constant added to the summed inputs
    */
   public Neuron(ActivationFunction activationFunction, double bias)
   {
      this.activationFunction = activationFunction;
      this.bias = bias; 
   }

   /**
    * Connects a neuron and sets the associated scalar that will be multiplied to it's output
    * @param neuron the input neuron
    * @param weight the scalar
    */
   public void addInputNeuron(Neuron neuron, double weight)
   {
      inputs.add(neuron);
      weights.put(neuron, weight);
   }

   /**
    * Computes the output of this neuron
    */
   public void compute()
   {
      double sum = bias;
      for (int i = 0; i < inputs.size(); i++)
      {
         Neuron neuron = inputs.get(i);
         double weight = weights.get(neuron);
         sum += neuron.getOutput() * weight;
      }
      output = activationFunction.computeOutput(sum);
   }
   
   /**
    * sets the constant added to the output
    * @param bias
    */
   public void setBias(double bias)
   {
      this.bias = bias;
   }

   /**
    * Returns the computed output
    * @return the output after going through the activation function
    */
   public double getOutput()
   {
      return output;
   }
}
