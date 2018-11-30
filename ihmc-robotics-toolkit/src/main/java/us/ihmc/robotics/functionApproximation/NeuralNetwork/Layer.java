package us.ihmc.robotics.functionApproximation.NeuralNetwork;

import java.util.ArrayList;

/**
 * A single layer of a Neural Network
 */
public class Layer
{
   ArrayList<Neuron> neurons = new ArrayList<>();

   public void addNeuron(Neuron neuron)
   {
      neurons.add(neuron);
   }

   public void compute()
   {
      for (int i = 0; i < neurons.size(); i++)
      {
         Neuron neuron = neurons.get(i);
         neuron.compute();
      }
   }

   public ArrayList<Neuron> getNeurons()
   {
      return neurons;
   }
}
