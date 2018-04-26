package us.ihmc.robotics.functionApproximation.NeuralNetwork;

import java.util.ArrayList;

import us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction.ActivationFunction;
import us.ihmc.robotics.functionApproximation.NeuralNetwork.activationFunction.PassThrough;
import us.ihmc.robotics.functionApproximation.NeuralNetwork.importing.NeuralNetworkConfiguration;

/**
 * Simple garbage free production neural network with no training ability. Train your Neural Network using an external library like TensorFlow and import the parameters. 
 * Does not currently support feedback. 
 */
public class NeuralNetwork
{
   private final ArrayList<Layer> layers = new ArrayList<>();
   
   public NeuralNetwork()
   {
   }

   /**
    * Construct a neural network using a NeuralNetworkConfiguration, this is useful if you want to save and load your NN parameters to a YAML file
    * @param config the neural network configuration
    */
   public NeuralNetwork(NeuralNetworkConfiguration config)
   {
      int[] numberOfNeuronsPerLayer = config.getNumberOfNeuronsPerLayer();
      createInputLayer(numberOfNeuronsPerLayer[0]);
      
      ActivationFunction[] activationFunctions = config.getActivationFunctions();
      double[][] bias = config.getBias();
      double[][][] weights = config.getWeights();
      
      //we already cretaed the input layer so start with an index of one
      for(int i = 1; i < numberOfNeuronsPerLayer.length; i++)
      {
         createLayer(numberOfNeuronsPerLayer[i], bias[i], weights[i], activationFunctions[i]);
      }
   }

   /**
    * Creates the initial layer with zero bias and a PassThrough Activation function (the output = the input)
    * @param numberOfNeurons the number of input variables
    */
   public void createInputLayer(int numberOfNeurons)
   {
      if (layers.size() > 0)
      {
         throw new IllegalArgumentException("Neural Network already contains a layer, make sure you create your input layer first! (or fix this class to be more modular)");
      }
      PassThrough passThrough = new PassThrough();
      Layer layer = new Layer();
      for(int i = 0; i < numberOfNeurons; i++)
      {
         layer.addNeuron(new Neuron(passThrough, 0.0));
      }
      layers.add(layer);
   }
   
   /**
    * Creates a hidden layer and connects every node of this layer to every node of the previous layer
    * @param numberOfNeurons the number of neurons in this neural network
    * @param bias the constant added to the output of the neuron
    * @param weights the weight used to scale the inputs. 
    *    [this layer neuron 0][previous layer neuron 0 weight, previous layer neuron 1 weight, etc]
    *    [this layer neuron 1][previous layer neuron 0 weight, previous layer neuron 1 weight, etc]
    * @param activationFunction the "transfer function" used by the neuron to compute the output
    */
   public void createLayer(int numberOfNeurons, double[] bias, double[][] weights, ActivationFunction activationFunction)
   {
      if (layers.size() < 1)
      {
         throw new IllegalArgumentException("Neural Network does not contain an input layer, make sure you create your input layer first! (or fix this class to be more modular)");
      }
      
      Layer previousLayer = layers.get(layers.size() - 1);
      ArrayList<Neuron> previousLayerNeurons = previousLayer.getNeurons();
      
      Layer layer = new Layer();
      
      for(int currentLayerNeuronIndex = 0; currentLayerNeuronIndex < numberOfNeurons; currentLayerNeuronIndex++)
      {
         Neuron neuron = new Neuron(activationFunction, bias[currentLayerNeuronIndex]);
         for(int previousLayerNeuronIndex = 0 ; previousLayerNeuronIndex < previousLayerNeurons.size(); previousLayerNeuronIndex++)
         {
            double weight = weights[currentLayerNeuronIndex][previousLayerNeuronIndex];
            Neuron inputNeuron = previousLayerNeurons.get(previousLayerNeuronIndex);
            neuron.addInputNeuron(inputNeuron, weight);
         }
         layer.addNeuron(neuron);
      }
      layers.add(layer);
   }
   
   /**
    * The input of the neural network. input.length should equal layer zero's number of neurons
    * @param input, index 0 is neuron 0, index 1 is neuron 1, etc
    */
   public void setInput(double[] input)
   {
      Layer inputLayer = layers.get(0);
      ArrayList<Neuron> inputNeurons = inputLayer.getNeurons();
      if (input.length != inputNeurons.size())
      {
         throw new IllegalArgumentException("input array does not equal NN input size");
      }
      
      for(int i = 0; i < inputNeurons.size(); i++)
      {
         Neuron neuron = inputNeurons.get(i);
         // Input variables are stored as bias in the input layer
         neuron.setBias(input[i]);
      }
   }

   /**
    * computes the output of the NN based on the current inputs
    * @param output the results stored in the last layer, output.length should equal the number of neurons in the last layer
    */
   public void compute(double[] output)
   {
      Layer outputLayer = layers.get(layers.size() - 1);
      ArrayList<Neuron> outputNeurons = outputLayer.getNeurons();

      if (output.length != outputNeurons.size())
      {
         throw new IllegalArgumentException("output array does not equal NN output size");
      }

      for (int i = 0; i < layers.size(); i++)
      {
         layers.get(i).compute();
      }

      for (int i = 0; i < outputNeurons.size(); i++)
      {
         output[i] = outputNeurons.get(i).getOutput();
      }
   }
}
