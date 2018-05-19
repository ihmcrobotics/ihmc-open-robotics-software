package us.ihmc.robotics.functionApproximation.NeuralNetwork.importing;

import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.robotics.functionApproximation.NeuralNetwork.NeuralNetwork;

public class NeuralNetworkYamlHelper
{
   private NeuralNetworkYamlHelper()
   {
   }

   public static NeuralNetwork createNeuralNetworkFromYamlFile(InputStream inputStream)
   {
      Yaml yaml = new Yaml();
      NeuralNetworkConfiguration config = (NeuralNetworkConfiguration) yaml.load(inputStream);
      NeuralNetwork nn = new NeuralNetwork(config);

      return nn;
   }

   public static void saveNeuralNetworkConfigurationToYamlFile(NeuralNetworkConfiguration config, String fileName)
   {
      Yaml yaml = new Yaml();
      String result = yaml.dump(config);
      FileWriter fileWriter;

      try
      {
         fileWriter = new FileWriter(fileName);
         fileWriter.write(result);
         fileWriter.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   /**
    * This is an example configuration created using TensorFlow.
    */
   public static void main(String[] args) throws IOException
   {
      //The neural network doesn't care about this, but the input data should match this ordering
      String[] inputVariableNames = new String[] {"LEFT_ANKLE_PITCH_joint_q", "LEFT_ANKLE_PITCH_joint_q_previous", "LEFT_ANKLE_PITCH_joint_qd",
            "LEFT_ANKLE_PITCH_joint_qd_previous", "LEFT_ANKLE_PITCH_joint_qdd", "LEFT_ANKLE_PITCH_joint_qdd_previous", "LEFT_ANKLE_PITCH_tauMeasured_previous",
            "LEFT_KNEE_PITCH_joint_q", "LEFT_KNEE_PITCH_joint_q_previous", "LEFT_KNEE_PITCH_joint_qd", "LEFT_KNEE_PITCH_joint_qd_previous",
            "LEFT_KNEE_PITCH_joint_qdd", "LEFT_KNEE_PITCH_joint_qdd_previous", "LEFT_KNEE_PITCH_tauMeasured_previous"};

      //This is hacky, but leaving something blank gets a passthrough activation function.
      String[] activationFunctionsPerLayer = new String[] {"", "RELU", ""};

      //The input layer has 16 neurons, one per input variable, the next has 4 neurons, and the output is a single neuron
      int[] numberOfNeuronsPerLayer = new int[] {16, 4, 1};

      //these are the constants added to each of the hidden layer neurons
      double[] hiddenLayerBias = new double[] {0.8374003, 0.03701664, -0.28729317, 0.01319704};

      //this is the constant added to the output neuron
      double[] outputBias = new double[] {-0.3590592};

      //the input layer bias's aren't used but need to be included to make things consistent. then we pass in the hidden layer bias and the output layer bias
      double[][] bias = new double[][] {new double[16], hiddenLayerBias, outputBias};

      //the input layer weights aren't used but need to be included to make things consistent. 
      double[][] inputLayerWeights = new double[16][1];

      //the weights between the input layer and the hidden layer
      double[][] hiddenLayerWeights = new double[4][];
      hiddenLayerWeights[0] = new double[] {-0.46510178, -0.05915822, 0.3717158, -0.45220408, 0.1210492, -0.12430278, -0.29257497, 0.3202149, 0.10641811,
            0.45007122, 0.24379735, -0.13266943, -0.1242077, 0.12984337, 0.62552196, 0.3651869};
      hiddenLayerWeights[1] = new double[] {-0.03552369, 0.7017831, -0.16389109, 0.2945423, -0.10112676, 0.10519658, 0.7274673, -0.2770513, -0.5848242,
            -0.08602782, 0.1309489, -0.2813644, 0.09275991, -0.099305, 1.612328, -0.42894414};
      hiddenLayerWeights[2] = new double[] {0.15122245, 0.5346764, 0.01194376, 0.33071777, -0.05709987, 0.05674522, -0.5670206, 0.0768759, -0.31645796,
            -0.13648617, 0.38817182, -0.31028637, 0.02570818, -0.03258159, 0.1956599, -0.17547709};
      hiddenLayerWeights[3] = new double[] {-0.50018334, -0.5318647, -0.24139157, 0.05813099, 0.15277725, -0.15938585, -0.71538013, 0.7469889, 0.17892408,
            0.76543343, 0.12041978, 0.10104298, -0.15151536, 0.160927, -2.4808238, 0.554581};

      //the weights between the hidden layer and the output layer
      double[][] ouputLayerWeights = new double[1][];
      ouputLayerWeights[0] = new double[] {0.46576336, 0.41093794, -0.01115055, -0.27883196};

      //all the weights together
      double[][][] weights = new double[][][] {inputLayerWeights, hiddenLayerWeights, ouputLayerWeights};

      //save them all in the configuration
      NeuralNetworkConfiguration neuralNetworkConfiguration = new NeuralNetworkConfiguration();
      neuralNetworkConfiguration.setInputVariableNames(inputVariableNames);
      neuralNetworkConfiguration.setActivationFunctionsPerLayer(activationFunctionsPerLayer);
      neuralNetworkConfiguration.setBias(bias);
      neuralNetworkConfiguration.setNumberOfNeuronsPerLayer(numberOfNeuronsPerLayer);
      neuralNetworkConfiguration.setWeights(weights);

      NeuralNetworkYamlHelper.saveNeuralNetworkConfigurationToYamlFile(neuralNetworkConfiguration, "testNNParam.yaml");

      // if you want to make the NN by hand you can simple call
      //      neuralNetwork.createInputLayer(16);
      //      neuralNetwork.createLayer(4, hiddenLayerBias, hiddenLayerWeights, new Relu());
      //      neuralNetwork.createLayer(1, outputBias, ouputLayerWeights, new PassThrough());

      FileInputStream fileInputStream = new FileInputStream("testNNParam.yaml");
      NeuralNetwork neuralNetwork = NeuralNetworkYamlHelper.createNeuralNetworkFromYamlFile(fileInputStream);

      neuralNetwork.setInput(new double[] {0.7041736, 0.7041736, -0.82175085, -0.7660606, -55.92842308, -55.55744086, -0.19960167, 0.20344828, 0.48069055,
            0.48069055, 0.07482435, 0.0718136, 3.15835607, 2.86869009, 1.14953754, -1.47241379});
      //ground truth 1.0482758620689654

      double[] output = new double[1];
      neuralNetwork.compute(output);
      System.out.println(output[0]);
   }
}
