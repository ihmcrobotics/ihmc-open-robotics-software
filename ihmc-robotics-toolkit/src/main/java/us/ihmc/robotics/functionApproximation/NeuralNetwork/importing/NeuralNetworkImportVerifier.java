package us.ihmc.robotics.functionApproximation.NeuralNetwork.importing;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Arrays;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.robotics.functionApproximation.NeuralNetwork.NeuralNetwork;

public class NeuralNetworkImportVerifier
{
   private double[][] inputs;
   private double[][] predictions;
   private double[][] actuals;

   public NeuralNetworkImportVerifier()
   {
   }

   public void setInputs(double[][] inputs)
   {
      this.inputs = inputs;
   }

   public void setPredictions(double[][] predictions)
   {
      this.predictions = predictions;
   }

   public void setActuals(double[][] actuals)
   {
      this.actuals = actuals;
   }

   double[][] getInputs()
   {
      return inputs;
   }

   double[][] getPredictions()
   {
      return predictions;
   }

   double[][] getActuals()
   {
      return actuals;
   }

   public static NeuralNetworkImportVerifier readNeuralNetworkTestPointsFromYaml(FileInputStream inputStream)
   {
      Yaml yaml = new Yaml();
      NeuralNetworkImportVerifier data = (NeuralNetworkImportVerifier) yaml.load(inputStream);
      double[][] testInputs = data.getInputs();
      double[][] testPredictions = data.getPredictions();
      double[][] testActuals = data.getActuals();
      for (int i = 0; i < testInputs.length; i++)
      {
         System.out.println("Input " + i + ": " + Arrays.toString(testInputs[i]));
         System.out.println("Prediction " + i + ": " + Arrays.toString(testPredictions[i]));
         System.out.println("Actual " + i + ": " + Arrays.toString(testActuals[i]));
      }
      return data;
   }

   public static boolean isApproximatelyEqual(double[] array1, double[] array2, double eps)
   {
      assert array1.length == array2.length;

      for (int i = 0; i < array1.length; i++)
      {
         double difference = Math.abs(array1[i] - array2[i]);
         // It's more prudent to check for one failure and then quickly exit, rather than check all and only report at the end
         if (difference > eps)
            return false;
      }
      return true;
   }

   public static void main(String[] args) throws IOException
   {
      FileInputStream modelStream = new FileInputStream(
            "ihmc-open-robotics-software/ihmc-robotics-toolkit/resources/functionApproximation/NeuralNetwork/testNNImport.yaml");
      NeuralNetwork model = NeuralNetworkYamlHelper.createNeuralNetworkFromYamlFile(modelStream);

      FileInputStream testPointsStream = new FileInputStream(
            "ihmc-open-robotics-software/ihmc-robotics-toolkit/resources/functionApproximation/NeuralNetwork/testNNImportTestPoints.yaml");
      NeuralNetworkImportVerifier testData = readNeuralNetworkTestPointsFromYaml(testPointsStream);

      // We're going to check if the imported model's predictions match the test predictions, this means the model has imported correctly
      double[][] testInputs = testData.getInputs();
      double[][] testPredictions = testData.getPredictions();
      // As we're only comparing predictions, no need to pull out the actuals

      double[][] importedModelPredictions = new double[testPredictions.length][];
      double[] importedModelPrediction = new double[testPredictions[0].length];
      double eps = 1E-5;
      for (int i = 0; i < testInputs.length; i++)
      {
         model.setInput(testInputs[i]);
         model.compute(importedModelPrediction);
         importedModelPredictions[i] = importedModelPrediction;
         if (isApproximatelyEqual(importedModelPrediction, testPredictions[i], eps))
         {
            System.out.println("Test prediction " + i + ": VALID");
         }
         else
         {
            System.out.println("Test prediction " + i + ": INVALID");
         }
      }
   }
}
