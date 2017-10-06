package us.ihmc.robotics.functionApproximation;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * <p>Title: LinearMapping</p>
 *
 * <p>Description: Defines a mapping from an n dimensional input space to an n dimensional output space, where the
 * transformation between the two is a simple scaling of each dimension.</p>
 *
 * @author jrebula
 * @version 1.0
 */
public class LinearMapping
{
   private ArrayList<double[]> inputDimensions;
   private ArrayList<double[]> outputDimensions;

   /**
    * Creates a mapping in n space to another n space that performs a simple scale in each dimension
    *
    * @param inputDimensions ArrayList list of length n, each element is a double array of length two, containing the min and max values for that dimension in the input space
    * @param outputDimensions ArrayList list of length n, each element is a double array of length two, containing the min and max values for that dimension in the output space
    */
   public LinearMapping(ArrayList<double[]> inputDimensions, ArrayList<double[]> outputDimensions)
   {
      this.inputDimensions = new ArrayList<double[]>();
      this.outputDimensions = new ArrayList<double[]>();

      if (inputDimensions.size() != outputDimensions.size())
      {
         throw new IllegalArgumentException("must have the same number of dimensions in the input and output spaces");
      }

      for (int i = 0; i < inputDimensions.size(); i++)
      {
         double[] inputLimits = inputDimensions.get(i);
         double[] outputLimits = outputDimensions.get(i);

         if ((inputLimits.length != 2) || (outputLimits.length != 2))
         {
            throw new IllegalArgumentException("each element of the input and output dimension arrays must be 2 (a min and a max)");
         }

         double[] inputLimitsCopy = new double[2];
         double[] outputLimitsCopy = new double[2];

         System.arraycopy(inputLimits, 0, inputLimitsCopy, 0, 2);
         System.arraycopy(outputLimits, 0, outputLimitsCopy, 0, 2);

         this.inputDimensions.add(inputLimitsCopy);
         this.outputDimensions.add(outputLimitsCopy);
      }

//    this.inputDimensions.addAll(inputDimensions);
//    this.outputDimensions.addAll(outputDimensions);
   }

   public double[] mapFromInputSpaceToOutputSpace(double[] input)
   {
      double[] output = new double[input.length];
      mapFromInputSpaceToOutputSpacePacked(input, output);

      return output;
   }

   public void writeOut(PrintWriter printWriter)
   {
      printWriter.println(inputDimensions.size());

      for (double[] inputArray : inputDimensions)
      {
         printWriter.println(Arrays.toString(inputArray));
      }

      for (double[] inputArray : outputDimensions)
      {
         printWriter.println(Arrays.toString(inputArray));
      }
   }

//   public static LinearMapping readIn(BufferedReader bufferedReader) throws IOException
//   {
//      ArrayList<double[]> inputDimensions = new ArrayList<double[]>();
//      ArrayList<double[]> outputDimensions = new ArrayList<double[]>();
//
//      int numberOfDimensions = Integer.parseInt(bufferedReader.readLine());
//      for (int i = 0; i < numberOfDimensions; i++)
//      {
//         double[] inputArray = ArrayTools.parseDoubleArray(bufferedReader);
//         inputDimensions.add(inputArray);
//      }
//
//      for (int i = 0; i < numberOfDimensions; i++)
//      {
//         double[] outputArray = ArrayTools.parseDoubleArray(bufferedReader);
//         outputDimensions.add(outputArray);
//      }
//
//      LinearMapping ret = new LinearMapping(inputDimensions, outputDimensions);
//
//      return ret;
//   }

   /**
    * Takes a vector in the input space, and copies the mapped vector in the output space into the supplied output array.
    *
    * @param input double[]
    * @param output double[]
    */
   public void mapFromInputSpaceToOutputSpacePacked(double[] input, double[] output)
   {
      if (input.length != inputDimensions.size())
      {
         throw new IllegalArgumentException(
             "must provide a vector in the input space with the same dimension of the inputDimensions vector used to create the mapping");
      }

      if (output.length != inputDimensions.size())
      {
         throw new IllegalArgumentException(
             "must provide a vector in the output space with the same dimension of the inputDimensions vector used to create the mapping");
      }

      for (int i = 0; i < inputDimensions.size(); i++)
      {
         double alpha = getAlphaFromValue(input[i], inputDimensions.get(i));
         output[i] = getValueFromAlpha(alpha, outputDimensions.get(i));
      }
   }

   public double[] mapFromOutputSpaceToInputSpace(double[] output)
   {
      double[] input = new double[output.length];
      mapFromOutputSpaceToInputSpacePacked(input, output);

      return input;
   }

   /**
    * Takes a vector in the output space, and copies the mapped vector in the input space into the supplied input array.
    * Note that the ordering of the arguments is (input, output)
    *
    * @param input double[]
    * @param output double[]
    */
   public void mapFromOutputSpaceToInputSpacePacked(double[] input, double[] output)
   {
      if (input.length != inputDimensions.size())
      {
         throw new IllegalArgumentException(
             "must provide a vector in the input space with the same dimension of the inputDimensions vector used to create the mapping");
      }

      if (output.length != inputDimensions.size())
      {
         throw new IllegalArgumentException(
             "must provide a vector in the output space with the same dimension of the inputDimensions vector used to create the mapping");
      }

      for (int i = 0; i < inputDimensions.size(); i++)
      {
         double alpha = getAlphaFromValue(output[i], outputDimensions.get(i));
         double outputValue = getValueFromAlpha(alpha, inputDimensions.get(i));
         input[i] = outputValue;
      }
   }

   private double getAlphaFromValue(double value, double[] limits)
   {
      return (value - limits[0]) / (limits[1] - limits[0]);
   }

   private double getValueFromAlpha(double alpha, double[] limits)
   {
      return limits[0] + (alpha * (limits[1] - limits[0]));
   }

}
