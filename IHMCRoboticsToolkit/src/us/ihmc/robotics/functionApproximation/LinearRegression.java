package us.ihmc.robotics.functionApproximation;

import java.util.ArrayList;

import Jama.Matrix;

/**
 * <p>Title:LinearRegression </p>
 *
 * <p>Description: A helper class to solve linear regression problems. The client provides a set of input vectors x_{1...p} and a
 * set of corresponding output values y_{1...p}, this calculates the weights to multiply the input vector x by to
 * approximate y. The number of data points provided, p, must be greater than n, the number of dimensions in x. This algorithm
 * makes all the assumptions needed when using the least squares (Gauss-Markov) method. Like all good things, this was found on
 * wikipedia. Note that to have an offset the client should augment the input vectors with a constant term.</p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: IHMC</p>
 *
 * @author jrebula
 * @version 1.0
 */
public class LinearRegression
{
   public static boolean VERBOSE = false;

   private Matrix inputMatrix;
   private Matrix outputVector;

   private boolean solved;
   private double error;
   private Matrix betaVector;

   /**
    * Creates a linear regression problem with the given data set.
    * @param inputData Matrix nxp matrix of input vectors
    * @param outputData Matrix 1xp vector of output values
    */
   public LinearRegression(Matrix inputData, Matrix outputData)
   {
      setMatrices(inputData, outputData);
   }

   /**
    * Creates a linear regression problem with the given data set.
    * @param inputs ArrayList<double[]> p sized list of double arrays, each of length n, representing the input vectors
    * @param outputs ArrayList<Double> p sized list of doubles representing the output values
    */
   public LinearRegression(ArrayList<double[]> inputs, ArrayList<Double> outputs)
   {
      double[][] inputArrays = new double[inputs.size()][inputs.get(0).length];
      for (int i = 0; i < inputArrays.length; i++)
      {
         inputArrays[i] = inputs.get(i);
      }

      Matrix inputMatrix = new Matrix(inputArrays);
      double[] outputArray = new double[outputs.size()];
      for (int i = 0; i < outputArray.length; i++)
      {
         outputArray[i] = outputs.get(i);
      }

      setMatrices(inputMatrix, new Matrix(outputArray, outputArray.length));
   }

   /**
    * Creates a linear regression problem with the given data set.
    * @param inputs double[][] the input matrix with dimensions [p][n].
    * @param outputs double[] the array of output values
    */
   public LinearRegression(double[][] inputs, double[] outputs)
   {
      setMatrices(new Matrix(inputs), new Matrix(outputs, outputs.length));
   }

   private void setMatrices(Matrix inputData, Matrix outputData)
   {
      this.inputMatrix = inputData;
      this.outputVector = outputData;
      solved = false;
   }


   // Solves and returns true if a solution was found. Otherwise returns false;
   public boolean solve()
   {
      if (solved)
      {
         return true;
      }

      // thank you jama...
      Matrix x = inputMatrix;
      Matrix xTrans = inputMatrix.transpose();
      Matrix xTransX = xTrans.times(x);

      if (VERBOSE)
      {
         System.out.println("LinearRegression::solve: X, Y : ");
         inputMatrix.print(5, 5);
         outputVector.print(5, 5);
         System.out.println("LinearRegression::solve: xTransX.det() : " + xTransX.det());
      }


      if (Math.abs(xTransX.det()) < 1e-86)    // 1e-12;
      {
         if (VERBOSE)
         {
            System.out.println("LinearRegression::solve: Determinate of xTransposeX " + xTransX.det() + ", is too small to invert safely");
         }

         return false;
      }


      Matrix xTransXInv = xTransX.inverse();
      Matrix xTransXInvXTrans = xTransXInv.times(xTrans);
      betaVector = xTransXInvXTrans.times(outputVector);
      Matrix residuals = outputVector.minus(x.times(betaVector));

      error = 0.0;

      for (int i = 0; i < residuals.getColumnDimension(); i++)
      {
         error += (residuals.get(0, i) * residuals.get(0, i));
      }

      solved = true;

      return true;
   }

   public double getSquaredError()
   {
      verifySolved();
      return error;
   }

   public Matrix getCoefficientVectorAsMatrix()
   {
      verifySolved();
      return betaVector;
   }

   public void getCoefficientVector(double[] coefficientVector)
   {
      verifySolved();
 
      if (coefficientVector.length != betaVector.getRowDimension())
      {
         throw new IllegalArgumentException("given array must have size " + betaVector.getRowDimension());
      }

      for (int i = 0; i < betaVector.getRowDimension(); i++)
      {
         coefficientVector[i] = betaVector.get(i, 0);
      }
   }
   
   private void verifySolved()
   {
      if (!solved)
      {
         throw new IllegalStateException("cannot get error before the Regression has been solved");
      }
   }

}
