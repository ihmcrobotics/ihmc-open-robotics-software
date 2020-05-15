package us.ihmc.robotics.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixDimensionException;

import us.ihmc.commons.Conversions;

public class LevenbergMarquardtParameterOptimizer
{
   private boolean DEBUG = true;
   private int parameterDimension;
   private int outputDimension;

   private FunctionOutputCalculator outputCalculator = null;
   private final DenseMatrix64F dampingCoefficient;

   private final DenseMatrix64F currentInput;
   private final DenseMatrix64F currentOutput;
   private final DenseMatrix64F purterbationVector;
   private final DenseMatrix64F perturbedInput;
   private final DenseMatrix64F perturbedOutput;
   private final DenseMatrix64F jacobian;
   //   private final DenseMatrix64F jacobianTranspose;
   //   private final DenseMatrix64F squaredJacobian;
   private final DenseMatrix64F optimizeDirection;

   private boolean[] correspondence;
   private double correspondenceThreshold = 0.3; // TODO:

   private double computationTime;
   private double quality;
   private int iteration;
   private boolean optimized;

   /**
    * iterate the direction to minimize output space, -inv(J^T * J + W_N) * J^T * e
    */
   public LevenbergMarquardtParameterOptimizer(int inputParameterDimension, int outputDimension)
   {
      if (DEBUG)
         System.out.println("Optimizer Info = " + inputParameterDimension + " " + outputDimension + " space solver");
      this.parameterDimension = inputParameterDimension;
      this.outputDimension = outputDimension;

      dampingCoefficient = new DenseMatrix64F(inputParameterDimension, inputParameterDimension);

      currentInput = new DenseMatrix64F(inputParameterDimension, 1);
      currentOutput = new DenseMatrix64F(outputDimension, 1);
      purterbationVector = new DenseMatrix64F(inputParameterDimension, 1);
      perturbedInput = new DenseMatrix64F(inputParameterDimension, 1);
      perturbedOutput = new DenseMatrix64F(outputDimension, 1);
      jacobian = new DenseMatrix64F(outputDimension, inputParameterDimension);

      optimizeDirection = new DenseMatrix64F(inputParameterDimension, 1);

      correspondence = new boolean[outputDimension];
   }

   public void reShape(int parameterDimension, int outputDimension)
   {
      this.parameterDimension = parameterDimension;
      this.outputDimension = outputDimension;
   }

   // TODO: set bound. especially, orientation.

   public void setPerturbationVector(DenseMatrix64F purterbationVector)
   {
      if (this.purterbationVector.getNumCols() != purterbationVector.getNumCols())
         throw new MatrixDimensionException("do reShape first.");
      this.purterbationVector.set(purterbationVector);
   }

   public void setOutputCalculator(FunctionOutputCalculator functionOutputCalculator)
   {
      outputCalculator = functionOutputCalculator;
   }

   private double computeQuality(DenseMatrix64F space, boolean[] correspondence)
   {
      double norm = 0.0;
      for (int i = 0; i < space.getNumRows(); i++)
      {
         if (correspondence[i])
         {
            norm = norm + space.get(i, 0) * space.get(i, 0);
         }
      }
      return norm;
   }

   public boolean solve(int terminalIteration, double terminalConvergencePercentage)
   {
      long startTime = System.nanoTime();
      iteration = 0;
      optimized = false;

      DenseMatrix64F newInput = new DenseMatrix64F(parameterDimension, 1);

      double currentQuality = 0.0;
      double qualityDiff = 0.0;

      // compute correspondence space
      currentOutput.set(outputCalculator.computeOutput(currentInput));
      for (int i = 0; i < outputDimension; i++)
      {
         if (currentOutput.get(i, 0) < correspondenceThreshold)
         {
            correspondence[i] = true;
         }
         else
         {
            correspondence[i] = false;
         }
      }
      quality = computeQuality(currentOutput, correspondence);
      if (DEBUG)
      {
         System.out.println("Initial Quality = " + quality);
      }
      // start
      for (int iter = 0; iter < terminalIteration; iter++)
      {
         iteration = iter;
         currentQuality = quality;

         // compute jacobian
         for (int i = 0; i < parameterDimension; i++)
         {
            perturbedInput.set(currentInput);
            perturbedInput.add(i, 0, purterbationVector.get(i));

            perturbedOutput.set(outputCalculator.computeOutput(perturbedInput));
            for (int j = 0; j < outputDimension; j++)
            {
               if (correspondence[j])
               {
                  double partialValue = (perturbedOutput.get(j) - currentOutput.get(j)) / purterbationVector.get(i);
                  jacobian.set(j, i, partialValue);
               }
               else
               {
                  jacobian.set(j, i, 0.0);
               }
            }
         }

         // compute direction.
         DenseMatrix64F jacobianTranspose = new DenseMatrix64F(outputDimension, parameterDimension);
         jacobianTranspose.set(jacobian);
         CommonOps.transpose(jacobianTranspose);

         DenseMatrix64F squaredJacobian = new DenseMatrix64F(parameterDimension, parameterDimension);
         CommonOps.mult(jacobianTranspose, jacobian, squaredJacobian);
         CommonOps.invert(squaredJacobian);

         DenseMatrix64F invMultJacobianTranspose = new DenseMatrix64F(parameterDimension, outputDimension);
         CommonOps.mult(squaredJacobian, jacobianTranspose, invMultJacobianTranspose);
         CommonOps.mult(invMultJacobianTranspose, currentOutput, optimizeDirection);

         // update currentInput.
         CommonOps.subtract(currentInput, optimizeDirection, newInput);

         // compute new quality.
         currentInput.set(newInput);
         currentOutput.set(outputCalculator.computeOutput(currentInput));
         for (int i = 0; i < outputDimension; i++)
         {
            if (currentOutput.get(i, 0) < correspondenceThreshold)
            {
               correspondence[i] = true;
            }
            else
            {
               correspondence[i] = false;
            }
         }
         quality = computeQuality(currentOutput, correspondence);
         currentQuality = computeQuality(currentOutput, correspondence);
         if (DEBUG)
         {
            System.out.println("# iter [" + iter + "] quality = " + quality);
         }

         quality = currentQuality;
         //         // compute terminal condition.
         //         qualityDiff = currentQuality - quality;
         //         if (qualityDiff / quality * 100 < terminalConvergencePercentage) // abs(qualityDiff)?
         //         {
         //            optimized = true;
         //            quality = currentQuality;
         //            break;
         //         }
         //         else
         //         {
         //            quality = currentQuality;            
         //         }
      }

      computationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - startTime);
      return optimized;
   }

   public DenseMatrix64F getOptimalParameter()
   {
      return currentInput;
   }

   public boolean isSolved()
   {
      return optimized;
   }

   public double getQuality()
   {
      return 0.0;
   }

   public int getIteration()
   {
      return 0;
   }

   public double getComputationTime()
   {
      return computationTime;
   }
}
