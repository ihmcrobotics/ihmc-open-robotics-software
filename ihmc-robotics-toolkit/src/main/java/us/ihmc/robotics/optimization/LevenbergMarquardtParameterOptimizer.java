package us.ihmc.robotics.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixDimensionException;

import us.ihmc.commons.Conversions;

public class LevenbergMarquardtParameterOptimizer
{
   private static final boolean DEBUG = false;
   private int parameterDimension;
   private int outputDimension;

   private static final boolean ENABLE_EARLY_TERMINAL = false;
   private FunctionOutputCalculator outputCalculator = null;
   private final DenseMatrix64F dampingCoefficient;
   private static final double DEFAULT_RESIDUAL_SCALER = 0.1;
   private double residualScaler = DEFAULT_RESIDUAL_SCALER;

   private final DenseMatrix64F currentInput;
   private final DenseMatrix64F currentOutput;
   private final DenseMatrix64F purterbationVector;
   private final DenseMatrix64F perturbedInput;
   private final DenseMatrix64F perturbedOutput;
   private final DenseMatrix64F jacobian;

   private final DenseMatrix64F jacobianTranspose;
   private final DenseMatrix64F squaredJacobian;
   private final DenseMatrix64F invMultJacobianTranspose;

   private final DenseMatrix64F optimizeDirection;

   // higher : make quick approach when the model and the data are in long distance.
   // lower   : slower approach but better accuracy in near distance.
   private boolean[] correspondence;
   private double correspondenceThreshold = 1.0;
   private int numberOfCoorespondingPoints = 0;

   private double computationTime;
   private double initialQuality;
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

      jacobianTranspose = new DenseMatrix64F(outputDimension, inputParameterDimension);
      squaredJacobian = new DenseMatrix64F(inputParameterDimension, inputParameterDimension);
      invMultJacobianTranspose = new DenseMatrix64F(inputParameterDimension, outputDimension);

      optimizeDirection = new DenseMatrix64F(inputParameterDimension, 1);

      correspondence = new boolean[outputDimension];
   }

   public void reShape(int parameterDimension, int outputDimension)
   {
      this.parameterDimension = parameterDimension;
      this.outputDimension = outputDimension;

      dampingCoefficient.reshape(parameterDimension, parameterDimension);

      currentInput.reshape(parameterDimension, 1);
      currentOutput.reshape(outputDimension, 1);
      purterbationVector.reshape(parameterDimension, 1);
      perturbedInput.reshape(parameterDimension, 1);
      perturbedOutput.reshape(outputDimension, 1);
      jacobian.reshape(outputDimension, parameterDimension);

      jacobianTranspose.reshape(outputDimension, parameterDimension);
      squaredJacobian.reshape(parameterDimension, parameterDimension);
      invMultJacobianTranspose.reshape(parameterDimension, outputDimension);

      optimizeDirection.reshape(parameterDimension, 1);

      correspondence = new boolean[outputDimension];
   }

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

   public void setCorrespondenceThreshold(double correspondenceThreshold)
   {
      this.correspondenceThreshold = correspondenceThreshold;
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

   private void updateDamping()
   {
      if (quality < initialQuality)
         residualScaler = quality / initialQuality * DEFAULT_RESIDUAL_SCALER;

      for (int i = 0; i < parameterDimension; i++)
      {
         for (int j = 0; j < parameterDimension; j++)
         {
            if (i == j)
            {
               dampingCoefficient.set(i, j, residualScaler);
            }
         }
      }
   }

   public void initialize()
   {
      iteration = 0;
      optimized = false;
      residualScaler = DEFAULT_RESIDUAL_SCALER;
      for (int i = 0; i < parameterDimension; i++)
      {
         for (int j = 0; j < parameterDimension; j++)
         {
            if (i == j)
            {
               dampingCoefficient.set(i, j, 1.0);
            }
            else
            {
               dampingCoefficient.set(i, j, 0.0);
            }
         }
      }
   }

   public double iterate()
   {
      long startTime = System.nanoTime();

      DenseMatrix64F newInput = new DenseMatrix64F(parameterDimension, 1);

      // compute correspondence space.
      numberOfCoorespondingPoints = 0;
      currentOutput.set(outputCalculator.computeOutput(currentInput));
      for (int i = 0; i < outputDimension; i++)
      {
         if (currentOutput.get(i, 0) < correspondenceThreshold)
         {
            correspondence[i] = true;
            numberOfCoorespondingPoints++;
         }
         else
         {
            correspondence[i] = false;
         }
      }
      quality = computeQuality(currentOutput, correspondence);
      initialQuality = quality;
      if (DEBUG)
      {
         System.out.println("Initial Quality = " + quality);
      }

      // start.      
      // compute jacobian.
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
      jacobianTranspose.set(jacobian);
      CommonOps.transpose(jacobianTranspose);

      CommonOps.mult(jacobianTranspose, jacobian, squaredJacobian);

      updateDamping();

      CommonOps.add(squaredJacobian, dampingCoefficient, squaredJacobian);
      CommonOps.invert(squaredJacobian);

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

      double iterateTime = Conversions.nanosecondsToSeconds(System.nanoTime() - startTime);
      if (DEBUG)
      {
         System.out.println("elapsed iteration time is " + iterateTime + " " + numberOfCoorespondingPoints);
         //         System.out.println("optimizeDirection");
         //         System.out.println(optimizeDirection);
         //         System.out.println("currentInput");
         //         System.out.println(currentInput);
      }
      return quality;
   }

   public boolean solve(int terminalIteration, double terminalConvergencePercentage)
   {
      long startTime = System.nanoTime();
      initialize();

      DenseMatrix64F newInput = new DenseMatrix64F(parameterDimension, 1);

      double previousQuality = 0.0;

      // compute correspondence space.
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
      initialQuality = quality;
      if (DEBUG)
      {
         System.out.println("Initial Quality = " + quality);
      }

      // start.      
      for (int iter = 0; iter < terminalIteration; iter++)
      {
         iteration = iter;
         previousQuality = quality;

         iterate();

         quality = iterate();
         double qualityDiff = previousQuality - quality;
         double regressionPercentage = qualityDiff / previousQuality * 100;
         if (DEBUG)
         {
            System.out.print("# iter [" + iter + "] quality = " + quality);
            System.out.print(", regression = " + regressionPercentage + " [%]");
            System.out.println(", residualScaler = " + residualScaler);
         }

         if (ENABLE_EARLY_TERMINAL)
         {
            if (iter > 5 && qualityDiff < 0)
            {
               optimized = false;

               // revert updating parameter.
               quality = previousQuality;
               CommonOps.add(currentInput, optimizeDirection, currentInput);
               break;
            }
            if (Math.abs(regressionPercentage) < terminalConvergencePercentage && regressionPercentage > 0)
            {
               // terminal.
               optimized = true;
               break;
            }
         }
      }

      computationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - startTime);
      return optimized;
   }

   public int getNumberOfCoorespondingPoints()
   {
      return numberOfCoorespondingPoints;
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
      return quality;
   }

   public int getIteration()
   {
      return iteration;
   }

   public double getComputationTime()
   {
      return computationTime;
   }
}
