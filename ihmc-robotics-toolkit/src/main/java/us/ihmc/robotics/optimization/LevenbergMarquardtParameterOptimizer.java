package us.ihmc.robotics.optimization;

import java.util.function.UnaryOperator;

import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.Conversions;

public class LevenbergMarquardtParameterOptimizer
{
   private static final boolean DEBUG = false;
   private int parameterDimension;
   private int outputDimension;

   private final UnaryOperator<DMatrixRMaj> outputCalculator;
   private boolean useDampingCoefficient = false;
   private final DMatrixRMaj dampingCoefficient;
   private static final double DEFAULT_RESIDUAL_SCALER = 0.1;
   private double residualScaler = DEFAULT_RESIDUAL_SCALER;

   private final DMatrixRMaj currentInput;
   private final DMatrixRMaj currentOutput;
   private final DMatrixRMaj purterbationVector;
   private final DMatrixRMaj perturbedInput;
   private final DMatrixRMaj perturbedOutput;
   private final DMatrixRMaj jacobian;

   private final DMatrixRMaj jacobianTranspose;
   private final DMatrixRMaj squaredJacobian;
   private final DMatrixRMaj invMultJacobianTranspose;

   private final DMatrixRMaj optimizeDirection;

   // higher : make quick approach when the model and the data are in long distance.
   // lower   : slower approach but better accuracy in near distance.
   private boolean[] correspondence;
   private double correspondenceThreshold = 1.0;
   private int numberOfCorespondingPoints = 0;

   private double computationTime;
   private double initialQuality;
   private double quality;
   private int iteration;
   private boolean optimized;

   /**
    * iterate the direction to minimize output space, -inv(J^T * J + W_N) * J^T * e
    */
   public LevenbergMarquardtParameterOptimizer(int inputParameterDimension, int outputDimension, UnaryOperator<DMatrixRMaj> outputCalculator)
   {
      if (DEBUG)
         System.out.println("Optimizer Info = " + inputParameterDimension + " " + outputDimension + " space solver");
      this.parameterDimension = inputParameterDimension;
      this.outputDimension = outputDimension;
      this.outputCalculator = outputCalculator;

      dampingCoefficient = new DMatrixRMaj(inputParameterDimension, inputParameterDimension);

      currentInput = new DMatrixRMaj(inputParameterDimension, 1);
      currentOutput = new DMatrixRMaj(outputDimension, 1);
      purterbationVector = new DMatrixRMaj(inputParameterDimension, 1);
      perturbedInput = new DMatrixRMaj(inputParameterDimension, 1);
      perturbedOutput = new DMatrixRMaj(outputDimension, 1);
      jacobian = new DMatrixRMaj(outputDimension, inputParameterDimension);

      jacobianTranspose = new DMatrixRMaj(outputDimension, inputParameterDimension);
      squaredJacobian = new DMatrixRMaj(inputParameterDimension, inputParameterDimension);
      invMultJacobianTranspose = new DMatrixRMaj(inputParameterDimension, outputDimension);

      optimizeDirection = new DMatrixRMaj(inputParameterDimension, 1);

      correspondence = new boolean[outputDimension];
   }

   public void setPerturbationVector(DMatrixRMaj purterbationVector)
   {
      if (this.purterbationVector.getNumCols() != purterbationVector.getNumCols())
         throw new MatrixDimensionException("do reShape first.");
      this.purterbationVector.set(purterbationVector);
   }

   public void setCorrespondenceThreshold(double correspondenceThreshold)
   {
      this.correspondenceThreshold = correspondenceThreshold;
   }

   private double computeQuality(DMatrixRMaj space, boolean[] correspondence)
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

   private double computePureQuality(DMatrixRMaj space)
   {
      double norm = 0.0;
      for (int i = 0; i < space.getNumRows(); i++)
      {
         norm = norm + space.get(i, 0) * space.get(i, 0);
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
      iteration++;
      long startTime = System.nanoTime();

      DMatrixRMaj newInput = new DMatrixRMaj(parameterDimension, 1);

      // compute correspondence space.
      numberOfCorespondingPoints = 0;
      currentOutput.set(outputCalculator.apply(currentInput));
      for (int i = 0; i < outputDimension; i++)
      {
         if (currentOutput.get(i, 0) < correspondenceThreshold)
         {
            correspondence[i] = true;
            numberOfCorespondingPoints++;
         }
         else
         {
            correspondence[i] = false;
         }
      }

      if (numberOfCorespondingPoints == 0)
      {
         return -1;
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

         perturbedOutput.set(outputCalculator.apply(perturbedInput));
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
      CommonOps_DDRM.transpose(jacobianTranspose);

      CommonOps_DDRM.mult(jacobianTranspose, jacobian, squaredJacobian);

      if (useDampingCoefficient)
      {
         updateDamping();
         CommonOps_DDRM.add(squaredJacobian, dampingCoefficient, squaredJacobian);
      }
      CommonOps_DDRM.invert(squaredJacobian);

      CommonOps_DDRM.mult(squaredJacobian, jacobianTranspose, invMultJacobianTranspose);
      CommonOps_DDRM.mult(invMultJacobianTranspose, currentOutput, optimizeDirection);

      // update currentInput.
      CommonOps_DDRM.subtract(currentInput, optimizeDirection, newInput);

      // compute new quality.
      currentInput.set(newInput);
      currentOutput.set(outputCalculator.apply(currentInput));
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
         System.out.println("elapsed iteration time is " + iterateTime + " " + numberOfCorespondingPoints);
      }
      return quality;
   }

   public int getNumberOfCorespondingPoints()
   {
      return numberOfCorespondingPoints;
   }

   public DMatrixRMaj getOptimalParameter()
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

   public double getPureQuality()
   {
      return computePureQuality(currentOutput);
   }

   public double getDampingCoefficient()
   {
      return residualScaler;
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
