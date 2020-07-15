package us.ihmc.robotics.optimization;

import java.util.function.Function;
import java.util.function.UnaryOperator;

import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * this class is to iterate minimizing input parameters for the given output calculator matrix. in
 * general, the input parameters are related six-dimensional in spatial space to transform rigid
 * body. for example, we may let the rigid body with a camera frame and minimize the distance
 * between two frames.
 * <p>
 * the way to use this is simple like defining input function {@link Function<DMatrixRMaj,
 * RigidBodyTransform>} and output calculator {@link UnaryOperator<DMatrixRMaj>}. and do
 * {@code initialize()} and {@code iterate()}
 * <p>
 * the method {@code iterate()} updates the input space with the following equation.
 * <p>
 * {@linkplain -inv(J^T * J + W_N) * J^T * e }
 * <p>
 * see the following examples and slides.
 * <p>
 * {@link LevenbergMarquardtICPTest}, {@link LevenbergMarquardtICPVisualizer}.
 * <p>
 * @see <a href="https://docs.google.com/presentation/d/1GH2QqMLM1cKgll5hcd-nX18c-cGVjB5ABkz_voOZU1w/edit?usp=sharing">20200722_LearningLunch_Inho</a>
 */
public class LevenbergMarquardtParameterOptimizer
{
   private static final boolean DEBUG = false;
   private int inputDimension;
   private int outputDimension;

   private final Function<DMatrixRMaj, RigidBodyTransform> inputFunction;
   private final UnaryOperator<DMatrixRMaj> outputCalculator;

   private final DMatrixRMaj currentInput;
   private final OuputSpace currentOutputSpace;
   private final DMatrixRMaj purterbationVector;
   private final DMatrixRMaj perturbedInput;
   private final DMatrixRMaj perturbedOutput;
   private final DMatrixRMaj jacobian;

   private final DMatrixRMaj jacobianTranspose;
   private final DMatrixRMaj squaredJacobian;
   private final DMatrixRMaj dampingMatrix;
   private final DMatrixRMaj invMultJacobianTranspose;

   private final DMatrixRMaj optimizeDirection;

   /**
    * higher : make quick approach when the model and the data are in long distance.
    * <p>
    * lower : slower approach but better accuracy in near distance.
    */
   private double correspondenceThreshold = 1.0;
   private static final double DEFAULT_PERTURBATION = 0.0001;
   private static final double DEFAULT_DAMPING_COEFFICIENT = 0.001;
   private boolean useDamping = true;

   private double computationTime;
   private int iteration;
   private boolean optimized;

   /**
    * iterate the direction to minimize output space, -inv(J^T * J + W_N) * J^T * e
    */
   public LevenbergMarquardtParameterOptimizer(Function<DMatrixRMaj, RigidBodyTransform> inputFunction, UnaryOperator<DMatrixRMaj> outputCalculator,
                                               int inputParameterDimension, int outputDimension)
   {
      this.inputFunction = inputFunction;
      this.inputDimension = inputParameterDimension;
      this.outputDimension = outputDimension;
      this.outputCalculator = outputCalculator;

      currentInput = new DMatrixRMaj(inputParameterDimension, 1);
      currentOutputSpace = new OuputSpace(outputDimension);

      purterbationVector = new DMatrixRMaj(inputParameterDimension, 1);
      for (int i = 0; i < inputParameterDimension; i++)
         purterbationVector.set(i, DEFAULT_PERTURBATION);
      perturbedInput = new DMatrixRMaj(inputParameterDimension, 1);
      perturbedOutput = new DMatrixRMaj(outputDimension, 1);
      jacobian = new DMatrixRMaj(outputDimension, inputParameterDimension);

      jacobianTranspose = new DMatrixRMaj(outputDimension, inputParameterDimension);
      squaredJacobian = new DMatrixRMaj(inputParameterDimension, inputParameterDimension);
      dampingMatrix = new DMatrixRMaj(inputParameterDimension, inputParameterDimension);
      invMultJacobianTranspose = new DMatrixRMaj(inputParameterDimension, outputDimension);

      optimizeDirection = new DMatrixRMaj(inputParameterDimension, 1);
   }

   public void setPerturbationVector(DMatrixRMaj purterbationVector)
   {
      if (this.purterbationVector.getNumCols() != purterbationVector.getNumCols())
         throw new MatrixDimensionException("dimension is wrong. " + this.purterbationVector.getNumCols() + " " + purterbationVector.getNumCols());
      this.purterbationVector.set(purterbationVector);
   }

   public void setCorrespondenceThreshold(double correspondenceThreshold)
   {
      this.correspondenceThreshold = correspondenceThreshold;
   }

   public boolean initialize()
   {
      iteration = 0;
      optimized = false;
      for (int i = 0; i < inputDimension; i++)
      {
         for (int j = 0; j < inputDimension; j++)
         {
            if (i == j)
            {
               dampingMatrix.set(i, j, DEFAULT_DAMPING_COEFFICIENT);
            }
         }
      }
      currentOutputSpace.updateOutputSpace(outputCalculator.apply(currentInput));

      return currentOutputSpace.computeCorrespondence();
   }

   public double iterate()
   {
      iteration++;
      long startTime = System.nanoTime();

      DMatrixRMaj newInput = new DMatrixRMaj(inputDimension, 1);

      currentOutputSpace.updateOutputSpace(outputCalculator.apply(currentInput));
      if (!currentOutputSpace.computeCorrespondence())
      {
         return -1;
      }
      currentOutputSpace.computeQuality();

      // start.      
      // compute jacobian.
      for (int i = 0; i < inputDimension; i++)
      {
         perturbedInput.set(currentInput);
         perturbedInput.add(i, 0, purterbationVector.get(i));

         perturbedOutput.set(outputCalculator.apply(perturbedInput));
         for (int j = 0; j < outputDimension; j++)
         {
            if (currentOutputSpace.isCorresponding(j))
            {
               double partialValue = (perturbedOutput.get(j) - currentOutputSpace.getOutput().get(j)) / purterbationVector.get(i);
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
      if (useDamping)
      {
         CommonOps_DDRM.add(squaredJacobian, dampingMatrix, squaredJacobian);
      }
      CommonOps_DDRM.invert(squaredJacobian);

      CommonOps_DDRM.mult(squaredJacobian, jacobianTranspose, invMultJacobianTranspose);
      CommonOps_DDRM.mult(invMultJacobianTranspose, currentOutputSpace.getOutput(), optimizeDirection);

      // update currentInput.
      CommonOps_DDRM.subtract(currentInput, optimizeDirection, newInput);

      // compute new quality.
      currentInput.set(newInput);

      double iterateTime = Conversions.nanosecondsToSeconds(System.nanoTime() - startTime);
      if (DEBUG)
      {
         System.out.println("elapsed iteration time is " + iterateTime);
      }

      return currentOutputSpace.getCorrespondingQuality();
   }

   public void convertInputToTransform(DMatrixRMaj input, RigidBodyTransform transformToPack)
   {
      if (input.getData().length != inputDimension)
         throw new MatrixDimensionException("dimension is wrong. " + input.getData().length + " " + inputDimension);
      transformToPack.set(inputFunction.apply(input));
   }

   public int getNumberOfCorespondingPoints()
   {
      return currentOutputSpace.getNumberOfCorrespondingPoints();
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
      return currentOutputSpace.getCorrespondingQuality();
   }

   public double getPureQuality()
   {
      return currentOutputSpace.getQuality();
   }

   public int getIteration()
   {
      return iteration;
   }

   public double getComputationTime()
   {
      return computationTime;
   }

   public static Function<DMatrixRMaj, RigidBodyTransform> createSpatialInputFunction()
   {
      return new Function<DMatrixRMaj, RigidBodyTransform>()
      {
         @Override
         public RigidBodyTransform apply(DMatrixRMaj input)
         {
            RigidBodyTransform transform = new RigidBodyTransform();

            transform.setRotationYawPitchRollAndZeroTranslation(input.get(5), input.get(4), input.get(3));
            transform.getTranslation().set(input.get(0), input.get(1), input.get(2));
            return transform;
         }
      };
   }

   private class OuputSpace
   {
      private final DMatrixRMaj output;
      private final boolean[] correspondence;
      private int numberOfCorrespondingPoints;
      private double correspondingQuality;
      private double quality;

      private OuputSpace(int dimension)
      {
         output = new DMatrixRMaj(dimension, 1);
         correspondence = new boolean[dimension];
      }

      /**
       * update output.
       */
      void updateOutputSpace(DMatrixRMaj output)
      {
         this.output.set(output);
      }

      /**
       * compute correspondence and update them. return false if there is no correspondence point.
       */
      boolean computeCorrespondence()
      {
         numberOfCorrespondingPoints = 0;
         for (int i = 0; i < output.getNumRows(); i++)
         {
            if (output.get(i, 0) < correspondenceThreshold)
            {
               correspondence[i] = true;
               numberOfCorrespondingPoints++;
            }
            else
            {
               correspondence[i] = false;
            }
         }
         if (numberOfCorrespondingPoints == 0)
         {
            return false;
         }
         return true;
      }

      /**
       * there are two kinds of quality. one is distance between corresponding points
       * (correspondingQuality). the other is distance between the closest points (quality).
       */
      void computeQuality()
      {
         correspondingQuality = 0.0;
         quality = 0.0;
         for (int i = 0; i < output.getNumRows(); i++)
         {
            double norm = output.get(i, 0) * output.get(i, 0);
            quality = quality + norm;
            if (correspondence[i])
            {
               correspondingQuality = correspondingQuality + norm;
            }
         }
      }

      DMatrixRMaj getOutput()
      {
         return output;
      }

      boolean isCorresponding(int index)
      {
         return correspondence[index];
      }

      int getNumberOfCorrespondingPoints()
      {
         return numberOfCorrespondingPoints;
      }

      double getCorrespondingQuality()
      {
         return correspondingQuality;
      }

      double getQuality()
      {
         return quality;
      }
   }
}
