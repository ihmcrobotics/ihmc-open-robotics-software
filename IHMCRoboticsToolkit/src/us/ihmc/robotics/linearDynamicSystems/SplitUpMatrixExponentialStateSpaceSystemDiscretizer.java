package us.ihmc.robotics.linearDynamicSystems;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

/**
 * Discretizes a continuous time system using Matrix Exponentials.
 * Assumes a zero-order hold on the inputs during the time delta dt.
 * See "Computing Integrals Involving the Matrix Exponential" by Charles F. Van Loan 
 * for information on the algorithm. Also see Matlab Documentation for 
 * kalmd (Design discrete Kalman filter for continuous plant) and 
 * c2d (Continuous to Discrete). Also see the Wikipedia entry on Matrix Exponentials, and Discretization.
 * Turns x_dot = Ax + Bu + w; y = Cx + Du + v  with process noise covariance Q and sensor noise covariance R.
 * into x_n+1 = A x_n + Bu + w; y = Cx + Du + v with new process noise covariance Q and sensor noise covariance R.
 *
 */
public class SplitUpMatrixExponentialStateSpaceSystemDiscretizer implements StateSpaceSystemDiscretizer
{
   private final DenseMatrix64F discretizationMatrix1;
   private final MatrixExponentialCalculator matrixExponentialCalculator1;

   private final DenseMatrix64F discretizationMatrix2;
   private final MatrixExponentialCalculator matrixExponentialCalculator2;

   private final DenseMatrix64F F3;
   private final DenseMatrix64F G2;
   private final DenseMatrix64F G3;
   
   private final DenseMatrix64F negativeATranspose;

   private final int nStates;
   private final int nInputs;

   public SplitUpMatrixExponentialStateSpaceSystemDiscretizer(int numberOfStates, int numberOfInputs)
   {
      int discretizationMatrix1Size = numberOfStates + numberOfInputs;
      this.discretizationMatrix1 = new DenseMatrix64F(discretizationMatrix1Size, discretizationMatrix1Size);
      this.matrixExponentialCalculator1 = new MatrixExponentialCalculator(discretizationMatrix1Size);
      
      int discretizationMatrix2Size = 2 * numberOfStates;
      this.discretizationMatrix2 = new DenseMatrix64F(discretizationMatrix2Size, discretizationMatrix2Size);
      this.matrixExponentialCalculator2 = new MatrixExponentialCalculator(discretizationMatrix2Size);

      this.negativeATranspose = new DenseMatrix64F(numberOfStates, numberOfStates);
      
      this.F3 = new DenseMatrix64F(numberOfStates, numberOfStates);
      this.G2 = new DenseMatrix64F(numberOfStates, numberOfStates);
      this.G3 = new DenseMatrix64F(numberOfStates, numberOfInputs);
      this.nStates = numberOfStates;
      this.nInputs = numberOfInputs;
   }

   /**
    * discretizes a continuous time system. All input matrices are modified.
    * @param A state matrix
    * @param B input matrix
    * @param Q process noise covariance matrix
    * @param dt time step
    */
   public void discretize(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, double dt)
   {
      MatrixTools.checkMatrixDimensions(A, nStates, nStates);
      MatrixTools.checkMatrixDimensions(B, nStates, nInputs);
      MatrixTools.checkMatrixDimensions(Q, nStates, nStates);
      
      
      /*
       * matrix exponential of
       * [A B]
       * [0 0];
       */
      CommonOps.insert(A, discretizationMatrix1, 0, 0);
      CommonOps.insert(B, discretizationMatrix1, 0, nStates);      
      CommonOps.scale(dt, discretizationMatrix1);
      matrixExponentialCalculator1.compute(discretizationMatrix1, discretizationMatrix1);

      /*
       * matrix exponential of
       * [-A^T Q]
       * [0    A]
       */
      CommonOps.insert(A, discretizationMatrix2, nStates, nStates);
      CommonOps.transpose(A, negativeATranspose);
      CommonOps.changeSign(negativeATranspose);
      CommonOps.insert(negativeATranspose, discretizationMatrix2, 0, 0);
      CommonOps.insert(Q, discretizationMatrix2, 0, nStates);
      CommonOps.scale(dt, discretizationMatrix2);
      matrixExponentialCalculator2.compute(discretizationMatrix2, discretizationMatrix2);

      // blocks
      CommonOps.extract(discretizationMatrix1, 0, nStates, 0, nStates, F3, 0, 0);
      CommonOps.extract(discretizationMatrix2, 0, nStates, nStates, 2 * nStates, G2, 0, 0);
      CommonOps.extract(discretizationMatrix1, 0, nStates, nStates, nStates + nInputs, G3, 0, 0);

      // A
      A.set(F3);

      // B
      CommonOps.mult(F3, G3, B);

      // Q
      CommonOps.multTransA(F3, G2, Q);

      // R stays R. Matlab kalmd says to scale it by 1/dt, but 
      // we think this is only true if you are averaging out the noise during the reading.
      // More realistically, you are not and just getting a noisy discrete reading.
//      CommonOps.scale(1.0 / dt, R);
   }
}
