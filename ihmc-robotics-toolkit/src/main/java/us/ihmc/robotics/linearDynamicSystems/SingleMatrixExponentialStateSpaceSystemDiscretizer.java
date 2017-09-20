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
public class SingleMatrixExponentialStateSpaceSystemDiscretizer implements StateSpaceSystemDiscretizer
{
   private final DenseMatrix64F discretizationMatrix;
   private final MatrixExponentialCalculator matrixExponentialCalculator;

   private final DenseMatrix64F F3;
   private final DenseMatrix64F G2;
   private final DenseMatrix64F G3;
   
   private final DenseMatrix64F negativeATranspose;

   private final int nStates;
   private final int nInputs;

   public SingleMatrixExponentialStateSpaceSystemDiscretizer(int numberOfStates, int numberOfInputs)
   {
      int discretizationMatrixSize = 2 * numberOfStates + numberOfInputs;
      this.discretizationMatrix = new DenseMatrix64F(discretizationMatrixSize, discretizationMatrixSize);
      this.matrixExponentialCalculator = new MatrixExponentialCalculator(discretizationMatrixSize);
      this.F3 = new DenseMatrix64F(numberOfStates, numberOfStates);
      this.G2 = new DenseMatrix64F(numberOfStates, numberOfStates);
      this.G3 = new DenseMatrix64F(numberOfStates, numberOfInputs);
      this.negativeATranspose = new DenseMatrix64F(numberOfStates, numberOfStates);
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
      
      // A
      CommonOps.insert(A, discretizationMatrix, nStates, nStates);

      // -A^T
      CommonOps.transpose(A, negativeATranspose);
      CommonOps.changeSign(negativeATranspose);
      CommonOps.insert(negativeATranspose, discretizationMatrix, 0, 0);

      // Q
      CommonOps.insert(Q, discretizationMatrix, 0, nStates);

      // B
      CommonOps.insert(B, discretizationMatrix, nStates, 2 * nStates);

      // exp(M * dt)
      CommonOps.scale(dt, discretizationMatrix);
      matrixExponentialCalculator.compute(discretizationMatrix, discretizationMatrix);

      // blocks
      CommonOps.extract(discretizationMatrix, nStates, 2 * nStates, nStates, 2 * nStates, F3, 0, 0);
      CommonOps.extract(discretizationMatrix, 0, nStates, nStates, 2 * nStates, G2, 0, 0);
      CommonOps.extract(discretizationMatrix, nStates, 2 * nStates, 2 * nStates, 2 * nStates + nInputs, G3, 0, 0);

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
