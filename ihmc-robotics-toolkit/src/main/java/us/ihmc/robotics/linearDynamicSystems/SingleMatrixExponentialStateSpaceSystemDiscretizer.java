package us.ihmc.robotics.linearDynamicSystems;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

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
   private final DMatrixRMaj discretizationMatrix;
   private final MatrixExponentialCalculator matrixExponentialCalculator;

   private final DMatrixRMaj F3;
   private final DMatrixRMaj G2;
   private final DMatrixRMaj G3;
   
   private final DMatrixRMaj negativeATranspose;

   private final int nStates;
   private final int nInputs;

   public SingleMatrixExponentialStateSpaceSystemDiscretizer(int numberOfStates, int numberOfInputs)
   {
      int discretizationMatrixSize = 2 * numberOfStates + numberOfInputs;
      this.discretizationMatrix = new DMatrixRMaj(discretizationMatrixSize, discretizationMatrixSize);
      this.matrixExponentialCalculator = new MatrixExponentialCalculator(discretizationMatrixSize);
      this.F3 = new DMatrixRMaj(numberOfStates, numberOfStates);
      this.G2 = new DMatrixRMaj(numberOfStates, numberOfStates);
      this.G3 = new DMatrixRMaj(numberOfStates, numberOfInputs);
      this.negativeATranspose = new DMatrixRMaj(numberOfStates, numberOfStates);
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
   public void discretize(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj Q, double dt)
   {
      MatrixTools.checkMatrixDimensions(A, nStates, nStates);
      MatrixTools.checkMatrixDimensions(B, nStates, nInputs);
      MatrixTools.checkMatrixDimensions(Q, nStates, nStates);
      
      // A
      CommonOps_DDRM.insert(A, discretizationMatrix, nStates, nStates);

      // -A^T
      CommonOps_DDRM.transpose(A, negativeATranspose);
      CommonOps_DDRM.changeSign(negativeATranspose);
      CommonOps_DDRM.insert(negativeATranspose, discretizationMatrix, 0, 0);

      // Q
      CommonOps_DDRM.insert(Q, discretizationMatrix, 0, nStates);

      // B
      CommonOps_DDRM.insert(B, discretizationMatrix, nStates, 2 * nStates);

      // exp(M * dt)
      CommonOps_DDRM.scale(dt, discretizationMatrix);
      matrixExponentialCalculator.compute(discretizationMatrix, discretizationMatrix);

      // blocks
      CommonOps_DDRM.extract(discretizationMatrix, nStates, 2 * nStates, nStates, 2 * nStates, F3, 0, 0);
      CommonOps_DDRM.extract(discretizationMatrix, 0, nStates, nStates, 2 * nStates, G2, 0, 0);
      CommonOps_DDRM.extract(discretizationMatrix, nStates, 2 * nStates, 2 * nStates, 2 * nStates + nInputs, G3, 0, 0);

      // A
      A.set(F3);

      // B
      CommonOps_DDRM.mult(F3, G3, B);

      // Q
      CommonOps_DDRM.multTransA(F3, G2, Q);

      // R stays R. Matlab kalmd says to scale it by 1/dt, but 
      // we think this is only true if you are averaging out the noise during the reading.
      // More realistically, you are not and just getting a noisy discrete reading.
//      CommonOps_DDRM.scale(1.0 / dt, R);
   }
}
