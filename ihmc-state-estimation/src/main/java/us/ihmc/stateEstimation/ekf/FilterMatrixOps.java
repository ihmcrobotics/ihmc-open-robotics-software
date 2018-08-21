package us.ihmc.stateEstimation.ekf;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

public class FilterMatrixOps
{
   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final DenseMatrix64F indentityToInvert = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F BAtrans = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ABAtransPlusC = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F inverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F PHtrans = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F innovation = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F identity = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F IKH = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F KRKtrans = new DenseMatrix64F(0, 0);

   /**
    * Sets the provided matrix to a square identity matrix of the given size.
    *
    * @param matrix (modified)
    * @param size is the the desired number of rows and columns for the matrix
    */
   public void setIdentity(DenseMatrix64F matrix, int size)
   {
      matrix.reshape(size, size);
      CommonOps.setIdentity(matrix);
   }

   /**
    * Sets the provided matrix to</br>
    * result = A * B * A'</br>
    * Note, that B must be square.
    */
   public void computeABAtrans(DenseMatrix64F result, DenseMatrix64F A, DenseMatrix64F B)
   {
      BAtrans.reshape(B.getNumRows(), A.getNumRows());
      CommonOps.multTransB(B, A, BAtrans);

      result.reshape(A.getNumRows(), A.getNumRows());
      CommonOps.mult(A, BAtrans, result);
   }

   /**
    * Sets the provided matrix to</br>
    * result = A * B * A' + C
    */
   public void computeABAtransPlusC(DenseMatrix64F result, DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C)
   {
      computeABAtrans(result, A, B);
      CommonOps.add(result, C, result);
   }

   /**
    * Sets the provided matrix to</br>
    * result = inverse(A)</br>
    * Will return whether the inversion succeeded.
    *
    * @return whether the inversion succeeded
    */
   public boolean invertMatrix(DenseMatrix64F result, DenseMatrix64F A)
   {
      setIdentity(indentityToInvert, A.getNumRows());

      if (!solver.setA(A))
      {
         CommonOps.fill(result, 0.0);
         return false;
      }

      result.reshape(A.getNumRows(), A.getNumCols());
      solver.solve(indentityToInvert, result);
      return true;
   }

   /**
    * Sets the provided matrix to</br>
    * result = inverse(A * B * A' + C)</br>
    * Will return whether the inversion succeeded.
    *
    * @return whether the inversion succeeded
    */
   public boolean computeInverseOfABAtransPlusC(DenseMatrix64F result, DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C)
   {
      computeABAtransPlusC(ABAtransPlusC, A, B, C);
      return invertMatrix(result, ABAtransPlusC);
   }

   /**
    * Sets the provided matrix to</br>
    * result = F * Pposterior * F' + Q
    *
    * @param result (modified)
    * @param F is the matrix that describes the linearized state evolution: x(k+1) = F * x(k)
    * @param Pposterior is the previous error covariance
    * @param Q is the covariance matrix of the state evolution
    */
   public void predictErrorCovariance(DenseMatrix64F result, DenseMatrix64F F, DenseMatrix64F Pposterior, DenseMatrix64F Q)
   {
      computeABAtransPlusC(result, F, Pposterior, Q);
   }

   /**
    * Sets the provided matrix to</br>
    * result = P * H' * inverse(H * P * H' + R)</br>
    * Will return whether the inversion succeeded.
    *
    * @return whether the inversion succeeded
    * @param result (modified)
    * @param P is the error covariance
    * @param H is the measurement jacobian
    * @param R is the measurement covariance
    */
   public boolean computeKalmanGain(DenseMatrix64F result, DenseMatrix64F P, DenseMatrix64F H, DenseMatrix64F R)
   {
      if (!computeInverseOfABAtransPlusC(inverse, H, P, R))
      {
         CommonOps.fill(result, 0.0);
         return false;
      }

      PHtrans.reshape(P.getNumRows(), H.getNumRows());
      CommonOps.multTransB(P, H, PHtrans);

      result.reshape(P.getNumRows(), R.getNumCols());
      CommonOps.mult(PHtrans, inverse, result);
      return true;
   }

   /**
    * Sets the provided matrix to</br>
    * result = xPrior + K * (z - h(xPrior)) = xPrior + K * residual
    *
    * @param result (modified)
    * @param K is the kalman gain
    * @param residual is the measurement residual
    * @param xPrior is the state before the measurement update
    */
   public void updateState(DenseMatrix64F result, DenseMatrix64F K, DenseMatrix64F residual, DenseMatrix64F xPrior)
   {
      innovation.reshape(xPrior.getNumRows(), 1);
      CommonOps.mult(K, residual, innovation);
      result.reshape(xPrior.getNumRows(), 1);
      CommonOps.add(xPrior, innovation, result);
   }

   /**
    * Sets the provided matrix to</br>
    * result = (identity - K * H) * pPrior * (identity - K * H)' + K * R * K'
    *
    * @param result (modified)
    * @param K is the kalman gain
    * @param H is the measurement jacobian
    * @param R
    * @param pPrior is the error covariance before the update
    */
   public void updateErrorCovariance(DenseMatrix64F result, DenseMatrix64F K, DenseMatrix64F H, DenseMatrix64F R, DenseMatrix64F pPrior)
   {
      computeABAtrans(KRKtrans, K, R);
      IKH.reshape(pPrior.getNumRows(), pPrior.getNumRows());
      setIdentity(identity, pPrior.getNumRows());
      CommonOps.mult(K, H, IKH);
      CommonOps.subtract(identity, IKH, IKH);
      computeABAtransPlusC(result, IKH, pPrior, KRKtrans);
   }
}
