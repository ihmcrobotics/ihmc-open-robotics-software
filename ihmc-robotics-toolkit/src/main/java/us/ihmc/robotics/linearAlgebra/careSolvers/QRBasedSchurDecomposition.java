package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.apache.commons.math3.analysis.interpolation.UnivariatePeriodicInterpolator;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.DecompositionInterface;
import org.ejml.interfaces.decomposition.QRDecomposition;
import org.ejml.ops.CommonOps;

/**
 * This algorithm decomposes the A matrix into its Schur components, formed as
 * A = UTU'
 * It is based on the QR algorithm, as described in
 * http://people.inf.ethz.ch/arbenz/ewp/Lnotes/chapter4.pdf
 */
public class QRBasedSchurDecomposition implements SchurDecomposition<DenseMatrix64F>
{
   private final QRDecomposition<DenseMatrix64F> qrDecomposition;
   private final DenseMatrix64F T;
   private final DenseMatrix64F Tprev;
   private final DenseMatrix64F Uprev;

   private final DenseMatrix64F U;
   private final DenseMatrix64F Q;
   private final DenseMatrix64F R;

   private int maxIterations = Integer.MAX_VALUE;
   private double epsilon = 1e-10;

   public QRBasedSchurDecomposition(int size)
   {
      qrDecomposition = DecompositionFactory.qr(size, size);
      T = new DenseMatrix64F(size, size);
      Tprev = new DenseMatrix64F(size, size);
      Uprev = new DenseMatrix64F(size, size);
      Q = new DenseMatrix64F(size, size);
      R = new DenseMatrix64F(size, size);
      U = new DenseMatrix64F(size, size);
   }

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public void setConvergenceEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public boolean decompose(DenseMatrix64F A)
   {
      if (A.getNumRows() != A.getNumCols())
         throw new IllegalArgumentException("A matrix is not square.");

      boolean converged = false;
      int iterations = 0;

      R.reshape(A.getNumRows(), A.getNumCols());
      Q.reshape(A.getNumRows(), A.getNumCols());
      T.reshape(A.getNumRows(), A.getNumCols());
      U.reshape(A.getNumRows(), A.getNumCols());
      Uprev.reshape(A.getNumRows(), A.getNumCols());

      Tprev.set(A);
      CommonOps.setIdentity(Uprev);
      while (!converged)
      {
         if (iterations > maxIterations)
            return false;

         qrDecomposition.decompose(Tprev);
         qrDecomposition.getQ(Q, false);
         qrDecomposition.getR(R, false);

         CommonOps.mult(R, Q, T);

         CommonOps.mult(Uprev, Q, U);

         double TDistance = MatrixToolsLocal.distance(T, Tprev);
         double UDistance = MatrixToolsLocal.distance(U, Uprev);
         converged = TDistance < epsilon;
         converged &= UDistance < epsilon;

         Tprev.set(T);
         Uprev.set(U);

         iterations++;
      }

      return true;
   }

   public DenseMatrix64F getU(DenseMatrix64F UToPack)
   {
      if (UToPack != null)
      {
         UToPack.set(U);
         return null;
      }
      else
      {
         return new DenseMatrix64F(U);
      }
   }

   public DenseMatrix64F getT(DenseMatrix64F TToPack)
   {
      if (TToPack != null)
      {
         TToPack.set(T);
         return null;
      }
      else
      {
         return new DenseMatrix64F(T);
      }
   }

   @Override
   public boolean inputModified()
   {
      return false;
   }
}
