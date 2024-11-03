package us.ihmc.robotics.linearAlgebra.careSolvers.signFunction;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixToolsLocal;

public class NewtonSignFunction implements SignFunction
{
   private static final boolean debug = false;
   private int maxIterations = Integer.MAX_VALUE;
   private double epsilon = 1e-12;

   private final DMatrixRMaj Wlocal = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj W = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj WInverse = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj WDiff = new DMatrixRMaj(0, 0);

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public void setConvergenceEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   @Override
   public boolean compute(DMatrixRMaj K)
   {
      int size = K.getNumRows();

      Wlocal.set(K);
      WDiff.reshape(size, size);
      W.reshape(size, size);
      WInverse.reshape(size, size);

      boolean converged = false;
      int iterations = 0;

      while (!converged)
      {
         if (iterations > maxIterations)
            return false;

         NativeCommonOps.invert(Wlocal, WInverse);

         if (debug)
            checkProperInversion(Wlocal, WInverse);

         CommonOps_DDRM.subtract(Wlocal, WInverse, WDiff);
         CommonOps_DDRM.add(Wlocal, -0.5, WDiff, W);

         converged = MatrixToolsLocal.distance(W, Wlocal) < epsilon;

         Wlocal.set(W);
         iterations++;
      }

      return true;
   }

   private static void checkProperInversion(DMatrixRMaj A, DMatrixRMaj Ainv)
   {
      int n = A.getNumRows();
      DMatrixRMaj expectedIdentify = new DMatrixRMaj(n, n);
      CommonOps_DDRM.mult(Ainv, A, expectedIdentify);
      for (int row = 0; row < n; row++)
      {
         for (int col = 0; col < n; col++)
         {
            if (row == col && !MathTools.epsilonEquals(1.0, expectedIdentify.get(row, col), 1e-7))
               throw new RuntimeException("Wrong inversion. Identity property should be held at 1, was " + expectedIdentify.get(row, col));
            else if (row != col && !MathTools.epsilonEquals(0.0, expectedIdentify.get(row, col), 1e-7))
               throw new RuntimeException("Wrong inversion. Off-diagonal property should be held at 0, was " + expectedIdentify.get(row, col));
         }
      }
   }

   @Override
   public DMatrixRMaj getW(DMatrixRMaj WToPack)
   {
      if (WToPack != null)
      {
         WToPack.set(W);
         return null;
      }
      else
         return new DMatrixRMaj(W);
   }
}
