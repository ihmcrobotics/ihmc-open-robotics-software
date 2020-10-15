package us.ihmc.robotics.linearAlgebra.careSolvers.signFunction;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixToolsLocal;

public class QuadraticSignFunction implements SignFunction
{
   private int maxIterations = Integer.MAX_VALUE;
   private double epsilon = 1e-12;

   private final DMatrixRMaj Wprev = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj W = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj ZInverse = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Z = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj ZDiff = new DMatrixRMaj(0, 0);

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

      Wprev.set(K);
      W.reshape(size, size);
      Z.reshape(size, size);
      ZInverse.reshape(size, size);
      ZDiff.reshape(size, size);

      boolean converged = false;
      int iterations = 0;

      while (!converged)
      {
         if (iterations > maxIterations)
            return false;

         double determinate = CommonOps_DDRM.det(Wprev);
         double c = Math.pow(Math.abs(determinate), -1.0 / (2 * size));
         Z.set(Wprev);
         CommonOps_DDRM.scale(c, Z);

         NativeCommonOps.invert(Z, ZInverse);

         CommonOps_DDRM.subtract(Z, ZInverse, ZDiff);
         CommonOps_DDRM.add(Z, -0.5, ZDiff, W);

         converged = MatrixToolsLocal.distance(W, Wprev) < epsilon;

         Wprev.set(W);
         iterations++;
      }

      return true;
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
