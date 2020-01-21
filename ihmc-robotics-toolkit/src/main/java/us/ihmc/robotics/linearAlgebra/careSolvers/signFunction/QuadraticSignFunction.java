package us.ihmc.robotics.linearAlgebra.careSolvers.signFunction;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixToolsLocal;

public class QuadraticSignFunction implements SignFunction
{
   private int maxIterations = Integer.MAX_VALUE;
   private double epsilon = 1e-12;

   private final DenseMatrix64F Wprev = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F W = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ZInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Z = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ZDiff = new DenseMatrix64F(0, 0);

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public void setConvergenceEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   @Override
   public boolean compute(DenseMatrix64F K)
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

         double determinate = CommonOps.det(Wprev);
         double c = Math.pow(Math.abs(determinate), -1.0 / (2 * size));
         Z.set(Wprev);
         CommonOps.scale(c, Z);

         NativeCommonOps.invert(Z, ZInverse);

         CommonOps.subtract(Z, ZInverse, ZDiff);
         CommonOps.add(Z, -0.5, ZDiff, W);

         converged = MatrixToolsLocal.distance(W, Wprev) < epsilon;

         Wprev.set(W);
         iterations++;
      }

      return true;
   }

   @Override
   public DenseMatrix64F getW(DenseMatrix64F WToPack)
   {
      if (WToPack != null)
      {
         WToPack.set(W);
         return null;
      }
      else
         return new DenseMatrix64F(W);
   }
}
