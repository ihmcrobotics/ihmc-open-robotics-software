package us.ihmc.robotics.linearAlgebra.careSolvers.matrixSignFunction;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixToolsLocal;

public class NewtonMatrixSignFunction implements MatrixSignFunction
{
   private int maxIterations = Integer.MAX_VALUE;
   private double epsilon = 1e-12;

   private final DenseMatrix64F Wprev = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F W = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F WInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F WDiff = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F WAlt = new DenseMatrix64F(0, 0);

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
      WDiff.reshape(size, size);
      WInverse.reshape(size, size);
      WAlt.reshape(size, size);

      boolean converged = false;
      int iterations = 0;

      while (!converged)
      {
         if (iterations > maxIterations)
            return false;

         NativeCommonOps.invert(Wprev, WInverse);

         CommonOps.subtract(Wprev, WInverse, WDiff);
         CommonOps.add(Wprev, -0.5, WDiff, W);

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
