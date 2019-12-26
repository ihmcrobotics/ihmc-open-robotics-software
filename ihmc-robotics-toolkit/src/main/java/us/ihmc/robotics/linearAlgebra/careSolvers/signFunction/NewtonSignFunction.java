package us.ihmc.robotics.linearAlgebra.careSolvers.signFunction;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixToolsLocal;

public class NewtonSignFunction implements SignFunction
{
   private static final boolean debug = false;
   private int maxIterations = Integer.MAX_VALUE;
   private double epsilon = 1e-12;

   private final DenseMatrix64F Wlocal = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F W = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F WInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F WDiff = new DenseMatrix64F(0, 0);

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

         CommonOps.subtract(Wlocal, WInverse, WDiff);
         CommonOps.add(Wlocal, -0.5, WDiff, W);

         converged = MatrixToolsLocal.distance(W, Wlocal) < epsilon;

         Wlocal.set(W);
         iterations++;
      }

      return true;
   }

   private static void checkProperInversion(DenseMatrix64F A, DenseMatrix64F Ainv)
   {
      int n = A.getNumRows();
      DenseMatrix64F expectedIdentify = new DenseMatrix64F(n, n);
      CommonOps.mult(Ainv, A, expectedIdentify);
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
