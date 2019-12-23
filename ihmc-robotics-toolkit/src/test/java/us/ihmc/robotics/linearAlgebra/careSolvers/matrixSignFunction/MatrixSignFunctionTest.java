package us.ihmc.robotics.linearAlgebra.careSolvers.matrixSignFunction;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.NativeCommonOps;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public abstract class MatrixSignFunctionTest
{
   private static final int iters = 10;
   private static final double epsilon = 1e-6;

   protected abstract MatrixSignFunction getSolver();

   @Test
   public void testRandomByConstruction()
   {
      Random random = new Random(1738L);

      MatrixSignFunction function = getSolver();

      for (int iter = 0; iter < iters;  iter++)
      {
         int size = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F J = new DenseMatrix64F(size, size);

         double jordanColumn = RandomNumbers.nextDouble(random, 10.0);
         for (int i = 0; i < size; i++)
         {
            J.set(i, i, jordanColumn);
            if (i < size - 1)
               J.set(i, i + 1, 1.0);
         }


         // K = M J Minv
         DenseMatrix64F M = new DenseMatrix64F(size, size);
         DenseMatrix64F Minv = new DenseMatrix64F(size, size);
         DenseMatrix64F temp = new DenseMatrix64F(size, size);
         DenseMatrix64F K = new DenseMatrix64F(size, size);
         M.setData(RandomNumbers.nextDoubleArray(random, size * size, 10.0));
         NativeCommonOps.invert(M, Minv);
         CommonOps.mult(J, Minv, temp);
         CommonOps.mult(M, temp, K);

         // Sign(K) = W = M S Minv
         // S = Minv W M
         function.compute(K);
         DenseMatrix64F W = function.getW(null);
         DenseMatrix64F S = new DenseMatrix64F(size, size);
         CommonOps.mult(W, M, temp);
         CommonOps.mult(Minv, temp, S);

         for (int i = 0; i < size; i++)
         {
            for (int j = 0; j < size; j++)
            {
               if (i == j)
                  assertEquals("iteration " + iter + " Failed. Column = " + jordanColumn, 1.0, Math.abs(S.get(i, j)), epsilon);
               else
                  assertEquals("iteration " + iter + " Failed.", 0.0, S.get(i, j), epsilon);
            }
         }

      }
   }
}
