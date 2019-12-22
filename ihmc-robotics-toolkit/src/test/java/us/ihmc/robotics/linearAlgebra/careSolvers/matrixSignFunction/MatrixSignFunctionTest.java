package us.ihmc.robotics.linearAlgebra.careSolvers.matrixSignFunction;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public abstract class MatrixSignFunctionTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-6;

   protected abstract MatrixSignFunction getSolver();

   @Test
   public void testRandom()
   {
      Random random = new Random(1738L);

      MatrixSignFunction function = getSolver();

      for (int iter = 0; iter < iters;  iter++)
      {
         int size = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F K = new DenseMatrix64F(size, size);
         K.setData(RandomNumbers.nextDoubleArray(random, size * size, 10.0));
         function.compute(K);
         DenseMatrix64F W = function.getW(null);

      }
   }

}
