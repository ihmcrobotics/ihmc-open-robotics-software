package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTools;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.linearAlgebra.careSolvers.CARESolverTestTools.generateRandomSymmetricMatrix;

public class CARESolverTestToolsTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-7;

   @Test
   public void testRandomSymmetricMatrix()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int size = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F matrix = generateRandomSymmetricMatrix(random, size);

         for (int row = 0; row < size; row++)
         {
            for (int col = 0; col < size; col++)
            {
               assertEquals(matrix.get(row, col), matrix.get(col, row), epsilon);
            }
         }
      }
   }

}
