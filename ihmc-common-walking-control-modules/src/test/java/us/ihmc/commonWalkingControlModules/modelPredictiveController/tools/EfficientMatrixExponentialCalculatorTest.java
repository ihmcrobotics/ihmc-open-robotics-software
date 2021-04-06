package us.ihmc.commonWalkingControlModules.modelPredictiveController.tools;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

import java.util.Random;

public class EfficientMatrixExponentialCalculatorTest
{
   private static final int iters = 500;
   private static final double epsilon = 1e-8;

   @Test
   public void test()
   {
      MatrixExponentialCalculator exponentialCalculator = new MatrixExponentialCalculator(10);
      EfficientMatrixExponentialCalculator efficientExponentialCalculator = new EfficientMatrixExponentialCalculator(10, 10, 10);
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int rows = RandomNumbers.nextInt(random, 1, 20);
         int Bcols = RandomNumbers.nextInt(random, 1, 100);
         int Ccols = RandomNumbers.nextInt(random, 1, 20);
         int size = rows + Bcols + Ccols;

         DMatrixRMaj subA = new DMatrixRMaj(rows, rows);
         DMatrixRMaj subB = new DMatrixRMaj(rows, Bcols);
         DMatrixRMaj subC = new DMatrixRMaj(rows, Ccols);

         subA.setData(RandomNumbers.nextDoubleArray(random, rows * rows, 1.0));
         subB.setData(RandomNumbers.nextDoubleArray(random, rows * Bcols, 1.0));
         subC.setData(RandomNumbers.nextDoubleArray(random, rows * Ccols, 1.0));

         DMatrixRMaj A = new DMatrixRMaj(size, size);
         MatrixTools.setMatrixBlock(A, 0, 0, subA, 0, 0, rows, rows, 1.0);
         MatrixTools.setMatrixBlock(A, 0, rows, subB, 0, 0, rows, Bcols, 1.0);
         MatrixTools.setMatrixBlock(A, 0, rows + Bcols, subC, 0, 0, rows, Ccols, 1.0);


         DMatrixRMaj result = new DMatrixRMaj(size, size);
         DMatrixRMaj resultExpected = new DMatrixRMaj(size, size);

         exponentialCalculator.reshape(size);
         efficientExponentialCalculator.reshape(rows, Bcols, Ccols);

         exponentialCalculator.compute(resultExpected, A);
         efficientExponentialCalculator.compute(result, A);

         MatrixTestTools.assertMatrixEquals(resultExpected, result, epsilon);
      }
   }

}
