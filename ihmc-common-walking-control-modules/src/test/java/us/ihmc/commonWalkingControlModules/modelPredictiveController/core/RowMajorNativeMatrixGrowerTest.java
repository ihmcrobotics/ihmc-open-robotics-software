package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.NativeMatrix;

import java.util.Random;

public class RowMajorNativeMatrixGrowerTest
{
   private static final int iters = 1000;
   @Test
   public void testGrowSomething()
   {
      Random random = new Random(1738L);

      RowMajorNativeMatrixGrower grower = new RowMajorNativeMatrixGrower();

      NativeMatrix originalNativeMatrix = new NativeMatrix(0, 0);
      NativeMatrix nativeMatrixToAdd = new NativeMatrix(0, 0);

      for (int iter = 0; iter < iters; iter++)
      {
         int originalRows = RandomNumbers.nextInt(random, 10, 100);
         int originalCols = RandomNumbers.nextInt(random, 10, 100);
         int rowsToAdd = RandomNumbers.nextInt(random, 10, 100);
         int colsToAdd = RandomNumbers.nextInt(random, 10, originalCols);
         int colOffset = RandomNumbers.nextInt(random, 0, originalCols - colsToAdd);

         DMatrixRMaj originalMatrix = RandomMatrices_DDRM.rectangle(originalRows, originalCols, -10.0, 10.0, random);
         DMatrixRMaj matrixToAdd = RandomMatrices_DDRM.rectangle(rowsToAdd, colsToAdd, -10.0, 10.0, random);

         originalNativeMatrix.set(RandomMatrices_DDRM.rectangle(originalRows + rowsToAdd, originalCols, -10.0, 10.0, random));
         originalNativeMatrix.set(originalMatrix);
         nativeMatrixToAdd.set(matrixToAdd);

         originalMatrix.reshape(originalRows + rowsToAdd, originalCols, true);
         CommonOps_DDRM.insert(matrixToAdd, originalMatrix, originalRows, colOffset);

         grower.appendRows(originalNativeMatrix, colOffset, nativeMatrixToAdd);

         EjmlUnitTests.assertEquals(matrixToAdd, nativeMatrixToAdd, 1e-5);
      }

   }
}
