package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;

import java.util.Random;

public class BlockInverseCalculatorTest
{
   private static final int iters = 10;
   private static final double epsilon = 1e-5;

   @Test
   public void testCalculateInverseOfOneSegment()
   {
      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      indexHandler.initialize((i) -> 4, 1);


      int size = indexHandler.getTotalProblemSize();
      Random random = new Random(1738L);
      BlockInverseCalculator inverseCalculator = new BlockInverseCalculator(indexHandler, i -> indexHandler.getRhoCoefficientsInSegment(i) + LinearMPCIndexHandler.comCoefficientsPerSegment);
      for (int i = 0; i < iters; i++)
      {
         DMatrixRMaj matrixRMaj = new DMatrixRMaj(size, size);
         matrixRMaj.setData(RandomNumbers.nextDoubleArray(random, size * size, 1.0));
         MatrixTools.addDiagonal(matrixRMaj, 100.0);

         NativeMatrix nativeMatrix = new NativeMatrix(size, size);
         nativeMatrix.set(matrixRMaj);

         NativeMatrix inverseMatrix = new NativeMatrix(size, size);
         NativeMatrix inverseMatrixExpected = new NativeMatrix(size, size);

         inverseCalculator.computeInverse(nativeMatrix, inverseMatrix);

         inverseMatrixExpected.invert(nativeMatrix);

         EjmlUnitTests.assertEquals(inverseMatrixExpected, inverseMatrix, epsilon);
      }
   }

   @Test
   public void testCalculateInverseOfTwoSegments()
   {
      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      indexHandler.initialize((i) -> 4, 2);


      int blockSize = 6 + indexHandler.getRhoCoefficientsInSegment(0);
      int size = indexHandler.getTotalProblemSize();

      Random random = new Random(1738L);
      BlockInverseCalculator inverseCalculator = new BlockInverseCalculator(indexHandler, i -> indexHandler.getRhoCoefficientsInSegment(i) + LinearMPCIndexHandler.comCoefficientsPerSegment);
      for (int i = 0; i < iters; i++)
      {
         DMatrixRMaj matrixRMaj = new DMatrixRMaj(blockSize, blockSize);
         matrixRMaj.setData(RandomNumbers.nextDoubleArray(random, size * size, 1.0));
         MatrixTools.addDiagonal(matrixRMaj, 100.0);

         NativeMatrix nativeMatrix = new NativeMatrix(size, size);
         nativeMatrix.zero();
         nativeMatrix.insert(matrixRMaj, 0, 0);
         nativeMatrix.insert(matrixRMaj, blockSize, blockSize);

         NativeMatrix inverseMatrix = new NativeMatrix(size, size);
         NativeMatrix inverseMatrixExpected = new NativeMatrix(size, size);

         inverseCalculator.computeInverse(nativeMatrix, inverseMatrix);

         inverseMatrixExpected.invert(nativeMatrix);

         EjmlUnitTests.assertEquals(inverseMatrixExpected, inverseMatrix, epsilon);
      }
   }
}
