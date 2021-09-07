package us.ihmc.robotics.blockMatrix;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.NativeMatrix;

import java.util.Random;

public class NativeBlockToolsTest
{
   private static final int iters = 500;


   @Test
   public void testEasyBlockAlignment()
   {
      Random random = new Random(1738L);

      NativeBlockMatrixRMaj leftMatrix = new NativeBlockMatrixRMaj();
      NativeBlockMatrixRMaj rightMatrix = new NativeBlockMatrixRMaj();

      NativeMatrixBlockTools blockTools = new NativeMatrixBlockTools();

      // test with the right matrix being one big block and the left being block diagonal.
      for (int iter = 0; iter < iters; iter++)
      {
         int block1Rows = RandomNumbers.nextInt(random, 1, 20);
         int block1Cols = RandomNumbers.nextInt(random, 1, 20);
         int block2Rows = RandomNumbers.nextInt(random, 1, 20);
         int block2Cols = RandomNumbers.nextInt(random, 1, 20);

         int totalLeftRows = block1Rows + block2Rows;
         int totalLeftCols = block1Cols + block2Cols;
         int rightCols = RandomNumbers.nextInt(random, 1, 40);

         DMatrixRMaj leftBlock1 = new DMatrixRMaj(block1Rows, block1Cols);
         DMatrixRMaj leftBlock2 = new DMatrixRMaj(block2Rows, block2Cols);
         DMatrixRMaj rightBlock = new DMatrixRMaj(totalLeftCols, rightCols);
         leftBlock1.setData(RandomNumbers.nextDoubleArray(random, block1Rows * block1Cols, 10.0));
         leftBlock2.setData(RandomNumbers.nextDoubleArray(random, block2Rows * block2Cols, 10.0));
         rightBlock.setData(RandomNumbers.nextDoubleArray(random, totalLeftCols * rightCols, 10.0));

         leftMatrix.clear();
         rightMatrix.clear();

         leftMatrix.addBlock(leftBlock1, 0, 0);
         leftMatrix.addBlock(leftBlock2, block1Rows, block1Cols);
         rightMatrix.addBlock(rightBlock, 0, 0);

         NativeMatrix results = new NativeMatrix(totalLeftRows, rightCols);
         NativeMatrix resultsExpected = new NativeMatrix(totalLeftRows, rightCols);
         blockTools.mult(results, leftMatrix, rightMatrix);

         NativeMatrix leftAlt = new NativeMatrix(totalLeftRows, totalLeftCols);
         leftAlt.zero();
         leftAlt.insert(leftBlock1, 0, 0);
         leftAlt.insert(leftBlock2, block1Rows, block1Cols);
         NativeMatrix rightAlt = new NativeMatrix(rightBlock);
         resultsExpected.mult(leftAlt, rightAlt);

         MatrixTestTools.assertMatrixEquals(resultsExpected, results, 1e-6);
      }

      // test with the right matrix composed of four blocks, and left two
      for (int iter = 0; iter < iters; iter++)
      {
         int block1Rows = RandomNumbers.nextInt(random, 1, 20);
         int block1Cols = RandomNumbers.nextInt(random, 1, 20);
         int block2Rows = RandomNumbers.nextInt(random, 1, 20);
         int block2Cols = RandomNumbers.nextInt(random, 1, 20);

         int totalLeftRows = block1Rows + block2Rows;
         int totalLeftCols = block1Cols + block2Cols;
         int rightCol1 = RandomNumbers.nextInt(random, 1, 20);
         int rightCol2 = RandomNumbers.nextInt(random, 1, 20);
         int totalRightCols = rightCol1 + rightCol2;

         DMatrixRMaj leftBlock1 = new DMatrixRMaj(block1Rows, block1Cols);
         DMatrixRMaj leftBlock2 = new DMatrixRMaj(block2Rows, block2Cols);

         leftBlock1.setData(RandomNumbers.nextDoubleArray(random, block1Rows * block1Cols, 10.0));
         leftBlock2.setData(RandomNumbers.nextDoubleArray(random, block2Rows * block2Cols, 10.0));

         DMatrixRMaj rightTopLeftBlock = new DMatrixRMaj(block1Cols, rightCol1);
         DMatrixRMaj rightTopRightBlock = new DMatrixRMaj(block1Cols, rightCol2);
         DMatrixRMaj rightBottomLeftBlock = new DMatrixRMaj(block2Cols, rightCol1);
         DMatrixRMaj rightBottomRightBlock = new DMatrixRMaj(block2Cols, rightCol2);
         rightTopLeftBlock.setData(RandomNumbers.nextDoubleArray(random, block1Cols * rightCol1, 10.0));
         rightTopRightBlock.setData(RandomNumbers.nextDoubleArray(random, block1Cols * rightCol2, 10.0));
         rightBottomLeftBlock.setData(RandomNumbers.nextDoubleArray(random, block2Cols * rightCol1, 10.0));
         rightBottomRightBlock.setData(RandomNumbers.nextDoubleArray(random, block2Cols * rightCol2, 10.0));

         leftMatrix.clear();
         rightMatrix.clear();

         leftMatrix.addBlock(leftBlock1, 0, 0);
         leftMatrix.addBlock(leftBlock2, block1Rows, block1Cols);

         rightMatrix.addBlock(rightTopLeftBlock, 0, 0);
         rightMatrix.addBlock(rightTopRightBlock, 0, rightCol1);
         rightMatrix.addBlock(rightBottomLeftBlock, block1Cols, 0);
         rightMatrix.addBlock(rightBottomRightBlock, block1Cols, rightCol1);

         NativeMatrix results = new NativeMatrix(totalLeftRows, totalRightCols);
         NativeMatrix resultsExpected = new NativeMatrix(totalLeftRows, totalRightCols);
         blockTools.mult(results, leftMatrix, rightMatrix);

         NativeMatrix leftAlt = new NativeMatrix(totalLeftRows, totalLeftCols);
         NativeMatrix rightAlt = new NativeMatrix(totalLeftCols, totalRightCols);
         leftAlt.zero();
         leftAlt.insert(leftBlock1, 0, 0);
         leftAlt.insert(leftBlock2, block1Rows, block1Cols);

         rightAlt.zero();
         rightAlt.insert(rightTopLeftBlock, 0, 0);
         rightAlt.insert(rightTopRightBlock, 0, rightCol1);
         rightAlt.insert(rightBottomLeftBlock, block1Cols, 0);
         rightAlt.insert(rightBottomRightBlock, block1Cols, rightCol1);

         resultsExpected.mult(leftAlt, rightAlt);

         MatrixTestTools.assertMatrixEquals(resultsExpected, results, 1e-6);
      }

      // test with the left and right matrices randomly composed of blocks
      for (int iter = 0; iter < iters; iter++)
      {
         int leftRows1 = RandomNumbers.nextInt(random, 1, 20);
         int leftCols1 = RandomNumbers.nextInt(random, 1, 20);
         int leftRows2 = RandomNumbers.nextInt(random, 1, 20);
         int leftCols2 = RandomNumbers.nextInt(random, 1, 20);
         int rightCols1 = RandomNumbers.nextInt(random, 1, 20);
         int rightCols2 = RandomNumbers.nextInt(random, 1, 20);

         int totalLeftRows = leftRows1 + leftRows2;
         int totalLeftCols = leftCols1 + leftCols2;
         int totalRightCols = rightCols1 + rightCols2;

         DMatrixRMaj leftTopLeftBlock = new DMatrixRMaj(leftRows1, leftCols1);
         DMatrixRMaj leftTopRightBlock = new DMatrixRMaj(leftRows1, leftCols2);
         DMatrixRMaj leftBottomLeftBlock = new DMatrixRMaj(leftRows2, leftCols1);
         DMatrixRMaj leftBottomRightBlock = new DMatrixRMaj(leftRows2, leftCols2);

         leftTopLeftBlock.setData(RandomNumbers.nextDoubleArray(random, leftRows1 * leftCols1, 10.0));
         leftTopRightBlock.setData(RandomNumbers.nextDoubleArray(random, leftRows1 * leftCols2, 10.0));
         leftBottomLeftBlock.setData(RandomNumbers.nextDoubleArray(random, leftRows2 * leftCols1, 10.0));
         leftBottomRightBlock.setData(RandomNumbers.nextDoubleArray(random, leftRows2 * leftCols2, 10.0));

         DMatrixRMaj rightTopLeftBlock = new DMatrixRMaj(leftCols1, rightCols1);
         DMatrixRMaj rightTopRightBlock = new DMatrixRMaj(leftCols1, rightCols2);
         DMatrixRMaj rightBottomLeftBlock = new DMatrixRMaj(leftCols2, rightCols1);
         DMatrixRMaj rightBottomRightBlock = new DMatrixRMaj(leftCols2, rightCols2);

         rightTopLeftBlock.setData(RandomNumbers.nextDoubleArray(random, leftCols1 * rightCols1, 10.0));
         rightTopRightBlock.setData(RandomNumbers.nextDoubleArray(random, leftCols1 * rightCols2, 10.0));
         rightBottomLeftBlock.setData(RandomNumbers.nextDoubleArray(random, leftCols2 * rightCols1, 10.0));
         rightBottomRightBlock.setData(RandomNumbers.nextDoubleArray(random, leftCols2 * rightCols2, 10.0));

         leftMatrix.clear();
         rightMatrix.clear();

         NativeMatrix leftAlt = new NativeMatrix(totalLeftRows, totalLeftCols);
         NativeMatrix rightAlt = new NativeMatrix(totalLeftCols, totalRightCols);
         leftAlt.zero();
         rightAlt.zero();

         if (random.nextBoolean())
         {
            leftAlt.insert(leftTopLeftBlock, 0, 0);
            leftMatrix.addBlock(leftTopLeftBlock, 0, 0);
         }
         if (random.nextBoolean())
         {
            leftAlt.insert(leftTopRightBlock, 0, leftCols1);
            leftMatrix.addBlock(leftTopRightBlock, 0, leftCols1);
         }
         if (random.nextBoolean())
         {
            leftAlt.insert(leftBottomLeftBlock, leftRows1, 0);
            leftMatrix.addBlock(leftBottomLeftBlock, leftRows1, 0);
         }
         if (random.nextBoolean())
         {
            leftAlt.insert(leftBottomRightBlock, leftRows1, leftCols1);
            leftMatrix.addBlock(leftBottomRightBlock, leftRows1, leftCols1);
         }

         if (random.nextBoolean())
         {
            rightAlt.insert(rightTopLeftBlock, 0, 0);
            rightMatrix.addBlock(rightTopLeftBlock, 0, 0);
         }
         if (random.nextBoolean())
         {
            rightAlt.insert(rightTopRightBlock, 0, rightCols1);
            rightMatrix.addBlock(rightTopRightBlock, 0, rightCols1);
         }
         if (random.nextBoolean())
         {
            rightAlt.insert(rightBottomLeftBlock, leftCols1, 0);
            rightMatrix.addBlock(rightBottomLeftBlock, leftCols1, 0);
         }
         if (random.nextBoolean())
         {
            rightAlt.insert(rightBottomRightBlock, leftCols1, rightCols1);
            rightMatrix.addBlock(rightBottomRightBlock, leftCols1, rightCols1);
         }

         NativeMatrix results = new NativeMatrix(totalLeftRows, totalRightCols);
         NativeMatrix resultsExpected = new NativeMatrix(totalLeftRows, totalRightCols);
         blockTools.mult(results, leftMatrix, rightMatrix);


         resultsExpected.mult(leftAlt, rightAlt);

         MatrixTestTools.assertMatrixEquals(resultsExpected, results, 1e-6);
      }
   }


   @Test
   public void testWithoutBlockAlignment()
   {
      Random random = new Random(1738L);

      NativeBlockMatrixRMaj leftMatrix = new NativeBlockMatrixRMaj();
      NativeBlockMatrixRMaj rightMatrix = new NativeBlockMatrixRMaj();

      NativeMatrixBlockTools blockTools = new NativeMatrixBlockTools();


      // test with the right matrix composed of four blocks, and left two
      for (int iter = 0; iter < iters; iter++)
      {
         int leftRows1 = RandomNumbers.nextInt(random, 1, 20);
         int leftRows2 = RandomNumbers.nextInt(random, 1, 20);
         int leftCols1 = RandomNumbers.nextInt(random, 1, 20);
         int leftCols2 = RandomNumbers.nextInt(random, 1, 20);

         int totalLeftRows = leftRows1 + leftRows2;
         int totalLeftCols = leftCols1 + leftCols2;

         int rightRows1 = RandomNumbers.nextInt(random, 1, totalLeftCols - 1);
         int rightRows2 = totalLeftCols - rightRows1;
         int rightCol1 = RandomNumbers.nextInt(random, 1, 20);
         int rightCol2 = RandomNumbers.nextInt(random, 1, 20);

         int totalRightCols = rightCol1 + rightCol2;

         DMatrixRMaj leftBlock1 = new DMatrixRMaj(leftRows1, leftCols1);
         DMatrixRMaj leftBlock2 = new DMatrixRMaj(leftRows2, leftCols2);

         leftBlock1.setData(RandomNumbers.nextDoubleArray(random, leftRows1 * leftCols1, 10.0));
         leftBlock2.setData(RandomNumbers.nextDoubleArray(random, leftRows2 * leftCols2, 10.0));

         DMatrixRMaj rightTopLeftBlock = new DMatrixRMaj(rightRows1, rightCol1);
         DMatrixRMaj rightTopRightBlock = new DMatrixRMaj(rightRows1, rightCol2);
         DMatrixRMaj rightBottomLeftBlock = new DMatrixRMaj(rightRows2, rightCol1);
         DMatrixRMaj rightBottomRightBlock = new DMatrixRMaj(rightRows2, rightCol2);

         rightTopLeftBlock.setData(RandomNumbers.nextDoubleArray(random, rightRows1 * rightCol1, 10.0));
         rightTopRightBlock.setData(RandomNumbers.nextDoubleArray(random, rightRows1 * rightCol2, 10.0));
         rightBottomLeftBlock.setData(RandomNumbers.nextDoubleArray(random, rightRows2 * rightCol1, 10.0));
         rightBottomRightBlock.setData(RandomNumbers.nextDoubleArray(random, rightRows2 * rightCol2, 10.0));

         leftMatrix.clear();
         rightMatrix.clear();

         leftMatrix.addBlock(leftBlock1, 0, 0);
         leftMatrix.addBlock(leftBlock2, leftRows1, leftCols1);

         rightMatrix.addBlock(rightTopLeftBlock, 0, 0);
         rightMatrix.addBlock(rightTopRightBlock, 0, rightCol1);
         rightMatrix.addBlock(rightBottomLeftBlock, rightRows1, 0);
         rightMatrix.addBlock(rightBottomRightBlock, rightRows1, rightCol1);

         NativeMatrix results = new NativeMatrix(totalLeftRows, totalRightCols);
         NativeMatrix resultsExpected = new NativeMatrix(totalLeftRows, totalRightCols);
         blockTools.mult(results, leftMatrix, rightMatrix);

         NativeMatrix leftAlt = new NativeMatrix(totalLeftRows, totalLeftCols);
         NativeMatrix rightAlt = new NativeMatrix(totalLeftCols, totalRightCols);
         leftAlt.zero();
         leftAlt.insert(leftBlock1, 0, 0);
         leftAlt.insert(leftBlock2, leftRows1, leftCols1);

         rightAlt.zero();
         rightAlt.insert(rightTopLeftBlock, 0, 0);
         rightAlt.insert(rightTopRightBlock, 0, rightCol1);
         rightAlt.insert(rightBottomLeftBlock, rightRows1, 0);
         rightAlt.insert(rightBottomRightBlock, rightRows1, rightCol1);

         resultsExpected.mult(leftAlt, rightAlt);

         MatrixTestTools.assertMatrixEquals(resultsExpected, results, 1e-6);
      }

      // test with the left and right matrices randomly composed of blocks
      for (int iter = 0; iter < iters; iter++)
      {
         int leftRows1 = RandomNumbers.nextInt(random, 1, 20);
         int leftCols1 = RandomNumbers.nextInt(random, 1, 20);
         int leftRows2 = RandomNumbers.nextInt(random, 1, 20);
         int leftCols2 = RandomNumbers.nextInt(random, 1, 20);

         int totalLeftRows = leftRows1 + leftRows2;
         int totalLeftCols = leftCols1 + leftCols2;


         int rightRows1 = RandomNumbers.nextInt(random, 1, totalLeftCols - 1);
         int rightRows2 = totalLeftCols - rightRows1;
         int rightCols1 = RandomNumbers.nextInt(random, 1, 20);
         int rightCols2 = RandomNumbers.nextInt(random, 1, 20);

         int totalRightCols = rightCols1 + rightCols2;

         DMatrixRMaj leftTopLeftBlock = new DMatrixRMaj(leftRows1, leftCols1);
         DMatrixRMaj leftTopRightBlock = new DMatrixRMaj(leftRows1, leftCols2);
         DMatrixRMaj leftBottomLeftBlock = new DMatrixRMaj(leftRows2, leftCols1);
         DMatrixRMaj leftBottomRightBlock = new DMatrixRMaj(leftRows2, leftCols2);

         leftTopLeftBlock.setData(RandomNumbers.nextDoubleArray(random, leftRows1 * leftCols1, 10.0));
         leftTopRightBlock.setData(RandomNumbers.nextDoubleArray(random, leftRows1 * leftCols2, 10.0));
         leftBottomLeftBlock.setData(RandomNumbers.nextDoubleArray(random, leftRows2 * leftCols1, 10.0));
         leftBottomRightBlock.setData(RandomNumbers.nextDoubleArray(random, leftRows2 * leftCols2, 10.0));

         DMatrixRMaj rightTopLeftBlock = new DMatrixRMaj(rightRows1, rightCols1);
         DMatrixRMaj rightTopRightBlock = new DMatrixRMaj(rightRows1, rightCols2);
         DMatrixRMaj rightBottomLeftBlock = new DMatrixRMaj(rightRows2, rightCols1);
         DMatrixRMaj rightBottomRightBlock = new DMatrixRMaj(rightRows2, rightCols2);

         rightTopLeftBlock.setData(RandomNumbers.nextDoubleArray(random, rightRows1 * rightCols1, 10.0));
         rightTopRightBlock.setData(RandomNumbers.nextDoubleArray(random, rightRows1 * rightCols2, 10.0));
         rightBottomLeftBlock.setData(RandomNumbers.nextDoubleArray(random, rightRows2 * rightCols1, 10.0));
         rightBottomRightBlock.setData(RandomNumbers.nextDoubleArray(random, rightRows2 * rightCols2, 10.0));

         leftMatrix.clear();
         rightMatrix.clear();

         NativeMatrix leftAlt = new NativeMatrix(totalLeftRows, totalLeftCols);
         NativeMatrix rightAlt = new NativeMatrix(totalLeftCols, totalRightCols);
         leftAlt.zero();
         rightAlt.zero();

         if (random.nextBoolean())
         {
            leftAlt.insert(leftTopLeftBlock, 0, 0);
            leftMatrix.addBlock(leftTopLeftBlock, 0, 0);
         }
         if (random.nextBoolean())
         {
            leftAlt.insert(leftTopRightBlock, 0, leftCols1);
            leftMatrix.addBlock(leftTopRightBlock, 0, leftCols1);
         }
         if (random.nextBoolean())
         {
            leftAlt.insert(leftBottomLeftBlock, leftRows1, 0);
            leftMatrix.addBlock(leftBottomLeftBlock, leftRows1, 0);
         }
         if (random.nextBoolean())
         {
            leftAlt.insert(leftBottomRightBlock, leftRows1, leftCols1);
            leftMatrix.addBlock(leftBottomRightBlock, leftRows1, leftCols1);
         }

         if (random.nextBoolean())
         {
            rightAlt.insert(rightTopLeftBlock, 0, 0);
            rightMatrix.addBlock(rightTopLeftBlock, 0, 0);
         }
         if (random.nextBoolean())
         {
            rightAlt.insert(rightTopRightBlock, 0, rightCols1);
            rightMatrix.addBlock(rightTopRightBlock, 0, rightCols1);
         }
         if (random.nextBoolean())
         {
            rightAlt.insert(rightBottomLeftBlock, rightRows1, 0);
            rightMatrix.addBlock(rightBottomLeftBlock, rightRows1, 0);
         }
         if (random.nextBoolean())
         {
            rightAlt.insert(rightBottomRightBlock, rightRows1, rightCols1);
            rightMatrix.addBlock(rightBottomRightBlock, rightRows1, rightCols1);
         }

         NativeMatrix results = new NativeMatrix(totalLeftRows, totalRightCols);
         NativeMatrix resultsExpected = new NativeMatrix(totalLeftRows, totalRightCols);
         blockTools.mult(results, leftMatrix, rightMatrix);


         resultsExpected.mult(leftAlt, rightAlt);

         MatrixTestTools.assertMatrixEquals(resultsExpected, results, 1e-6);
      }
   }
}
