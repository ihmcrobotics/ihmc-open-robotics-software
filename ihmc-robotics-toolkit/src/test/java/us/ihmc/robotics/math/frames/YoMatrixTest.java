package us.ihmc.robotics.math.frames;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoMatrixTest
{
   @Test
   public void testSimpleYoMatrixExample()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      YoRegistry registry = new YoRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix("testMatrix", maxNumberOfRows, maxNumberOfColumns, registry);
      assertEquals(maxNumberOfRows, yoMatrix.getNumRows());
      assertEquals(maxNumberOfColumns, yoMatrix.getNumCols());

      DMatrixRMaj denseMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(denseMatrix);

      MatrixTestTools.assertMatrixEqualsZero(denseMatrix, 1e-10);

      Random random = new Random(1984L);

      DMatrixRMaj randomMatrix = RandomGeometry.nextDenseMatrix64F(random, maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.set(randomMatrix);

      DMatrixRMaj checkMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(checkMatrix);

      MatrixTestTools.assertMatrixEquals(randomMatrix, checkMatrix, 1e-10);

      assertEquals(registry.findVariable("testMatrix_0_0").getValueAsDouble(), checkMatrix.get(0, 0), 1e-10);
   }

   @Test
   public void testYoMatrixDimensioning()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      String name = "testMatrix";
      
      YoRegistry registry = new YoRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix(name, maxNumberOfRows, maxNumberOfColumns, registry);

      int smallerRows = maxNumberOfRows - 2;
      int smallerColumns = maxNumberOfColumns - 3;
      DMatrixRMaj denseMatrix = new DMatrixRMaj(smallerRows, smallerColumns);

      try
      {
         yoMatrix.get(denseMatrix);
         fail("Should throw an exception if the size isn't right!");
      }
      catch (Exception e)
      {
      }

      yoMatrix.getAndReshape(denseMatrix);
      MatrixTestTools.assertMatrixEqualsZero(denseMatrix, 1e-10);
      assertEquals(maxNumberOfRows, denseMatrix.getNumRows());
      assertEquals(maxNumberOfColumns, denseMatrix.getNumCols());

      assertEquals(maxNumberOfRows, yoMatrix.getNumRows());
      assertEquals(maxNumberOfColumns, yoMatrix.getNumCols());

      Random random = new Random(1984L);

      DMatrixRMaj randomMatrix = RandomGeometry.nextDenseMatrix64F(random, maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.set(randomMatrix);

      DMatrixRMaj checkMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(checkMatrix);

      MatrixTestTools.assertMatrixEquals(randomMatrix, checkMatrix, 1e-10);

      DMatrixRMaj smallerMatrix = RandomGeometry.nextDenseMatrix64F(random, smallerRows, smallerColumns);
      yoMatrix.set(smallerMatrix);

      assertEquals(smallerRows, smallerMatrix.getNumRows());
      assertEquals(smallerColumns, smallerMatrix.getNumCols());

      assertEquals(smallerRows, yoMatrix.getNumRows());
      assertEquals(smallerColumns, yoMatrix.getNumCols());

      DMatrixRMaj checkMatrix2 = new DMatrixRMaj(1, 1);
      yoMatrix.getAndReshape(checkMatrix2);

      MatrixTestTools.assertMatrixEquals(smallerMatrix, checkMatrix2, 1e-10);

      checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, checkMatrix2, registry);
   }
   
   @Test
   public void testYoMatrixSetToZero()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      String name = "testMatrixForZero";
      
      YoRegistry registry = new YoRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix(name, maxNumberOfRows, maxNumberOfColumns, registry);
      
      Random random = new Random(1984L);

      DMatrixRMaj randomMatrix = RandomGeometry.nextDenseMatrix64F(random, maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.set(randomMatrix);
      
      int numberOfRows = 2;
      int numberOfColumns = 6;
      yoMatrix.zero();
      
      DMatrixRMaj zeroMatrix = new DMatrixRMaj(numberOfRows, numberOfColumns);
      checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, zeroMatrix, registry);  
   }
   
   @Test
   public void testYoMatrixSetTooBig()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      String name = "testMatrix";
      YoRegistry registry = new YoRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix(name, maxNumberOfRows, maxNumberOfColumns, registry);

      DMatrixRMaj tooBigMatrix = new DMatrixRMaj(maxNumberOfRows + 1, maxNumberOfColumns);

      try
      {
         yoMatrix.set(tooBigMatrix);
         fail("Too Big");
      }
      catch (RuntimeException e)
      {
      }

      tooBigMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns + 1);

      try
      {
         yoMatrix.set(tooBigMatrix);
         fail("Too Big");
      }
      catch (RuntimeException e)
      {
      }

      // Test a 0 X Big Matrix
      DMatrixRMaj okMatrix = new DMatrixRMaj(0, maxNumberOfColumns + 10);
      yoMatrix.set(okMatrix);
      assertMatrixYoVariablesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, registry);
      
      DMatrixRMaj checkMatrix = new DMatrixRMaj(1, 1);
      yoMatrix.getAndReshape(checkMatrix);
      
      assertEquals(0, checkMatrix.getNumRows());
      assertEquals(maxNumberOfColumns + 10, checkMatrix.getNumCols());
      
      // Test a Big X 0 Matrix

      okMatrix = new DMatrixRMaj(maxNumberOfRows + 10, 0);
      yoMatrix.set(okMatrix);
      assertMatrixYoVariablesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, registry);
      
      checkMatrix = new DMatrixRMaj(1, 1);
      yoMatrix.getAndReshape(checkMatrix);
      
      assertEquals(maxNumberOfRows + 10, checkMatrix.getNumRows());
      assertEquals(0, checkMatrix.getNumCols());
      
   }


   private void checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(String name, int maxNumberOfRows, int maxNumberOfColumns, DMatrixRMaj checkMatrix, YoRegistry registry)
   {
      int smallerRows = checkMatrix.getNumRows();
      int smallerColumns = checkMatrix.getNumCols();
      
   // Make sure the values are correct, including values outside the range should be NaN:
      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            YoDouble variable = (YoDouble) registry.findVariable(name + "_" + row + "_" + column);

            if ((row < smallerRows) && (column < smallerColumns))
            {
               assertEquals(checkMatrix.get(row, column), variable.getDoubleValue(), 1e-10);
            }
            else
            {
               assertTrue(Double.isNaN(variable.getDoubleValue()));
            }

         }
      }
   }
   
   private void assertMatrixYoVariablesAreNaN(String name, int maxNumberOfRows, int maxNumberOfColumns, YoRegistry registry)
   {
      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            YoDouble variable = (YoDouble) registry.findVariable(name + "_" + row + "_" + column);
            assertTrue(Double.isNaN(variable.getDoubleValue()));
         }
      }
   }

}
