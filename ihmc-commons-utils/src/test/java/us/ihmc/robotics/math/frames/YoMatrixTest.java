package us.ihmc.robotics.math.frames;

import java.util.Random;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.filters.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.*;

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
      yoMatrix.reshape(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.zero();
      yoMatrix.get(denseMatrix);

      DMatrixRMaj zeroMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      EjmlUnitTests.assertEquals(zeroMatrix, denseMatrix, 1e-10);

      Random random = new Random(1984L);

      DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(maxNumberOfRows, maxNumberOfColumns, random);
      yoMatrix.set(randomMatrix);

      DMatrixRMaj checkMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(checkMatrix);

      EjmlUnitTests.assertEquals(randomMatrix, checkMatrix, 1e-10);

      assertEquals(registry.findVariable(YoMatrix.getFieldName("testMatrix", 0, 0)).getValueAsDouble(), checkMatrix.get(0, 0), 1e-10);
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

      yoMatrix.reshape(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.zero();
      yoMatrix.getAndReshape(denseMatrix);
      DMatrixRMaj zeroMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      EjmlUnitTests.assertEquals(zeroMatrix, denseMatrix, 1e-10);
      assertEquals(maxNumberOfRows, denseMatrix.getNumRows());
      assertEquals(maxNumberOfColumns, denseMatrix.getNumCols());

      assertEquals(maxNumberOfRows, yoMatrix.getNumRows());
      assertEquals(maxNumberOfColumns, yoMatrix.getNumCols());

      Random random = new Random(1984L);

      DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(maxNumberOfRows, maxNumberOfColumns, random);
      yoMatrix.set(randomMatrix);

      DMatrixRMaj checkMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(checkMatrix);

      EjmlUnitTests.assertEquals(randomMatrix, checkMatrix, 1e-10);

      DMatrixRMaj smallerMatrix = RandomMatrices_DDRM.rectangle(smallerRows, smallerColumns, random);
      yoMatrix.set(smallerMatrix);

      assertEquals(smallerRows, smallerMatrix.getNumRows());
      assertEquals(smallerColumns, smallerMatrix.getNumCols());

      assertEquals(smallerRows, yoMatrix.getNumRows());
      assertEquals(smallerColumns, yoMatrix.getNumCols());

      DMatrixRMaj checkMatrix2 = new DMatrixRMaj(1, 1);
      yoMatrix.getAndReshape(checkMatrix2);

      EjmlUnitTests.assertEquals(smallerMatrix, checkMatrix2, 1e-10);

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

      DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(maxNumberOfRows, maxNumberOfColumns, random);
      yoMatrix.set(randomMatrix);
      
      int numberOfRows = 2;
      int numberOfColumns = 6;
      yoMatrix.reshape(numberOfRows, numberOfColumns);
      yoMatrix.zero();
      
      DMatrixRMaj zeroMatrix = new DMatrixRMaj(numberOfRows, numberOfColumns);
      zeroMatrix.zero();
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
            YoDouble variable = (YoDouble) registry.findVariable(YoMatrix.getFieldName(name, row, column));

            if ((row < smallerRows) && (column < smallerColumns))
            {
               assertEquals(checkMatrix.get(row, column), variable.getDoubleValue(), 1e-10);
            }
            else
            {
               assertTrue(Double.isNaN(variable.getDoubleValue()), "Values outside aren't NaN, instead are " + variable.getDoubleValue());
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
            YoDouble variable = (YoDouble) registry.findVariable(YoMatrix.getFieldName(name, row, column));
            assertTrue(Double.isNaN(variable.getDoubleValue()));
         }
      }
   }

}
