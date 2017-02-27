package us.ihmc.robotics.math.frames;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.testing.JUnitTools;

public class YoMatrixTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleYoMatrixExample()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix("testMatrix", maxNumberOfRows, maxNumberOfColumns, registry);
      assertEquals(maxNumberOfRows, yoMatrix.getNumberOfRows());
      assertEquals(maxNumberOfColumns, yoMatrix.getNumberOfColumns());

      DenseMatrix64F denseMatrix = new DenseMatrix64F(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(denseMatrix);

      JUnitTools.assertMatrixEqualsZero(denseMatrix, 1e-10);

      Random random = new Random(1984L);

      DenseMatrix64F randomMatrix = RandomTools.generateRandomMatrix(random, maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.set(randomMatrix);

      DenseMatrix64F checkMatrix = new DenseMatrix64F(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(checkMatrix);

      JUnitTools.assertMatrixEquals(randomMatrix, checkMatrix, 1e-10);

      assertEquals(registry.getVariable("testMatrix_0_0").getValueAsDouble(), checkMatrix.get(0, 0), 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testYoMatrixDimensioning()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      String name = "testMatrix";
      
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix(name, maxNumberOfRows, maxNumberOfColumns, registry);

      int smallerRows = maxNumberOfRows - 2;
      int smallerColumns = maxNumberOfColumns - 3;
      DenseMatrix64F denseMatrix = new DenseMatrix64F(smallerRows, smallerColumns);

      try
      {
         yoMatrix.get(denseMatrix);
         fail("Should throw an exception if the size isn't right!");
      }
      catch (Exception e)
      {
      }

      yoMatrix.getAndReshape(denseMatrix);
      JUnitTools.assertMatrixEqualsZero(denseMatrix, 1e-10);
      assertEquals(maxNumberOfRows, denseMatrix.getNumRows());
      assertEquals(maxNumberOfColumns, denseMatrix.getNumCols());

      assertEquals(maxNumberOfRows, yoMatrix.getNumberOfRows());
      assertEquals(maxNumberOfColumns, yoMatrix.getNumberOfColumns());

      Random random = new Random(1984L);

      DenseMatrix64F randomMatrix = RandomTools.generateRandomMatrix(random, maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.set(randomMatrix);

      DenseMatrix64F checkMatrix = new DenseMatrix64F(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(checkMatrix);

      JUnitTools.assertMatrixEquals(randomMatrix, checkMatrix, 1e-10);

      DenseMatrix64F smallerMatrix = RandomTools.generateRandomMatrix(random, smallerRows, smallerColumns);
      yoMatrix.set(smallerMatrix);

      assertEquals(smallerRows, smallerMatrix.getNumRows());
      assertEquals(smallerColumns, smallerMatrix.getNumCols());

      assertEquals(smallerRows, yoMatrix.getNumberOfRows());
      assertEquals(smallerColumns, yoMatrix.getNumberOfColumns());

      DenseMatrix64F checkMatrix2 = new DenseMatrix64F(1, 1);
      yoMatrix.getAndReshape(checkMatrix2);

      JUnitTools.assertMatrixEquals(smallerMatrix, checkMatrix2, 1e-10);

      checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, checkMatrix2, registry);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testYoMatrixSetToZero()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      String name = "testMatrixForZero";
      
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix(name, maxNumberOfRows, maxNumberOfColumns, registry);
      
      Random random = new Random(1984L);

      DenseMatrix64F randomMatrix = RandomTools.generateRandomMatrix(random, maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.set(randomMatrix);
      
      int numberOfRows = 2;
      int numberOfColumns = 6;
      yoMatrix.setToZero(numberOfRows, numberOfColumns);
      
      DenseMatrix64F zeroMatrix = new DenseMatrix64F(numberOfRows, numberOfColumns);
      checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, zeroMatrix, registry);  
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testYoMatrixSetTooBig()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      String name = "testMatrix";
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix(name, maxNumberOfRows, maxNumberOfColumns, registry);

      DenseMatrix64F tooBigMatrix = new DenseMatrix64F(maxNumberOfRows + 1, maxNumberOfColumns);

      try
      {
         yoMatrix.set(tooBigMatrix);
         fail("Too Big");
      }
      catch (RuntimeException e)
      {
      }

      tooBigMatrix = new DenseMatrix64F(maxNumberOfRows, maxNumberOfColumns + 1);

      try
      {
         yoMatrix.set(tooBigMatrix);
         fail("Too Big");
      }
      catch (RuntimeException e)
      {
      }

      // Test a 0 X Big Matrix
      DenseMatrix64F okMatrix = new DenseMatrix64F(0, maxNumberOfColumns + 10);
      yoMatrix.set(okMatrix);
      assertMatrixYoVariablesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, registry);
      
      DenseMatrix64F checkMatrix = new DenseMatrix64F(1, 1);
      yoMatrix.getAndReshape(checkMatrix);
      
      assertEquals(0, checkMatrix.getNumRows());
      assertEquals(maxNumberOfColumns + 10, checkMatrix.getNumCols());
      
      // Test a Big X 0 Matrix

      okMatrix = new DenseMatrix64F(maxNumberOfRows + 10, 0);
      yoMatrix.set(okMatrix);
      assertMatrixYoVariablesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, registry);
      
      checkMatrix = new DenseMatrix64F(1, 1);
      yoMatrix.getAndReshape(checkMatrix);
      
      assertEquals(maxNumberOfRows + 10, checkMatrix.getNumRows());
      assertEquals(0, checkMatrix.getNumCols());
      
   }


   private void checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(String name, int maxNumberOfRows, int maxNumberOfColumns, DenseMatrix64F checkMatrix, YoVariableRegistry registry)
   {
      int smallerRows = checkMatrix.getNumRows();
      int smallerColumns = checkMatrix.getNumCols();
      
   // Make sure the values are correct, including values outside the range should be NaN:
      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            DoubleYoVariable variable = (DoubleYoVariable) registry.getVariable(name + "_" + row + "_" + column);

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
   
   private void assertMatrixYoVariablesAreNaN(String name, int maxNumberOfRows, int maxNumberOfColumns, YoVariableRegistry registry)
   {
      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            DoubleYoVariable variable = (DoubleYoVariable) registry.getVariable(name + "_" + row + "_" + column);
            assertTrue(Double.isNaN(variable.getDoubleValue()));
         }
      }
   }

}
