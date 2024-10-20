package us.ihmc.math;

import Jama.Matrix;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class ComplexMatrixTest
{
   private final double[][][] realElements = new double[][][] {{{1.0, 0.0}, {2.0, 0.0}}, {{3.0, 0.0}, {4.0, 0.0}}};

   private final double[][][] singleElements = new double[][][] {{{1.0, 1.0}}};

   private final double[][][] complexElements = new double[][][] {{{1.0, 7.0}, {2.0, 9.0}}, {{3.0, 11.0}, {4.0, 15.0}}};

   private final double[][][] threeByFourElements = new double[][][] {{{1.0, 7.0}, {2.0, 9.0}, {2.0, 9.0}, {-0.88, 1.9}},
                                                                      {{3.0, 11.0}, {4.0, 5.0}, {2.0, 9.0}, {1.1, 1.9}},
                                                                      {{3.0, 19.0}, {4.0, 15.0}, {2.0, 39.0}, {1.1, 1.9}},};

   private ComplexMatrix realExample, singleComplexNumber, complexExample, threeByFour, identityOne, identityFour;
   private ComplexMatrix[] allExamples;

   @BeforeEach
   public void setUp()
   {
      realExample = new ComplexMatrix(realElements);
      singleComplexNumber = new ComplexMatrix(singleElements);
      complexExample = new ComplexMatrix(complexElements);
      threeByFour = new ComplexMatrix(threeByFourElements);

      identityOne = ComplexMatrix.constructIdentity(1);
      identityFour = ComplexMatrix.constructIdentity(4);

      allExamples = new ComplexMatrix[] {realExample, singleComplexNumber, complexExample, threeByFour, identityOne, identityFour};
   }

   @AfterEach
   public void tearDown()
   {
      realExample = null;
      singleComplexNumber = null;
      complexExample = null;
      threeByFour = null;

      identityOne = null;
      identityFour = null;

      allExamples = null;
   }

   @Test
   public void testIdentity()
   {
      assertTrue(identityOne.epsilonEquals(new Matrix(new double[][] {{1.0}}), 1e-7));
   }

   @Test
   public void testGetRowAndColumnDimensions()
   {
      assertEquals(2, realExample.getRowDimension());
      assertEquals(2, realExample.getColumnDimension());

      assertEquals(1, singleComplexNumber.getRowDimension());
      assertEquals(1, singleComplexNumber.getColumnDimension());

      assertEquals(2, complexExample.getRowDimension());
      assertEquals(2, complexExample.getColumnDimension());

      assertEquals(1, identityOne.getRowDimension());
      assertEquals(1, identityOne.getColumnDimension());

      assertEquals(4, identityFour.getRowDimension());
      assertEquals(4, identityFour.getColumnDimension());

      assertEquals(3, threeByFour.getRowDimension());
      assertEquals(4, threeByFour.getColumnDimension());
   }

   @Test
   public void testEpsilonEquals()
   {
      for (int i = 0; i < allExamples.length; i++)
      {
         for (int j = 0; j < allExamples.length; j++)
         {
            if (i == j)
               assertTrue(allExamples[i].epsilonEquals(allExamples[j], 1e-7));
            else
               assertFalse(allExamples[i].epsilonEquals(allExamples[j], 1e-7));
         }
      }
   }

   @Test
   public void testConstructDiagonalMatrix()
   {
      ComplexNumber[] diagonalElements = new ComplexNumber[] {new ComplexNumber(1.0, 2.0), new ComplexNumber(3.0, 4.0)};

      ComplexMatrix matrix = ComplexMatrix.constructDiagonalMatrix(diagonalElements);

      int numRows = matrix.getRowDimension();
      int numColumns = matrix.getColumnDimension();

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            if (i == j)
               assertTrue(diagonalElements[i].epsilonEquals(matrix.get(i, j), 1e-7));
            else
               assertTrue(matrix.get(i, j).epsilonEquals(0.0, 1e-7));
         }
      }
   }

   @Test
   public void testTranspose()
   {
      ComplexMatrix fourByThree = threeByFour.transpose();
      assertEquals(4, fourByThree.getRowDimension());
      assertEquals(3, fourByThree.getColumnDimension());

      int numRows = threeByFour.getRowDimension();
      int numColumns = threeByFour.getColumnDimension();

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            assertFalse(threeByFour.get(i, j) == (fourByThree.get(j, i)));
            assertTrue(threeByFour.get(i, j).epsilonEquals(fourByThree.get(j, i), 0.0));
         }
      }
   }

   @Test
   public void testTimes()
   {
      double timesByReal = 7.11;
      ComplexNumber timesByComplex = new ComplexNumber(8.23, 11.1);

      verifyTimes(threeByFour, new ComplexNumber(timesByReal, 0.0), threeByFour.times(timesByReal));
      verifyTimes(threeByFour, timesByComplex, threeByFour.times(timesByComplex));
   }

   private void verifyTimes(ComplexMatrix original, ComplexNumber timesBy, ComplexMatrix result)
   {
      assertEquals(original.getRowDimension(), result.getRowDimension());
      assertEquals(original.getColumnDimension(), result.getColumnDimension());

      for (int i = 0; i < result.getRowDimension(); i++)
      {
         for (int j = 0; j < result.getColumnDimension(); j++)
         {
            assertTrue(original.get(i, j).times(timesBy).epsilonEquals(result.get(i, j), 1e-7));
         }
      }
   }

   @Test
   public void testMatrixTimes()
   {
      ComplexMatrix fourByThree = threeByFour.transpose();
      ComplexMatrix result = threeByFour.times(fourByThree);

      assertEquals(3, result.getRowDimension());
      assertEquals(3, result.getColumnDimension());

      // Just verify manually here:
      ComplexNumber result00 = threeByFour.get(0, 0)
                                          .times(fourByThree.get(0, 0))
                                          .plus(threeByFour.get(0, 1).times(fourByThree.get(1, 0)))
                                          .plus(threeByFour.get(0, 2).times(fourByThree.get(2, 0)))
                                          .plus(threeByFour.get(0, 3).times(fourByThree.get(3, 0)));

      ComplexNumber result12 = threeByFour.get(1, 0)
                                          .times(fourByThree.get(0, 2))
                                          .plus(threeByFour.get(1, 1).times(fourByThree.get(1, 2)))
                                          .plus(threeByFour.get(1, 2).times(fourByThree.get(2, 2)))
                                          .plus(threeByFour.get(1, 3).times(fourByThree.get(3, 2)));

      assertTrue(result00.epsilonEquals(result.get(0, 0), 1e-7));
      assertTrue(result12.epsilonEquals(result.get(1, 2), 1e-7));
   }

   @Test
   public void testInverse()
   {
      ComplexMatrix fourByThree = threeByFour.transpose();
      ComplexMatrix threeByThree = threeByFour.times(fourByThree);

      ComplexMatrix threeByThreeInverse = threeByThree.inverse();
      ComplexMatrix identityOne = threeByThree.times(threeByThreeInverse);
      ComplexMatrix identityTwo = threeByThreeInverse.times(threeByThree);

      ComplexMatrix identity = ComplexMatrix.constructIdentity(3);

      assertTrue(identity.epsilonEquals(identityOne, 1e-7));
      assertTrue(identity.epsilonEquals(identityTwo, 1e-7));
   }
}
