package us.ihmc.simulationconstructionset.mathfunctions;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import Jama.Matrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class MatrixTest
{
   private static final boolean VERBOSE = false;
   
   int rowDimension = 40;
   int columnDimension = 30;

   Matrix matrix;
   Matrix matrixInverted;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testInvert()
   {
      if (VERBOSE)
      {
         System.out.println("");
         System.out.println("Matrix Inversion test");
      }

      fillMatrix(rowDimension, rowDimension);
      if (VERBOSE) displayMatrix(matrix, "matrix");

      Matrix matrixMultiplicationResult = new Matrix(rowDimension, rowDimension);

      // Invert Matrix
      matrixInverted = matrix.inverse();    // with jama
      if (VERBOSE) displayMatrix(matrixInverted, "matrixInverted");

      // Matrix * Matrix inverted * Matrix should be equal to Matrix
      matrixMultiplicationResult = matrix.times(matrixInverted).times(matrix);    // with jama
      if (VERBOSE) displayMatrix(matrixMultiplicationResult, "matrixMultiplicationResult");

      boolean numberAreCloseEnough = false;

      for (int i = 0; i < matrixMultiplicationResult.getRowDimension(); i++)
      {
         for (int j = 0; j < matrixMultiplicationResult.getColumnDimension(); j++)
         {
            // We lost precision during the multiplication so result can not be exactly the same
            if ((matrixMultiplicationResult.get(i, j) * 1.001 > matrix.get(i, j)) && (matrixMultiplicationResult.get(i, j) * 0.999 < matrix.get(i, j)))
               numberAreCloseEnough = true;
            else
               numberAreCloseEnough = false;

            assertFalse("Error at i = " + i + " j = " + j, !numberAreCloseEnough);
         }
      }

      assertTrue("Matrix Inversion works well", true);
   }

   private void fillMatrix(int row, int column)
   {
      matrix = new Matrix(row, column);
      matrixInverted = new Matrix(column, row);
      Random generator = new Random(4876L);
      for (int i = 0; i < matrix.getRowDimension(); i++)
      {
         for (int j = 0; j < matrix.getColumnDimension(); j++)
         {
            matrix.set(i, j, generator.nextDouble());
         }
      }
   }

   private void displayMatrix(Matrix matrix, String matrixName)
   {
      System.out.println("");
      System.out.println(matrixName);

      for (int i = 0; i < matrix.getRowDimension(); i++)
      {
         System.out.println("");

         for (int j = 0; j < matrix.getColumnDimension(); j++)
         {
            System.out.print(matrix.get(i, j) + " ");
         }
      }
   }

}
