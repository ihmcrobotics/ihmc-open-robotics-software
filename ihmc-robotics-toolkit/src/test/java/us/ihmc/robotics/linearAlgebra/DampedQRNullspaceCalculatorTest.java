package us.ihmc.robotics.linearAlgebra;

import static us.ihmc.robotics.Assert.assertEquals;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

public class DampedQRNullspaceCalculatorTest extends DampedNullspaceCalculatorTest
{
   @Override
   public DampedNullspaceCalculator getDampedNullspaceProjectorCalculator()
   {
      return new DampedQRNullspaceCalculator(10, 0.0);
   }

   @Override
   public NullspaceCalculator getNullspaceProjectorCalculator()
   {
      return getDampedNullspaceProjectorCalculator();
   }

   /**
    * Computes the inner product of a, assuming a is an upper diagonal matrix
    */

   public static void matrixEquals(DMatrixRMaj expected, DMatrixRMaj actual, double epsilon)
   {
      for (int i = 0; i < expected.getNumElements(); i++)
         assertEquals(expected.get(i), actual.get(i), epsilon);
   }

   private static DMatrixRMaj createMatrix(int size)
   {
      DMatrixRMaj upperDiagonal = new DMatrixRMaj(size, size);
      double value = 1.0;
      for (int i = 0; i < size; i++)
      {
         for (int j = i; j < size; j++)
         {
            upperDiagonal.set(i,j, value);
            value += 1.0;
         }
      }

      return upperDiagonal;
   }

   @Test
   public void testInnerProductMultiply()
   {
      for (int size = 2; size < 20; size++)
      {
         DMatrixRMaj upperDiagonal = createMatrix(size);

         DMatrixRMaj expected = new DMatrixRMaj(size, size);
         CommonOps_DDRM.multTransA(upperDiagonal, upperDiagonal, expected);

         DMatrixRMaj actual = new DMatrixRMaj(size, size);
         DampedQRNullspaceCalculator.inner_small_upper_diagonal(upperDiagonal, actual);

         matrixEquals(expected, actual, 1e-7);
      }
   }
}
