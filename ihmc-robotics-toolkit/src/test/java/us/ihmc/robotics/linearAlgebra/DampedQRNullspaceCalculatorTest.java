package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;

import static org.junit.Assert.assertEquals;

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

   public static void matrixEquals(DenseMatrix64F expected, DenseMatrix64F actual, double epsilon)
   {
      for (int i = 0; i < expected.getNumElements(); i++)
         assertEquals(expected.get(i), actual.get(i), epsilon);
   }

   private static DenseMatrix64F createMatrix(int size)
   {
      DenseMatrix64F upperDiagonal = new DenseMatrix64F(size, size);
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

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 100000000)
   public void testInnerProductMultiply()
   {
      for (int size = 2; size < 20; size++)
      {
         DenseMatrix64F upperDiagonal = createMatrix(size);

         DenseMatrix64F expected = new DenseMatrix64F(size, size);
         CommonOps.multTransA(upperDiagonal, upperDiagonal, expected);

         DenseMatrix64F actual = new DenseMatrix64F(size, size);
         DampedQRNullspaceCalculator.inner_small_upper_diagonal(upperDiagonal, actual);

         matrixEquals(expected, actual, 1e-7);
      }
   }
}
