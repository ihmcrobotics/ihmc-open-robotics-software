package us.ihmc.euclid.tools;

import us.ihmc.euclid.matrix.Matrix3D;

import java.util.Random;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextMatrix3D;

public class EuclidCoreMissingRandomTools
{
   /**
    * Generates a random positive definite matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-minMaxValue, minMaxValue]
    * </p>
    * <p>
    * The approach used here generates a random 3D matrix with values in [{@code -minMaxValue}, {@code minMaxValue}], and then performs A * A<sup>T</sup>,
    * which is guaranteed to result in a symmetric positive semi-definite matrix. We then add diagonal terms to make the matrix positive definite, and finally
    * scale the matrix by a random double that upper bounds the absolute values of the positive definite matrix elements to {@code minMaxValue}.
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxValue the maximum value for each element.
    * @return the random positive definite matrix.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static Matrix3D nextPositiveDefiniteMatrix3D(Random random, double minMaxValue)
   {
      Matrix3D matrix3D = nextMatrix3D(random, minMaxValue);
      matrix3D.multiplyTransposeOther(matrix3D);

      double diagonalDominanceScalar = Math.abs(minMaxValue);
      matrix3D.addM00(diagonalDominanceScalar);
      matrix3D.addM11(diagonalDominanceScalar);
      matrix3D.addM22(diagonalDominanceScalar);

      double scalarToShrinkMatrixWithinBounds = nextDouble(random, 0.0, minMaxValue / matrix3D.maxAbsElement());
      matrix3D.scale(scalarToShrinkMatrixWithinBounds);
      return matrix3D;
   }

   /**
    * Generates a random positive definite matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-1.0; 1.0].
    * </p>
    * <p>
    * The approach used here generates a random 3D matrix with values in [-1.0, 1.0], and then performs A * A<sup>T</sup> which is guaranteed to result in a
    * symmetric positive semi-definite matrix. We then add diagonal terms to make the matrix positive definite, and finally scale the matrix by a random double
    * that upper bounds the absolute values of the positive definite matrix elements to 1.0.
    * </p>
    *
    * @param random the random generator to use.
    * @return the random positive definite matrix.
    */
   public static Matrix3D nextPositiveDefiniteMatrix3D(Random random)
   {
      return nextPositiveDefiniteMatrix3D(random, 1.0);
   }
}
