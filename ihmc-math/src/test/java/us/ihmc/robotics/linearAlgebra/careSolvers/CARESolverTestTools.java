package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTools;

import java.util.Random;

public class CARESolverTestTools
{
   public static DMatrixRMaj generateRandomSymmetricPDMatrix(Random random, int size)
   {
      DMatrixRMaj matrix = generateRandomSymmetricMatrix(random, size);
      MatrixTools.addDiagonal(matrix, size);

      return matrix;
   }

   public static DMatrixRMaj generateRandomSymmetricMatrix(Random random, int size)
   {
      DMatrixRMaj a = new DMatrixRMaj(size, size);
      double[] aValues = RandomNumbers.nextDoubleArray(random, size * size, 1.0);
      a.setData(aValues);

      DMatrixRMaj matrix =  new DMatrixRMaj(size, size);
      CommonOps_DDRM.transpose(a, matrix);
      CommonOps_DDRM.addEquals(matrix, a);
      CommonOps_DDRM.scale(0.5, a);

      return matrix;
   }
}
