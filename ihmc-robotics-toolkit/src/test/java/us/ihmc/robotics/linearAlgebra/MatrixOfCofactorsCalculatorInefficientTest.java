package us.ihmc.robotics.linearAlgebra;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.matrixlib.MatrixTestTools;

public class MatrixOfCofactorsCalculatorInefficientTest
{

	@Test
   public void test()
   {
      int n = 5;
      
      DMatrixRMaj mat = RandomMatrices_DDRM.rectangle(n, n, new Random());
      DMatrixRMaj inverse = new DMatrixRMaj(n, n);
      CommonOps_DDRM.invert(mat, inverse);
      
      DMatrixRMaj matrixOfCoFactors = MatrixOfCofactorsCalculatorInefficient.computeMatrixOfCoFactors(mat);
      DMatrixRMaj inverseViaMatrixOfCoFactors = new DMatrixRMaj(matrixOfCoFactors.getNumCols(), matrixOfCoFactors.getNumRows());
      CommonOps_DDRM.transpose(matrixOfCoFactors, inverseViaMatrixOfCoFactors);
      
      CommonOps_DDRM.scale(1.0/CommonOps_DDRM.det(mat), inverseViaMatrixOfCoFactors);

      MatrixTestTools.assertMatrixEquals(inverse, inverseViaMatrixOfCoFactors, 1e-5);
   }

}
