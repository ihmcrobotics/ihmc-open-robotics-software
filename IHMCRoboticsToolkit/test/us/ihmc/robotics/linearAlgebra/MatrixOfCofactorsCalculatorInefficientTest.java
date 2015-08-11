package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.test.JUnitTools;

import java.util.Random;

public class MatrixOfCofactorsCalculatorInefficientTest
{

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      int n = 5;
      
      DenseMatrix64F mat = RandomMatrices.createRandom(n, n, new Random());
      DenseMatrix64F inverse = new DenseMatrix64F(n, n);
      CommonOps.invert(mat, inverse);
      
      DenseMatrix64F matrixOfCoFactors = MatrixOfCofactorsCalculatorInefficient.computeMatrixOfCoFactors(mat);
      DenseMatrix64F inverseViaMatrixOfCoFactors = new DenseMatrix64F(matrixOfCoFactors.getNumCols(), matrixOfCoFactors.getNumRows());
      CommonOps.transpose(matrixOfCoFactors, inverseViaMatrixOfCoFactors);
      
      CommonOps.scale(1.0/CommonOps.det(mat), inverseViaMatrixOfCoFactors);

      JUnitTools.assertMatrixEquals(inverse, inverseViaMatrixOfCoFactors, 1e-5);
   }

}
