package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import static org.junit.Assert.fail;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.junit.AfterClass;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.testing.JUnitTools;

public class SingularValueExplorationAndExamplesTest
{

   @AfterClass
   public static void tearDownAfterClass() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleCase()
   {
      DenseMatrix64F matrixJ = new DenseMatrix64F(new double[][]{{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});

      int numRows = matrixJ.getNumRows();
      int numColumns = matrixJ.getNumCols();
      boolean needU = true;
      boolean needV = true;
      boolean compact = false;
      
      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(numRows, numColumns, needU, needV, compact);
      svd.decompose(matrixJ);
      
      double[] singularValues = svd.getSingularValues();
      DenseMatrix64F matrixU = new DenseMatrix64F(numRows, numRows);
      DenseMatrix64F matrixW = new DenseMatrix64F(numRows, numColumns);
      DenseMatrix64F matrixVTranspose = new DenseMatrix64F(numColumns, numColumns);
      
      boolean transposeU = false;
      boolean transposeV = true;
      svd.getU(matrixU, transposeU);
      svd.getV(matrixVTranspose, transposeV);
      svd.getW(matrixW);
      
      System.out.println("matrixJ = " + matrixJ);
      System.out.println("matrixU = " + matrixU);
      System.out.println("matrixW = " + matrixW);
      System.out.println("matrixV = " + matrixVTranspose);
      
      DenseMatrix64F matrixJReconstructed = reconstructMatrix(matrixU, matrixW, matrixVTranspose);
      System.out.println("matrixJReconstructed = " + matrixJReconstructed);

      JUnitTools.assertMatrixEquals(matrixJ, matrixJReconstructed, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void showIMinusNNTransposeJDoesntMakeSense()
   {
      
// To make this not singular, get rid of row 1 (0.001) 2 (0.001) 3 (0.554) 4 (0.973) or 5 (0.463)  NOT: 6, 7
      double[][] entries = new double[][]{
            {1.000, -0.018, 0.004, -0.000, 0.000, 0.000, 0.000, 0.003, 0.000, 0.000, 1.000, 0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  1.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
            {0.018,  1.000,  0.009, -0.000,  0.000,  0.000,  0.000,  0.009,  0.000,  0.000,  0.000,  0.000,  0.000,  0.996,  0.000,  0.000,  0.000,  0.000,  0.996,  0.000,  0.000,  0.000,  0.996,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
            {-0.137, -0.282,  0.807,  0.351, -0.172, -0.000,  0.000,  0.838,  0.000,  0.000, -0.132,  0.000,  0.000, -0.182,  0.000,  0.000,  0.000,  0.000, -0.054,  0.000,  0.000,  0.000,  0.049,  0.000,  0.000,  0.000, -0.011,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
            {0.238, -0.267,  0.203,  0.339,  0.305,  0.004,  0.000,  0.233,  0.000,  0.000,  0.242,  0.000,  0.000, -0.223,  0.000,  0.000,  0.000,  0.000, -0.099,  0.000,  0.000,  0.000,  0.010,  0.000,  0.000,  0.000,  0.020,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
            {0.667, -0.251, -0.483,  0.327,  0.849,  0.009,  0.000, -0.454,  0.000,  0.000,  0.670,  0.000,  0.000, -0.271,  0.000,  0.000,  0.000,  0.000, -0.150,  0.000,  0.000,  0.000, -0.033,  0.000,  0.000,  0.000,  0.057,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
            {0.159,  0.313,  0.003, -0.004, -0.009,  1.000,  0.000,  0.003,  0.000,  0.000,  0.065,  0.000,  0.000,  0.362,  0.000,  0.000,  0.000,  0.000,  0.265,  0.000,  0.000,  0.000, -0.027,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
            {0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,  0.000, -0.433,  0.000,  0.000,  0.000,  0.000,  0.817,  0.000,  0.000,  0.000, -0.381,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000}
      };
      
      DenseMatrix64F matrixJ = new DenseMatrix64F(entries);
      
      int numRows = matrixJ.getNumRows();
      int numColumns = matrixJ.getNumCols();
      boolean needU = true;
      boolean needV = true;
      boolean compact = false;
      
      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(numRows, numColumns, needU, needV, compact);
      svd.decompose(matrixJ);
      
      double[] singularValues = svd.getSingularValues();
      DenseMatrix64F matrixU = new DenseMatrix64F(numRows, numRows);
      DenseMatrix64F matrixW = new DenseMatrix64F(numRows, numColumns);
      DenseMatrix64F matrixVTranspose = new DenseMatrix64F(numColumns, numColumns);
      
      boolean transposeU = false;
      boolean transposeV = true;
      svd.getU(matrixU, transposeU);
      svd.getV(matrixVTranspose, transposeV);
      svd.getW(matrixW);
      
      System.out.println("matrixJ = " + matrixJ);
      System.out.println("matrixU = " + matrixU);
      System.out.println("matrixW = " + matrixW);
      System.out.println("matrixVTranspose = " + matrixVTranspose);
      
      DenseMatrix64F matrixJReconstructed = reconstructMatrix(matrixU, matrixW, matrixVTranspose);
      System.out.println("matrixJReconstructed = " + matrixJReconstructed);

      JUnitTools.assertMatrixEquals(matrixJ, matrixJReconstructed, 1e-7);
      
      // Extract Nullspace:

      DenseMatrix64F matrixNTranspose = new DenseMatrix64F(1, matrixVTranspose.getNumCols());
      CommonOps.extract(matrixVTranspose, matrixVTranspose.getNumRows()-1, matrixVTranspose.getNumRows(), 0, matrixVTranspose.getNumCols(), matrixNTranspose, 0, 0);
      
      System.out.println("matrixNTranspose = " + matrixNTranspose);

      DenseMatrix64F identity = CommonOps.identity(matrixNTranspose.getNumCols());
      DenseMatrix64F matrixNNTranspose = new DenseMatrix64F(matrixNTranspose.getNumCols(), matrixNTranspose.getNumCols());
      CommonOps.multInner(matrixNTranspose, matrixNNTranspose);
      
      System.out.println("matrixNNTranspose = " + matrixNNTranspose);

      DenseMatrix64F iMinusNNTranspose = new DenseMatrix64F(matrixNNTranspose.getNumRows(), matrixNNTranspose.getNumCols());
      CommonOps.subtract(identity, matrixNNTranspose, iMinusNNTranspose);
      
      System.out.println("iMinusNNTranspose = " + iMinusNNTranspose);

      DenseMatrix64F matrixJPrime = new DenseMatrix64F(matrixJ.getNumRows(), matrixJ.getNumCols());

      try
      {
         CommonOps.mult(iMinusNNTranspose, matrixJ, matrixJPrime);
         System.out.println("matrixJPrime = " + matrixJPrime);
         fail("Dimensions don't even match!");
      }
      catch(Exception e)
      {
         
      }

   }

   private DenseMatrix64F reconstructMatrix(DenseMatrix64F matrixU, DenseMatrix64F matrixW, DenseMatrix64F matrixV)
   {
      DenseMatrix64F temp = new DenseMatrix64F(matrixW.getNumRows(), matrixW.getNumCols());
      DenseMatrix64F ret = new DenseMatrix64F(matrixW.getNumRows(), matrixW.getNumCols());
      
      CommonOps.mult(matrixU, matrixW, temp);
      CommonOps.mult(temp, matrixV, ret);
      
      return ret;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void foo2()
   {
      
   // To make this not singular, get rid of row 1 (0.001) 2 (0.001) 3 (0.554) 4 (0.973) or 5 (0.463)  NOT: 6, 7
         double[][] entries = new double[][]{
               {1.000, 0.001}};
         
         DenseMatrix64F matrixJ = new DenseMatrix64F(entries);
         
         int numRows = matrixJ.getNumRows();
         int numColumns = matrixJ.getNumCols();
         boolean needU = true;
         boolean needV = true;
         boolean compact = false;
         
         SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(numRows, numColumns, needU, needV, compact);
         svd.decompose(matrixJ);
         
         double[] singularValues = svd.getSingularValues();
         DenseMatrix64F matrixU = new DenseMatrix64F(numRows, numRows);
         DenseMatrix64F matrixW = new DenseMatrix64F(numRows, numColumns);
         DenseMatrix64F matrixVTranspose = new DenseMatrix64F(numColumns, numColumns);
         
         boolean transposeU = false;
         boolean transposeV = true;
         svd.getU(matrixU, transposeU);
         svd.getV(matrixVTranspose, transposeV);
         svd.getW(matrixW);
         
         System.out.println("matrixJ = " + matrixJ);
         System.out.println("matrixU = " + matrixU);
         System.out.println("matrixW = " + matrixW);
         System.out.println("matrixV = " + matrixVTranspose);
         
         DenseMatrix64F matrixJReconstructed = reconstructMatrix(matrixU, matrixW, matrixVTranspose);
         System.out.println("matrixJReconstructed = " + matrixJReconstructed);

         JUnitTools.assertMatrixEquals(matrixJ, matrixJReconstructed, 1e-7);
         
         DenseMatrix64F matrixJTranspose = new DenseMatrix64F(matrixJ);
         CommonOps.transpose(matrixJTranspose);
         
         DenseMatrix64F matrixJJTranspose = new DenseMatrix64F(matrixJ.getNumRows(), matrixJ.getNumRows());
         CommonOps.mult(matrixJ, matrixJTranspose, matrixJJTranspose);
         
         DenseMatrix64F matrixJJTransposeInverse = new DenseMatrix64F(matrixJJTranspose);
         CommonOps.invert(matrixJJTransposeInverse);
         

         DenseMatrix64F jPlus = new DenseMatrix64F(matrixJ.getNumCols(), matrixJ.getNumRows());
         CommonOps.mult(matrixJTranspose, matrixJJTransposeInverse, jPlus);

         System.out.println("jPlus = " + jPlus);

         
         DenseMatrix64F jPlusJ = new DenseMatrix64F(jPlus.getNumRows(), matrixJ.getNumCols());
         CommonOps.mult(jPlus, matrixJ, jPlusJ);
         
         System.out.println("jPlusJ = " + jPlusJ);
         
         DenseMatrix64F identity = CommonOps.identity(jPlusJ.getNumRows());
         DenseMatrix64F matrixQ = new DenseMatrix64F(identity.getNumRows(), identity.getNumCols());
         
         CommonOps.subtract(identity, jPlusJ, matrixQ);
         System.out.println("matrixQ = " + matrixQ);


   }

}
