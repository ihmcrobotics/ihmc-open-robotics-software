package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import static us.ihmc.robotics.Assert.fail;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Test;

import us.ihmc.matrixlib.MatrixTestTools;

public class SingularValueExplorationAndExamplesTest
{

   @AfterAll
   public static void tearDownAfterClass() throws Exception
   {
   }

	@Test
   public void testSimpleCase()
   {
      DMatrixRMaj matrixJ = new DMatrixRMaj(new double[][]{{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});

      int numRows = matrixJ.getNumRows();
      int numColumns = matrixJ.getNumCols();
      boolean needU = true;
      boolean needV = true;
      boolean compact = false;
      
      SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(numRows, numColumns, needU, needV, compact);
      svd.decompose(matrixJ);
      
      double[] singularValues = svd.getSingularValues();
      DMatrixRMaj matrixU = new DMatrixRMaj(numRows, numRows);
      DMatrixRMaj matrixW = new DMatrixRMaj(numRows, numColumns);
      DMatrixRMaj matrixVTranspose = new DMatrixRMaj(numColumns, numColumns);
      
      boolean transposeU = false;
      boolean transposeV = true;
      svd.getU(matrixU, transposeU);
      svd.getV(matrixVTranspose, transposeV);
      svd.getW(matrixW);
      
      System.out.println("matrixJ = " + matrixJ);
      System.out.println("matrixU = " + matrixU);
      System.out.println("matrixW = " + matrixW);
      System.out.println("matrixV = " + matrixVTranspose);
      
      DMatrixRMaj matrixJReconstructed = reconstructMatrix(matrixU, matrixW, matrixVTranspose);
      System.out.println("matrixJReconstructed = " + matrixJReconstructed);

      MatrixTestTools.assertMatrixEquals(matrixJ, matrixJReconstructed, 1e-7);
   }

	@Test
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
      
      DMatrixRMaj matrixJ = new DMatrixRMaj(entries);
      
      int numRows = matrixJ.getNumRows();
      int numColumns = matrixJ.getNumCols();
      boolean needU = true;
      boolean needV = true;
      boolean compact = false;
      
      SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(numRows, numColumns, needU, needV, compact);
      svd.decompose(matrixJ);
      
      double[] singularValues = svd.getSingularValues();
      DMatrixRMaj matrixU = new DMatrixRMaj(numRows, numRows);
      DMatrixRMaj matrixW = new DMatrixRMaj(numRows, numColumns);
      DMatrixRMaj matrixVTranspose = new DMatrixRMaj(numColumns, numColumns);
      
      boolean transposeU = false;
      boolean transposeV = true;
      svd.getU(matrixU, transposeU);
      svd.getV(matrixVTranspose, transposeV);
      svd.getW(matrixW);
      
      System.out.println("matrixJ = " + matrixJ);
      System.out.println("matrixU = " + matrixU);
      System.out.println("matrixW = " + matrixW);
      System.out.println("matrixVTranspose = " + matrixVTranspose);
      
      DMatrixRMaj matrixJReconstructed = reconstructMatrix(matrixU, matrixW, matrixVTranspose);
      System.out.println("matrixJReconstructed = " + matrixJReconstructed);

      MatrixTestTools.assertMatrixEquals(matrixJ, matrixJReconstructed, 1e-7);
      
      // Extract Nullspace:

      DMatrixRMaj matrixNTranspose = new DMatrixRMaj(1, matrixVTranspose.getNumCols());
      CommonOps_DDRM.extract(matrixVTranspose, matrixVTranspose.getNumRows()-1, matrixVTranspose.getNumRows(), 0, matrixVTranspose.getNumCols(), matrixNTranspose, 0, 0);
      
      System.out.println("matrixNTranspose = " + matrixNTranspose);

      DMatrixRMaj identity = CommonOps_DDRM.identity(matrixNTranspose.getNumCols());
      DMatrixRMaj matrixNNTranspose = new DMatrixRMaj(matrixNTranspose.getNumCols(), matrixNTranspose.getNumCols());
      CommonOps_DDRM.multInner(matrixNTranspose, matrixNNTranspose);
      
      System.out.println("matrixNNTranspose = " + matrixNNTranspose);

      DMatrixRMaj iMinusNNTranspose = new DMatrixRMaj(matrixNNTranspose.getNumRows(), matrixNNTranspose.getNumCols());
      CommonOps_DDRM.subtract(identity, matrixNNTranspose, iMinusNNTranspose);
      
      System.out.println("iMinusNNTranspose = " + iMinusNNTranspose);

      DMatrixRMaj matrixJPrime = new DMatrixRMaj(matrixJ.getNumRows(), matrixJ.getNumCols());

      try
      {
         CommonOps_DDRM.mult(iMinusNNTranspose, matrixJ, matrixJPrime);
         System.out.println("matrixJPrime = " + matrixJPrime);
         fail("Dimensions don't even match!");
      }
      catch(Exception e)
      {
         
      }

   }

   private DMatrixRMaj reconstructMatrix(DMatrixRMaj matrixU, DMatrixRMaj matrixW, DMatrixRMaj matrixV)
   {
      DMatrixRMaj temp = new DMatrixRMaj(matrixW.getNumRows(), matrixW.getNumCols());
      DMatrixRMaj ret = new DMatrixRMaj(matrixW.getNumRows(), matrixW.getNumCols());
      
      CommonOps_DDRM.mult(matrixU, matrixW, temp);
      CommonOps_DDRM.mult(temp, matrixV, ret);
      
      return ret;
   }

	@Test
   public void foo2()
   {
      
   // To make this not singular, get rid of row 1 (0.001) 2 (0.001) 3 (0.554) 4 (0.973) or 5 (0.463)  NOT: 6, 7
         double[][] entries = new double[][]{
               {1.000, 0.001}};
         
         DMatrixRMaj matrixJ = new DMatrixRMaj(entries);
         
         int numRows = matrixJ.getNumRows();
         int numColumns = matrixJ.getNumCols();
         boolean needU = true;
         boolean needV = true;
         boolean compact = false;
         
         SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(numRows, numColumns, needU, needV, compact);
         svd.decompose(matrixJ);
         
         double[] singularValues = svd.getSingularValues();
         DMatrixRMaj matrixU = new DMatrixRMaj(numRows, numRows);
         DMatrixRMaj matrixW = new DMatrixRMaj(numRows, numColumns);
         DMatrixRMaj matrixVTranspose = new DMatrixRMaj(numColumns, numColumns);
         
         boolean transposeU = false;
         boolean transposeV = true;
         svd.getU(matrixU, transposeU);
         svd.getV(matrixVTranspose, transposeV);
         svd.getW(matrixW);
         
         System.out.println("matrixJ = " + matrixJ);
         System.out.println("matrixU = " + matrixU);
         System.out.println("matrixW = " + matrixW);
         System.out.println("matrixV = " + matrixVTranspose);
         
         DMatrixRMaj matrixJReconstructed = reconstructMatrix(matrixU, matrixW, matrixVTranspose);
         System.out.println("matrixJReconstructed = " + matrixJReconstructed);

         MatrixTestTools.assertMatrixEquals(matrixJ, matrixJReconstructed, 1e-7);
         
         DMatrixRMaj matrixJTranspose = new DMatrixRMaj(matrixJ);
         CommonOps_DDRM.transpose(matrixJTranspose);
         
         DMatrixRMaj matrixJJTranspose = new DMatrixRMaj(matrixJ.getNumRows(), matrixJ.getNumRows());
         CommonOps_DDRM.mult(matrixJ, matrixJTranspose, matrixJJTranspose);
         
         DMatrixRMaj matrixJJTransposeInverse = new DMatrixRMaj(matrixJJTranspose);
         CommonOps_DDRM.invert(matrixJJTransposeInverse);
         

         DMatrixRMaj jPlus = new DMatrixRMaj(matrixJ.getNumCols(), matrixJ.getNumRows());
         CommonOps_DDRM.mult(matrixJTranspose, matrixJJTransposeInverse, jPlus);

         System.out.println("jPlus = " + jPlus);

         
         DMatrixRMaj jPlusJ = new DMatrixRMaj(jPlus.getNumRows(), matrixJ.getNumCols());
         CommonOps_DDRM.mult(jPlus, matrixJ, jPlusJ);
         
         System.out.println("jPlusJ = " + jPlusJ);
         
         DMatrixRMaj identity = CommonOps_DDRM.identity(jPlusJ.getNumRows());
         DMatrixRMaj matrixQ = new DMatrixRMaj(identity.getNumRows(), identity.getNumCols());
         
         CommonOps_DDRM.subtract(identity, jPlusJ, matrixQ);
         System.out.println("matrixQ = " + matrixQ);


   }

}
