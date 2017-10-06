package us.ihmc.robotics.linearAlgebra;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * @author twan
 * Date: 4/11/13
 */
public class NullspaceCalculatorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRemoveNullspaceComponent()
   {
      int matrixSize = 10;
      boolean makeLargestComponentPositive = true;
      NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(matrixSize, makeLargestComponentPositive);

      Random random = new Random();
      double[] singularValues = RandomNumbers.nextDoubleArray(random, matrixSize, 1.0, 2.0);

//    singularValues[0] = 0.0;
      DenseMatrix64F matrix = RandomMatrices.createSingularValues(matrixSize, matrixSize, random, singularValues);
      nullspaceCalculator.setMatrix(matrix, 1);
      DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();

      DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);
      nullspaceCalculator.removeNullspaceComponent(matrixCopy);

      assertTrue(isNullspaceComponentZero(matrixCopy, nullspace));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRemoveNullspaceComponent2()
   {
      int matrixSize = 6;
      boolean makeLargestComponentPositive = true;
      NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(matrixSize, makeLargestComponentPositive);

      Random random = new Random();
      double[] singularValues = RandomNumbers.nextDoubleArray(random, matrixSize, 1.0, 2.0);
      singularValues[matrixSize / 2] = 0.0;
      DenseMatrix64F matrix1 = RandomMatrices.createSingularValues(matrixSize, matrixSize, random, singularValues);
      nullspaceCalculator.setMatrix(matrix1, 1);
      DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();

      DenseMatrix64F matrix2 = RandomMatrices.createRandom(matrixSize, matrixSize, random);
      nullspaceCalculator.removeNullspaceComponent(matrix2);

      assertTrue(isNullspaceComponentZero(matrix2, nullspace));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRemoveNullspaceComponentProblematicCase()
   {
      double[] array = new double[]
      {
         -0.025615105184150402, 0.9996675004299076, 0.0, 0.0, 0.0, 1.0, -0.014943128023036597, 0.0025766266962912077, 0.994994904861614, 0.994994904861614,
         0.994994904861614, 0.0, 0.999560187938321, 0.02565637501847126, -0.09992566887155414, -0.09992566887155414, -0.09992566887155414, 0.0,
         0.09097146270512581, -1.7263398865150674E-4, -0.8020248363106273, -0.42534773269796133, -0.06666465862572812, 0.0, -0.14355859699862836,
         0.8437205015326914, 0.017305680726114284, 0.019518441231078962, -0.0026979930595319615, 0.06700000000000006, 1.8511050602308394E-4,
         -0.07800697405973986, 0.17231872793145048, 0.19435195976249223, -0.026864862431263573, 0.0
      };

      DenseMatrix64F matrix = DenseMatrix64F.wrap(6, 6, array);
      int matrixSize = matrix.getNumCols();
      boolean makeLargestComponentPositive = true;
      NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(matrixSize, makeLargestComponentPositive);
      nullspaceCalculator.setMatrix(matrix, 1);
      DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();

      double[] otherArray = new double[] {0.00246580738404988,0.10002072505447161,0.9945001472400798,0.0031310670129175676,-0.03272836951791036,0.013266977355589604,-0.0799682878842701,0.017947029012374344,0.1785478182378059,2.451030346550186E-4,1.2580020435047565,-0.125654221458273,-0.0023320638677076684,-0.22293285167952487,0.11210325702085407,-1.2227732419985802,0.0314067395046658,0.34283533772447394,5.308713273090554E-4,0.30992163367400194,0.009505290863762235,-0.05548471542070279,-5.301903531607067E-4,0.013997484370442227,0.002119726640328553,0.9169404167718533,-0.10706886430962542,1.273221672566972,-0.035158584156337144,-0.36049933068373563,1.0770237098714068,-0.015298793070857523,-0.15257197980669274,0.0015768159883589102,-1.2541201073200425,0.1318372721623865};
      DenseMatrix64F otherMatrix = DenseMatrix64F.wrap(6, 6, otherArray);

      nullspaceCalculator.removeNullspaceComponent(otherMatrix);

      assertTrue(isNullspaceComponentZero(otherMatrix, nullspace));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNullspace()
   {
      int matrixSize = 10;
      boolean makeLargestComponentPositive = true;
      NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(matrixSize, makeLargestComponentPositive);

      Random random = new Random();
      double[] singularValues = RandomNumbers.nextDoubleArray(random, matrixSize, 1.0, 2.0);
      singularValues[0] = 0.0;
      DenseMatrix64F matrix = RandomMatrices.createSingularValues(matrixSize, matrixSize, random, singularValues);
      nullspaceCalculator.setMatrix(matrix, 1);
      DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();

      DenseMatrix64F matrixTimesNullspace = new DenseMatrix64F(matrix.getNumRows(), nullspace.getNumCols());
      CommonOps.mult(matrix, nullspace, matrixTimesNullspace);

      assertTrue(MatrixFeatures.isConstantVal(matrixTimesNullspace, 0.0, 1e-12));
   }

   private boolean isNullspaceComponentZero(DenseMatrix64F matrixToTest, DenseMatrix64F nullspace)
   {
      DenseMatrix64F nullspaceCheck = new DenseMatrix64F(nullspace.getNumCols(), matrixToTest.getNumCols());
      CommonOps.multTransA(nullspace, matrixToTest, nullspaceCheck);
      boolean nullspaceComponentZero = MatrixFeatures.isConstantVal(nullspaceCheck, 0.0, 1e-7);
      return nullspaceComponentZero;
   }
}
