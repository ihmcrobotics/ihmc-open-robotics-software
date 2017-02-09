package us.ihmc.robotics.math;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 * @author twan
 *         Date: 4/26/13
 */
public class MatrixYoVariableConversionToolsTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testBackAndForthVector()
   {
      List<DoubleYoVariable> yoArray = new ArrayList<DoubleYoVariable>();
      int size = 50;
      String prefix = "test";
      YoVariableRegistry registry = new YoVariableRegistry("test");

      DenseMatrix64F matrix = new DenseMatrix64F(size, 1);
      Random random = new Random(1235612L);
      RandomMatrices.setRandom(matrix, random);
      DenseMatrix64F matrixBack = new DenseMatrix64F(size, 1);

      MatrixYoVariableConversionTools.populateYoVariablesVector(yoArray, size, prefix, registry);
      MatrixYoVariableConversionTools.storeInYoVariablesVector(matrix, yoArray);
      MatrixYoVariableConversionTools.getFromYoVariablesVector(matrixBack, yoArray);

      EjmlUnitTests.assertEquals(matrix, matrixBack, 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void testBackAndForthMatrix()
   {
      List<List<DoubleYoVariable>> yoArray = new ArrayList<List<DoubleYoVariable>>();
      int nRows = 50;
      int nColumns = 60;
      String prefix = "test";
      YoVariableRegistry registry = new YoVariableRegistry("test");

      DenseMatrix64F matrix = new DenseMatrix64F(nRows, nColumns);
      Random random = new Random(1235612L);
      RandomMatrices.setRandom(matrix, random);
      DenseMatrix64F matrixBack = new DenseMatrix64F(nRows, nColumns);

      MatrixYoVariableConversionTools.populateYoVariablesMatrix(yoArray, nRows, nColumns, prefix, registry);
      MatrixYoVariableConversionTools.storeInYoVariablesMatrix(matrix, yoArray);
      MatrixYoVariableConversionTools.getFromYoVariablesMatrix(matrixBack, yoArray);

      EjmlUnitTests.assertEquals(matrix, matrixBack, 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testBackAndForthSymmetricMatrix()
   {
      List<List<DoubleYoVariable>> yoArray = new ArrayList<List<DoubleYoVariable>>();
      int size = 50;
      String prefix = "test";
      YoVariableRegistry registry = new YoVariableRegistry("test");

      Random random = new Random(1235612L);
      DenseMatrix64F matrix = RandomMatrices.createSymmPosDef(size, random);
      DenseMatrix64F matrixBack = new DenseMatrix64F(size, size);

      MatrixYoVariableConversionTools.populateYoVariablesSymmetricMatrix(yoArray, size, prefix, registry);
      MatrixYoVariableConversionTools.storeInYoVariablesSymmetric(matrix, yoArray);
      MatrixYoVariableConversionTools.getFromYoVariablesSymmetric(matrixBack, yoArray);

      EjmlUnitTests.assertEquals(matrix, matrixBack, 1e-12);
   }
}
