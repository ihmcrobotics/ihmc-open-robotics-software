package us.ihmc.robotics.math;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * @author twan
 *         Date: 4/26/13
 */
public class MatrixYoVariableConversionToolsTest
{

	@Test
   public void testBackAndForthVector()
   {
      List<YoDouble> yoArray = new ArrayList<YoDouble>();
      int size = 50;
      String prefix = "test";
      YoRegistry registry = new YoRegistry("test");

      DMatrixRMaj matrix = new DMatrixRMaj(size, 1);
      Random random = new Random(1235612L);
      RandomMatrices_DDRM.fillUniform(matrix, random);
      DMatrixRMaj matrixBack = new DMatrixRMaj(size, 1);

      MatrixYoVariableConversionTools.populateYoVariablesVector(yoArray, size, prefix, registry);
      MatrixYoVariableConversionTools.storeInYoVariablesVector(matrix, yoArray);
      MatrixYoVariableConversionTools.getFromYoVariablesVector(matrixBack, yoArray);

      EjmlUnitTests.assertEquals(matrix, matrixBack, 1e-12);
   }

	@Test
   public void testBackAndForthMatrix()
   {
      List<List<YoDouble>> yoArray = new ArrayList<List<YoDouble>>();
      int nRows = 50;
      int nColumns = 60;
      String prefix = "test";
      YoRegistry registry = new YoRegistry("test");

      DMatrixRMaj matrix = new DMatrixRMaj(nRows, nColumns);
      Random random = new Random(1235612L);
      RandomMatrices_DDRM.fillUniform(matrix, random);
      DMatrixRMaj matrixBack = new DMatrixRMaj(nRows, nColumns);

      MatrixYoVariableConversionTools.populateYoVariablesMatrix(yoArray, nRows, nColumns, prefix, registry);
      MatrixYoVariableConversionTools.storeInYoVariablesMatrix(matrix, yoArray);
      MatrixYoVariableConversionTools.getFromYoVariablesMatrix(matrixBack, yoArray);

      EjmlUnitTests.assertEquals(matrix, matrixBack, 1e-12);
   }

	@Test
   public void testBackAndForthSymmetricMatrix()
   {
      List<List<YoDouble>> yoArray = new ArrayList<List<YoDouble>>();
      int size = 50;
      String prefix = "test";
      YoRegistry registry = new YoRegistry("test");

      Random random = new Random(1235612L);
      DMatrixRMaj matrix = RandomMatrices_DDRM.symmetricPosDef(size, random);
      DMatrixRMaj matrixBack = new DMatrixRMaj(size, size);

      MatrixYoVariableConversionTools.populateYoVariablesSymmetricMatrix(yoArray, size, prefix, registry);
      MatrixYoVariableConversionTools.storeInYoVariablesSymmetric(matrix, yoArray);
      MatrixYoVariableConversionTools.getFromYoVariablesSymmetric(matrixBack, yoArray);

      EjmlUnitTests.assertEquals(matrix, matrixBack, 1e-12);
   }
}
