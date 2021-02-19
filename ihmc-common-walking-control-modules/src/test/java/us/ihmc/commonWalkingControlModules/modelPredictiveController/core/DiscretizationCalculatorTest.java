package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;

import java.util.Random;

public abstract class DiscretizationCalculatorTest
{
   public abstract DiscretizationCalculator getCalculator();

   @Test
   public void testEquivalent()
   {
      Random random = new Random(1738L);
      DiscretizationCalculator calculator = getCalculator();

      for (int i = 0; i < 500; i++)
      {
         int rows = RandomNumbers.nextInt(random, 1, 20);
         int Bcols = RandomNumbers.nextInt(random, 1, 100);
         int Ccols = RandomNumbers.nextInt(random, 1, 20);
         int size = rows + Bcols + Ccols;

         double tickDuration = RandomNumbers.nextDouble(random, 0.001, 1.0);

         DMatrixRMaj subA = new DMatrixRMaj(rows, rows);
         DMatrixRMaj subB = new DMatrixRMaj(rows, Bcols);
         DMatrixRMaj subC = new DMatrixRMaj(rows, Ccols);

         DMatrixRMaj Ad = new DMatrixRMaj(rows, rows);
         DMatrixRMaj Bd = new DMatrixRMaj(rows, Bcols);
         DMatrixRMaj Cd = new DMatrixRMaj(rows, Ccols);

         DMatrixRMaj AdExpected = new DMatrixRMaj(rows, rows);
         DMatrixRMaj BdExpected = new DMatrixRMaj(rows, Bcols);
         DMatrixRMaj CdExpected = new DMatrixRMaj(rows, Ccols);


         subA.setData(RandomNumbers.nextDoubleArray(random, rows * rows, 1.0));
         subB.setData(RandomNumbers.nextDoubleArray(random, rows * Bcols, 1.0));
         subC.setData(RandomNumbers.nextDoubleArray(random, rows * Ccols, 1.0));

         CommonOps_DDRM.scale(tickDuration, subA, AdExpected);
         CommonOps_DDRM.scale(tickDuration, subB, BdExpected);
         CommonOps_DDRM.scale(tickDuration, subC, CdExpected);
         MatrixTools.addDiagonal(AdExpected, 1.0);


         calculator.compute(subA, subB, subC, Ad, Bd, Cd, tickDuration);

         MatrixTestTools.assertMatrixEquals(CdExpected, Cd, 1e-3);
         MatrixTestTools.assertMatrixEquals(BdExpected, Bd, 1e-3);
         MatrixTestTools.assertMatrixEquals(AdExpected, Ad, 1e-3);
      }
   }
}
