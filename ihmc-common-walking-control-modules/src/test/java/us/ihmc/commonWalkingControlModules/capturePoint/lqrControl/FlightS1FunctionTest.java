package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;

import java.util.Arrays;
import java.util.Random;

public class FlightS1FunctionTest
{
   @Test
   public void testSimpleCase()
   {
      Random random = new Random(1738L);
      DMatrixRMaj S1AtEnd = new DMatrixRMaj(6, 6);
      S1AtEnd.setData(RandomNumbers.nextDoubleArray(random, 36, 10.0));

      FlightS1Function flightS1Function = new FlightS1Function();
      flightS1Function.set(S1AtEnd, 1.0);

      DMatrixRMaj S1 = new DMatrixRMaj(6, 6);
      flightS1Function.compute(1.0, S1);

      MatrixTestTools.assertMatrixEquals(S1AtEnd, S1, 1e-8);
   }
}
