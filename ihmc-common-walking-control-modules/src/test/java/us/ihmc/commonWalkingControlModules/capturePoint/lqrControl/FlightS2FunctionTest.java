package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;

import java.util.Random;

public class FlightS2FunctionTest
{
   @Test
   public void testSimpleCase()
   {
      Random random = new Random(1738L);
      DMatrixRMaj S1AtEnd = new DMatrixRMaj(6, 6);
      DMatrixRMaj s2AtEnd = new DMatrixRMaj(6, 1);
      S1AtEnd.setData(RandomNumbers.nextDoubleArray(random, 36, 10.0));
      s2AtEnd.setData(RandomNumbers.nextDoubleArray(random, 6, 10.0));

      FlightS2Function flightS2Function = new FlightS2Function(-9.81);
      flightS2Function.set(S1AtEnd, s2AtEnd, 1.0);

      DMatrixRMaj s2 = new DMatrixRMaj(6, 1);
      flightS2Function.compute(1.0, s2);

      MatrixTestTools.assertMatrixEquals(s2AtEnd, s2, 1e-8);
   }
}
