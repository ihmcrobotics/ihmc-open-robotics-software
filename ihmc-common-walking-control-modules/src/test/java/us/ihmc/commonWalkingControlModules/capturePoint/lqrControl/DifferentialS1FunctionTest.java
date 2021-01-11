package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.matrixlib.MatrixTestTools;

public class DifferentialS1FunctionTest
{
   @Test
   public void test()
   {
      LQRCommonValues commonValues = new LQRCommonValues();
      commonValues.computeDynamicsMatrix(3.0);
      commonValues.computeEquivalentCostValues(1.0, 1e-3);

      AlgebraicS1Function endingS1Function = new AlgebraicS1Function();
      FlightS1Function flightS1Function = new FlightS1Function();
      DifferentialS1Segment firstS1Function = new DifferentialS1Segment(1e-5);

      DMatrixRMaj s1End = new DMatrixRMaj(6, 6);
      DMatrixRMaj s1StartOfFlight = new DMatrixRMaj(6, 6);
      DMatrixRMaj s1StartOfFlightOther = new DMatrixRMaj(6, 6);
      DMatrixRMaj s1Start = new DMatrixRMaj(6, 6);

      endingS1Function.set(commonValues);
      endingS1Function.compute(0.0, s1End);

      flightS1Function.set(s1End, 0.3);
      flightS1Function.compute(0.0, s1StartOfFlight);

      firstS1Function.set(commonValues, s1StartOfFlight, 1.0);
      firstS1Function.compute(1.0, s1StartOfFlightOther);

      MatrixTestTools.assertMatrixEquals(s1StartOfFlight, s1StartOfFlightOther, 1e-7);
   }
}
