package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class DifferentialS1SegmentTest
{
   @Test
   public void test()
   {
      LQRCommonValues commonValues = new LQRCommonValues();
      commonValues.computeDynamicsMatrix(3.0);
      commonValues.computeEquivalentCostValues(1.0, 1e-3);

      Random random = new Random(1738L);
      DMatrixRMaj S1AtEnd = new DMatrixRMaj(6, 6);
      S1AtEnd.setData(RandomNumbers.nextDoubleArray(random, 36, 10.0));

      DifferentialS1Segment firstS1Function = new DifferentialS1Segment(1e-5);

      DMatrixRMaj S1AtEndOther = new DMatrixRMaj(6, 6);
      firstS1Function.set(commonValues, S1AtEnd, 1.0);
      firstS1Function.compute(1.0, S1AtEndOther);

      MatrixTestTools.assertMatrixEquals(S1AtEnd, S1AtEndOther, 1e-7);
   }

   @Test
   public void testSize()
   {
      LQRCommonValues commonValues = new LQRCommonValues();
      commonValues.computeDynamicsMatrix(3.0);
      commonValues.computeEquivalentCostValues(1.0, 1e-3);

      Random random = new Random(1738L);
      DMatrixRMaj S1AtEnd = new DMatrixRMaj(6, 6);
      S1AtEnd.setData(RandomNumbers.nextDoubleArray(random, 36, 10.0));

      DifferentialS1Segment firstS1Function = new DifferentialS1Segment(0.1);

      firstS1Function.set(commonValues, S1AtEnd, 1.0);
      assertEquals(11, firstS1Function.S1Trajectory.size());
      assertEquals(10, firstS1Function.getStartIndex(1.0));

      firstS1Function = new DifferentialS1Segment(0.01);

      firstS1Function.set(commonValues, S1AtEnd, 1.0);
      assertEquals(101, firstS1Function.S1Trajectory.size());
      assertEquals(100, firstS1Function.getStartIndex(1.0));

      firstS1Function = new DifferentialS1Segment(0.001);

      firstS1Function.set(commonValues, S1AtEnd, 1.0);
      assertEquals(1001, firstS1Function.S1Trajectory.size());
      assertEquals(1000, firstS1Function.getStartIndex(1.0));


      firstS1Function = new DifferentialS1Segment(0.0001);

      firstS1Function.set(commonValues, S1AtEnd, 1.0);
      assertEquals(10001, firstS1Function.S1Trajectory.size());
      assertEquals(10000, firstS1Function.getStartIndex(1.0));

      firstS1Function = new DifferentialS1Segment(0.00001);

      firstS1Function.set(commonValues, S1AtEnd, 1.0);
      assertEquals(100001, firstS1Function.S1Trajectory.size());
      assertEquals(100000, firstS1Function.getStartIndex(1.0));

      firstS1Function = new DifferentialS1Segment(0.000001);

      firstS1Function.set(commonValues, S1AtEnd, 1.0);
      assertEquals(1000001, firstS1Function.S1Trajectory.size());
      assertEquals(1000000, firstS1Function.getStartIndex(1.0));
   }
}
