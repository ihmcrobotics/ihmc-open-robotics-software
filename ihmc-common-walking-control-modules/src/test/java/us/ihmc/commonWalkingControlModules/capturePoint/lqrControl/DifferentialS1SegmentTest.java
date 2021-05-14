package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;

import java.util.ArrayList;
import java.util.List;
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

   @Test
   public void testSparseFunction()
   {
      LQRCommonValues commonValues = new LQRCommonValues();
      commonValues.computeDynamicsMatrix(3.0);
      commonValues.computeEquivalentCostValues(1.0, 1e-3);

      Random random = new Random(1738L);
      DMatrixRMaj S1AtEnd = new DMatrixRMaj(6, 6);
      S1AtEnd.setData(RandomNumbers.nextDoubleArray(random, 36, 10.0));

      double dt = 1e-1;
      double duration = 1.0;
      DifferentialS1Segment firstS1Function = new DifferentialS1Segment(dt);

      DMatrixRMaj S1AtEndOther = new DMatrixRMaj(6, 6);
      firstS1Function.set(commonValues, S1AtEnd, duration);
      firstS1Function.compute(1.0, S1AtEndOther);

      List<DMatrixRMaj> S1ReverseTrajectory = new ArrayList<>();
      List<DMatrixRMaj> S1Trajectory = new ArrayList<>();


      S1ReverseTrajectory.add(new DMatrixRMaj(S1AtEnd));

      for (double t = dt; t <= duration + dt / 10.0; t += dt)
      {
         DMatrixRMaj previousS1 = S1ReverseTrajectory.get(S1ReverseTrajectory.size() - 1);
         DMatrixRMaj newS1 = new DMatrixRMaj(6, 6);

         DMatrixRMaj NB = new DMatrixRMaj(3, 3);
         DMatrixRMaj S1Dot = new DMatrixRMaj(6, 6);

         DifferentialS1Segment.computeNB(commonValues.getB(), commonValues.getNTranspose(), previousS1, NB);
         DifferentialS1Segment.computeS1Dot(commonValues.getQ1(), NB, commonValues.getR1Inverse(), previousS1, commonValues.getA(), S1Dot);

         CommonOps_DDRM.add(previousS1, -dt, S1Dot, newS1);

         S1ReverseTrajectory.add(newS1);
      }

      for (int i = S1ReverseTrajectory.size() - 1; i >= 0; i--)
      {
         S1Trajectory.add(S1ReverseTrajectory.get(i));
      }

      assertEquals(S1Trajectory.size(), firstS1Function.S1Trajectory.size());
      for (int i = 0; i < S1Trajectory.size(); i++)
      {
         MatrixTestTools.assertMatrixEquals(S1Trajectory.get(i), firstS1Function.S1Trajectory.get(i), 1e-5);
      }
      MatrixTestTools.assertMatrixEquals(S1AtEnd, S1AtEndOther, 1e-7);
   }

}
