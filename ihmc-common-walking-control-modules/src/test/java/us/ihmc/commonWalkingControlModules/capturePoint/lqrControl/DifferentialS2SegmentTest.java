package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class DifferentialS2SegmentTest
{
   @Test
   public void test()
   {
      LQRCommonValues commonValues = new LQRCommonValues();
      commonValues.computeDynamicsMatrix(3.0);
      commonValues.computeEquivalentCostValues(1.0, 1e-3);

      Random random = new Random(1738L);

      Point3D startVRP1 = EuclidCoreRandomTools.nextPoint3D(random);
      Point3D startVRP2 = EuclidCoreRandomTools.nextPoint3D(random);
      Point3D endVRP1 = EuclidCoreRandomTools.nextPoint3D(random);
      Point3D endVRP2 = EuclidCoreRandomTools.nextPoint3D(random);
      Trajectory3D vrpTrajectory1 = new Trajectory3D(3);
      Trajectory3D vrpTrajectory2 = new Trajectory3D(3);
      vrpTrajectory1.setLinear(0.0, 1.0, startVRP1, endVRP1);
      vrpTrajectory2.setLinear(0.0, 1.0, startVRP2, endVRP2);

      DMatrixRMaj s2AtEnd = new DMatrixRMaj(6, 1);

      DMatrixRMaj s1AtLanding = new DMatrixRMaj(6, 6);
      DMatrixRMaj s2AtLanding = new DMatrixRMaj(6, 1);

      AlgebraicS1Function endingS1Function = new AlgebraicS1Function();
      AlgebraicS2Segment endingS2Function = new AlgebraicS2Segment();
      endingS1Function.set(commonValues);
      endingS1Function.compute(0.0, s1AtLanding);

      commonValues.computeS2ConstantStateMatrices(s1AtLanding);
      endingS2Function.set(new DMatrixRMaj(6, 1), vrpTrajectory2, commonValues);
      endingS2Function.compute(0.0, s2AtLanding);

      DMatrixRMaj s1AtTakeOff = new DMatrixRMaj(6, 6);
      DMatrixRMaj s2AtTakeOff = new DMatrixRMaj(6, 1);

      FlightS1Function flightS1Function = new FlightS1Function();
      flightS1Function.set(s1AtLanding, 0.3);
      flightS1Function.compute(0.0, s1AtTakeOff);
      FlightS2Function flightS2Function = new FlightS2Function(-9.81);
      flightS2Function.set(s1AtLanding, s2AtLanding, 0.3);
      flightS2Function.compute(0.0, s2AtTakeOff);


      s2AtEnd.setData(RandomNumbers.nextDoubleArray(random, 6, 10.0));

      DifferentialS2Segment firstS2Function = new DifferentialS2Segment(1e-6);
      DifferentialS1Segment firstS1Function = new DifferentialS1Segment(1e-6);

      DMatrixRMaj s2AtEndOther = new DMatrixRMaj(6, 1);
      DMatrixRMaj s1AtEndOther = new DMatrixRMaj(6, 6);
      firstS1Function.set(commonValues, s1AtTakeOff, 1.0);
      firstS2Function.set(firstS1Function, vrpTrajectory1, commonValues, s2AtTakeOff);

      firstS1Function.compute(1.0, s1AtEndOther);
      firstS2Function.compute(1.0, s2AtEndOther);

      MatrixTestTools.assertMatrixEquals(s2AtTakeOff, s2AtEndOther, 1e-7);
      MatrixTestTools.assertMatrixEquals(s1AtTakeOff, s1AtEndOther, 5e-3);
   }

   /*
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

    */
}
