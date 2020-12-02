package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class LQRJumpMomentumControllerTest
{
   private static final double omega = 3.0;
   private static final double epsilon = 1e-9;

   @Test
   public void testComputingAndS1OneContact()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega, 1.0);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      vrpTrajectory.setLinear(0.0, 1.0, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);

      SettableContactStateProvider contact = new SettableContactStateProvider();
      contact.setStartCopPosition(new Point2D(vrpStart));
      contact.setEndCopPosition(new Point2D(vrpEnd));
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.setContactState(ContactState.IN_CONTACT);

      List<SettableContactStateProvider> contactStates = new ArrayList<>();
      contactStates.add(contact);

      controller.setVRPTrajectory(trajectories, contactStates);

      DMatrixRMaj AExpected = new DMatrixRMaj(6, 6);
      AExpected.set(0, 3, 1.0);
      AExpected.set(1, 4, 1.0);
      AExpected.set(2, 5, 1.0);

      DMatrixRMaj BExpected = new DMatrixRMaj(6, 3);
      BExpected.set(3, 0, 1.0);
      BExpected.set(4, 1, 1.0);
      BExpected.set(5, 2, 1.0);

      DMatrixRMaj CExpected = new DMatrixRMaj(3, 6);

      CExpected.set(0, 0, 1.0);
      CExpected.set(1, 1, 1.0);
      CExpected.set(2, 2, 1.0);

      DMatrixRMaj DExpected = new DMatrixRMaj(3, 3);

      DExpected.set(0, 0, -1.0 / MathTools.square(omega));
      DExpected.set(1, 1, -1.0 / MathTools.square(omega));
      DExpected.set(2, 2, -1.0 / MathTools.square(omega));

      EjmlUnitTests.assertEquals(AExpected, controller.getA(), epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.getB(), epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.getC(), epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.getD(), epsilon);

      DMatrixRMaj QExpected = new DMatrixRMaj(3, 3);
      QExpected.set(0, 0, LQRJumpMomentumController.defaultVrpTrackingWeight);
      QExpected.set(1, 1, LQRJumpMomentumController.defaultVrpTrackingWeight);
      QExpected.set(2, 2, LQRJumpMomentumController.defaultVrpTrackingWeight);

      DMatrixRMaj RExpected = new DMatrixRMaj(3, 3);
      RExpected.set(0, 0, LQRJumpMomentumController.defaultMomentumRateWeight);
      RExpected.set(1, 1, LQRJumpMomentumController.defaultMomentumRateWeight);
      RExpected.set(2, 2, LQRJumpMomentumController.defaultMomentumRateWeight);

      EjmlUnitTests.assertEquals(QExpected, controller.getQ(), epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.getR(), epsilon);

      DMatrixRMaj Q1Expected = new DMatrixRMaj(3, 3);
      NativeCommonOps.multQuad(CExpected, QExpected, Q1Expected);

      DMatrixRMaj R1Expected = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InverseExpected = new DMatrixRMaj(3, 3);
      NativeCommonOps.multQuad(DExpected, QExpected, R1Expected);
      CommonOps_DDRM.addEquals(R1Expected, RExpected);
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

//      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
//      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
//      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(QExpected, DExpected, tempMatrix);
      CommonOps_DDRM.multTransA(CExpected, tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

//      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);
//      EjmlUnitTests.assertEquals(NTransposeExpected, controller.NTranspose, epsilon);


      controller.computeP();

      EjmlUnitTests.assertEquals(AExpected, controller.getA(), epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.getB(), epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.getC(), epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.getD(), epsilon);

      EjmlUnitTests.assertEquals(QExpected, controller.getQ(), epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.getR(), epsilon);

//      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
//      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
//      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

//      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);

      DMatrixRMaj QRiccatiExpected = new DMatrixRMaj(3, 3);
      DMatrixRMaj ARiccatiExpected = new DMatrixRMaj(6, 6);

      NativeCommonOps.multQuad(NTransposeExpected, R1InverseExpected, QRiccatiExpected);
      CommonOps_DDRM.scale(-1.0, QRiccatiExpected);
      CommonOps_DDRM.addEquals(QRiccatiExpected, Q1Expected);

      tempMatrix = new DMatrixRMaj(3, 6);
      CommonOps_DDRM.multTransB(R1InverseExpected, NExpected, tempMatrix);
      CommonOps_DDRM.mult(-1.0, BExpected, tempMatrix, ARiccatiExpected);
      CommonOps_DDRM.addEquals(ARiccatiExpected, AExpected);

//      EjmlUnitTests.assertEquals(QRiccatiExpected, controller.QRiccati, epsilon);
//      EjmlUnitTests.assertEquals(ARiccatiExpected, controller.ARiccati, epsilon);


      controller.computeS1Segments();
      controller.computeS1AndK1(0.0);
      DMatrixRMaj P = new DMatrixRMaj(controller.getCostHessian());
//
      DMatrixRMaj NB = new DMatrixRMaj(NExpected);
      CommonOps_DDRM.transpose(NB);
      CommonOps_DDRM.multAddTransA(BExpected, P, NB);
      DMatrixRMaj S1DotExpected = new DMatrixRMaj(6, 6);
      NativeCommonOps.multQuad(NB, R1InverseExpected, S1DotExpected);
      CommonOps_DDRM.addEquals(S1DotExpected, -1.0, Q1Expected);
      CommonOps_DDRM.multAdd(-1.0, P, AExpected, S1DotExpected);
      CommonOps_DDRM.multAddTransA(-1.0, AExpected, P, S1DotExpected);

      DMatrixRMaj H = new DMatrixRMaj( 4, 4);
      H.set(0, 1, 1.0);
      H.set(1, 0, 0.5);
      H.set(1, 3, -1.0);
      H.set(2, 0, -0.5);
      H.set(2, 3, -0.5);
      H.set(3, 2, -1.0);


      /*
      tempMatrix.reshape(6, 3);
      DMatrixRMaj S1Dot = new DMatrixRMaj(6, 6);
      DMatrixRMaj BTranspose = new DMatrixRMaj(controller.getB());
      CommonOps_DDRM.transpose(BTranspose);
      NativeCommonOps.multQuad(BTranspose, R1InverseExpected, tempMatrix);
      NativeCommonOps.multQuad(P, tempMatrix, S1Dot);
      CommonOps_DDRM.addEquals(S1Dot, -1.0, controller.QRiccati);
      CommonOps_DDRM.multAdd(-1.0, P, controller.ARiccati, S1Dot);
      CommonOps_DDRM.multAddTransA(-1.0, controller.ARiccati, P, S1Dot);

      EjmlUnitTests.assertEquals(S1DotExpected, S1Dot, epsilon);
      EjmlUnitTests.assertEquals(new DMatrixRMaj(6, 6), S1Dot, epsilon);
      */

      for (double time = 0.0; time <= 1.0; time += 0.001)
      {
         controller.computeS1Segments();
         controller.computeS1AndK1(time);
         MatrixTestTools.assertMatrixEquals(P, controller.getCostHessian(), epsilon);
      }
   }


   @Test
   public void testCostFunctionContinuity()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega, 1.0);

      CoMTrajectoryPlanner coMTrajectoryPlanner = new CoMTrajectoryPlanner(-9.81, 1.0, null);

      Point3D vrpStart1 = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd1 = new Point3D(1.0, 0.5, 1.0);
      Point3D vrpStart2 = new Point3D(1.5, 0.2, 1.0);
      Point3D vrpEnd2 = new Point3D(2.0, -0.5, 1.0);

      double contactDuration = 1.0;
      double flightDuration = 0.3;

      double startTime = 0.0;

      SettableContactStateProvider contact1 = new SettableContactStateProvider();
      contact1.setStartCopPosition(new Point2D(vrpStart1));
      contact1.setEndCopPosition(new Point2D(vrpEnd1));
      contact1.getTimeInterval().setInterval(startTime, startTime + contactDuration);
      contact1.setContactState(ContactState.IN_CONTACT);

      startTime += contactDuration;

      SettableContactStateProvider contact2 = new SettableContactStateProvider();
      contact2.getTimeInterval().setInterval(startTime, startTime + flightDuration);
      contact2.setContactState(ContactState.FLIGHT);

      startTime += flightDuration;

      SettableContactStateProvider contact3 = new SettableContactStateProvider();
      contact3.setStartCopPosition(new Point2D(vrpStart2));
      contact3.setEndCopPosition(new Point2D(vrpEnd2));
      contact3.getTimeInterval().setInterval(startTime, startTime + contactDuration);
      contact3.setContactState(ContactState.IN_CONTACT);

      List<SettableContactStateProvider> contactStates = new ArrayList<>();
      contactStates.add(contact1);
      contactStates.add(contact2);
      contactStates.add(contact3);

      coMTrajectoryPlanner.solveForTrajectory(contactStates);

      controller.setVRPTrajectory(coMTrajectoryPlanner.getVRPTrajectories(), contactStates);
      controller.computeS1Segments();
      controller.computeS2Segments();

      LQRCommonValues commonValues = new LQRCommonValues();
      commonValues.computeDynamicsMatrix(omega);
      commonValues.computeEquivalentCostValues(controller.defaultMomentumRateWeight, controller.defaultVrpTrackingWeight);

      AlgebraicS1Function finalS1Function = new AlgebraicS1Function();
      FlightS1Function flightS1Function = new FlightS1Function();
      DifferentialS1Segment initialS1Function = new DifferentialS1Segment(1e-4);

      AlgebraicS2Segment finalS2Function = new AlgebraicS2Segment();
      FlightS2Function flightS2Function = new FlightS2Function(-9.81);
      DifferentialS2Segment initialS2Function = new DifferentialS2Segment(1e-4);

      finalS1Function.set(commonValues);

      DMatrixRMaj expectedStartOfS13 = new DMatrixRMaj(6, 6);
      DMatrixRMaj expectedStartOfS12 = new DMatrixRMaj(6, 6);
      DMatrixRMaj expectedStartOfS11 = new DMatrixRMaj(6, 6);
      DMatrixRMaj expectedStartOfS23 = new DMatrixRMaj(6, 1);
      DMatrixRMaj expectedStartOfS22 = new DMatrixRMaj(6, 1);
      DMatrixRMaj expectedStartOfS21 = new DMatrixRMaj(6, 1);

      DMatrixRMaj expectedEndOfS13 = new DMatrixRMaj(6, 6);
      DMatrixRMaj expectedEndOfS12 = new DMatrixRMaj(6, 6);
      DMatrixRMaj expectedEndOfS11 = new DMatrixRMaj(6, 6);
      DMatrixRMaj expectedEndOfS23 = new DMatrixRMaj(6, 1);
      DMatrixRMaj expectedEndOfS22 = new DMatrixRMaj(6, 1);
      DMatrixRMaj expectedEndOfS21 = new DMatrixRMaj(6, 1);

      finalS1Function.compute(0.0, expectedStartOfS13);
      finalS1Function.compute(contactDuration, expectedEndOfS13);

      Trajectory3D relativeVRPTrajectory = new Trajectory3D(4);
      Trajectory3D initialVRPTrajectory = new Trajectory3D(4);
      relativeVRPTrajectory.set(coMTrajectoryPlanner.getVRPTrajectories().get(2));
      initialVRPTrajectory.set(coMTrajectoryPlanner.getVRPTrajectories().get(0));
      relativeVRPTrajectory.compute(relativeVRPTrajectory.getFinalTime());
      Point3DReadOnly finalPosition = relativeVRPTrajectory.getPosition();
      relativeVRPTrajectory.offsetTrajectoryPosition(-finalPosition.getX(), -finalPosition.getY(), -finalPosition.getZ());
      initialVRPTrajectory.offsetTrajectoryPosition(-finalPosition.getX(), -finalPosition.getY(), -finalPosition.getZ());

      commonValues.computeS2ConstantStateMatrices(expectedStartOfS13);
      finalS2Function.set(new DMatrixRMaj(6, 1), relativeVRPTrajectory, commonValues);
      finalS2Function.compute(0.0, expectedStartOfS23);
      finalS2Function.compute(contactDuration, expectedEndOfS23);

      flightS1Function.set(expectedStartOfS13, flightDuration);
      flightS1Function.compute(0.0, expectedStartOfS12);
      flightS1Function.compute(flightDuration, expectedEndOfS12);

      flightS2Function.set(expectedStartOfS13, expectedStartOfS23, flightDuration);
      flightS2Function.compute(0.0, expectedStartOfS22);
      flightS2Function.compute(flightDuration, expectedEndOfS22);

      initialS1Function.set(commonValues, expectedStartOfS12, contactDuration);
      initialS1Function.compute(0.0, expectedStartOfS11);
      initialS1Function.compute(contactDuration, expectedEndOfS11);

      initialS2Function.set(initialS1Function, initialVRPTrajectory, commonValues, expectedStartOfS22);
      initialS2Function.compute(0.0, expectedStartOfS21);
      initialS2Function.compute(contactDuration, expectedEndOfS21);


      DMatrixRMaj startOfS11 = new DMatrixRMaj(6, 6);
      DMatrixRMaj endOfS11 = new DMatrixRMaj(6, 6);
      DMatrixRMaj startOfS12 = new DMatrixRMaj(6, 6);
      DMatrixRMaj endOfS12 = new DMatrixRMaj(6, 6);
      DMatrixRMaj startOfS13 = new DMatrixRMaj(6, 6);
      DMatrixRMaj endOfS13 = new DMatrixRMaj(6, 6);
      controller.getS1Segment(0).compute(0.0, startOfS11);
      controller.getS1Segment(0).compute(contactDuration, endOfS11);
      controller.getS1Segment(1).compute(0.0, startOfS12);
      controller.getS1Segment(1).compute(flightDuration, endOfS12);
      controller.getS1Segment(2).compute(0.0, startOfS13);
      controller.getS1Segment(2).compute(contactDuration, endOfS13);

      DMatrixRMaj startOfS21 = new DMatrixRMaj(6, 1);
      DMatrixRMaj endOfS21 = new DMatrixRMaj(6, 1);
      DMatrixRMaj startOfS22 = new DMatrixRMaj(6, 1);
      DMatrixRMaj endOfS22 = new DMatrixRMaj(6, 1);
      DMatrixRMaj startOfS23 = new DMatrixRMaj(6, 1);
      DMatrixRMaj endOfS23 = new DMatrixRMaj(6, 1);
      controller.getS2Segment(0).compute(0.0, startOfS21);
      controller.getS2Segment(0).compute(contactDuration, endOfS21);
      controller.getS2Segment(1).compute(0.0, startOfS22);
      controller.getS2Segment(1).compute(flightDuration, endOfS22);
      controller.getS2Segment(2).compute(0.0, startOfS23);
      controller.getS2Segment(2).compute(contactDuration, endOfS23);


      assertPercentageMatrixEquals(finalS2Function.getAlpha(), ((AlgebraicS2Segment) controller.getS2Segment(2)).getAlpha(), 1e-7);
      assertPercentageMatrixEquals(finalS2Function.getBeta(0), ((AlgebraicS2Segment) controller.getS2Segment(2)).getBeta(0), 1e-7);
      assertPercentageMatrixEquals(finalS2Function.getBeta(1), ((AlgebraicS2Segment) controller.getS2Segment(2)).getBeta(1), 1e-7);
      assertPercentageMatrixEquals(expectedStartOfS13, startOfS13, 1e-4);
      assertPercentageMatrixEquals(expectedStartOfS12, startOfS12, 1e-4);
      assertPercentageMatrixEquals(expectedStartOfS11, startOfS11, 1e-3);
      assertPercentageMatrixEquals(expectedStartOfS23, startOfS23, 1e-4);
      assertPercentageMatrixEquals(expectedStartOfS22, startOfS22, 1e-4);
      assertPercentageMatrixEquals(expectedStartOfS21, startOfS21, 1e-2);

      MatrixTestTools.assertMatrixEquals(expectedEndOfS13, endOfS13, 1e-7);
      MatrixTestTools.assertMatrixEquals(expectedEndOfS12, endOfS12, 1e-7);
      MatrixTestTools.assertMatrixEquals(expectedEndOfS11, endOfS11, 1e-7);
      MatrixTestTools.assertMatrixEquals(expectedEndOfS23, endOfS23, 1e-7);
      MatrixTestTools.assertMatrixEquals(expectedEndOfS22, endOfS22, 1e-7);
      MatrixTestTools.assertMatrixEquals(expectedEndOfS21, endOfS21, 1e-7);

      MatrixTestTools.assertMatrixEquals(expectedEndOfS12, expectedStartOfS13, 1e-7);
      MatrixTestTools.assertMatrixEquals(expectedEndOfS11, expectedStartOfS12, 7e-1);

      MatrixTestTools.assertMatrixEquals(expectedEndOfS22, expectedStartOfS23, 1e-7);
      MatrixTestTools.assertMatrixEquals(expectedEndOfS21, expectedStartOfS22, 1e-7);

      MatrixTestTools.assertMatrixEquals(startOfS23, endOfS22, 1e-7);
      MatrixTestTools.assertMatrixEquals(startOfS22, endOfS21, 1e-7);
   }

   private static void assertPercentageMatrixEquals(DMatrix expected, DMatrix actual, double delta)
   {
      assertPercentageMatrixEquals("", expected, actual, delta);
   }

   private static void assertPercentageMatrixEquals(String message, DMatrix expected, DMatrix actual, double delta)
   {
      Assertions.assertEquals(expected.getNumRows(), actual.getNumRows(), message);
      Assertions.assertEquals(expected.getNumCols(), actual.getNumCols(), message);

      for (int i = 0; i < expected.getNumRows(); i++)
      {
         for (int j = 0; j < expected.getNumCols(); j++)
         {
            double epsilon = Math.max(Math.abs(expected.get(i, j)) * delta, delta);
            Assertions.assertEquals(expected.get(i, j), actual.get(i, j), epsilon, message + " index (" + i + ", " + j + ")");
         }
      }
   }
}
