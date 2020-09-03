package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
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
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega);

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
      QExpected.set(0, 0, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(1, 1, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(2, 2, LQRMomentumController.defaultVrpTrackingWeight);

      DMatrixRMaj RExpected = new DMatrixRMaj(3, 3);
      RExpected.set(0, 0, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(1, 1, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(2, 2, LQRMomentumController.defaultMomentumRateWeight);

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
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega);

      CoMTrajectoryPlanner coMTrajectoryPlanner = new CoMTrajectoryPlanner(-9.81, 1.0, null);

      Point3D vrpStart1 = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd1 = new Point3D(1.0, 0.5, 1.0);
      Point3D vrpStart2 = new Point3D(1.5, 0.2, 1.0);
      Point3D vrpEnd2 = new Point3D(2.0, -0.5, 1.0);


      SettableContactStateProvider contact1 = new SettableContactStateProvider();
      contact1.setStartCopPosition(new Point2D(vrpStart1));
      contact1.setEndCopPosition(new Point2D(vrpEnd1));
      contact1.getTimeInterval().setInterval(0.0, 1.0);
      contact1.setContactState(ContactState.IN_CONTACT);

      SettableContactStateProvider contact2 = new SettableContactStateProvider();
      contact2.getTimeInterval().setInterval(1.0, 1.3);
      contact2.setContactState(ContactState.FLIGHT);

      SettableContactStateProvider contact3 = new SettableContactStateProvider();
      contact3.setStartCopPosition(new Point2D(vrpStart2));
      contact3.setEndCopPosition(new Point2D(vrpEnd2));
      contact3.getTimeInterval().setInterval(1.3, 2.3);
      contact3.setContactState(ContactState.IN_CONTACT);

      List<SettableContactStateProvider> contactStates = new ArrayList<>();
      contactStates.add(contact1);
      contactStates.add(contact2);
      contactStates.add(contact3);

      coMTrajectoryPlanner.solveForTrajectory(contactStates);

      controller.setVRPTrajectory(coMTrajectoryPlanner.getVRPTrajectories(), contactStates);
   }
}
