package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
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

      EjmlUnitTests.assertEquals(AExpected, controller.A, epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.B, epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.C, epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.D, epsilon);

      DMatrixRMaj QExpected = new DMatrixRMaj(3, 3);
      QExpected.set(0, 0, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(1, 1, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(2, 2, LQRMomentumController.defaultVrpTrackingWeight);

      DMatrixRMaj RExpected = new DMatrixRMaj(3, 3);
      RExpected.set(0, 0, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(1, 1, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(2, 2, LQRMomentumController.defaultMomentumRateWeight);

      EjmlUnitTests.assertEquals(QExpected, controller.Q, epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.R, epsilon);

      DMatrixRMaj Q1Expected = new DMatrixRMaj(3, 3);
      NativeCommonOps.multQuad(CExpected, QExpected, Q1Expected);

      DMatrixRMaj R1Expected = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InverseExpected = new DMatrixRMaj(3, 3);
      NativeCommonOps.multQuad(DExpected, QExpected, R1Expected);
      CommonOps_DDRM.addEquals(R1Expected, RExpected);
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(QExpected, DExpected, tempMatrix);
      CommonOps_DDRM.multTransA(CExpected, tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);
      EjmlUnitTests.assertEquals(NTransposeExpected, controller.NTranspose, epsilon);


      controller.computeP();

      EjmlUnitTests.assertEquals(AExpected, controller.A, epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.B, epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.C, epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.D, epsilon);

      EjmlUnitTests.assertEquals(QExpected, controller.Q, epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.R, epsilon);

      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);

      DMatrixRMaj QRiccatiExpected = new DMatrixRMaj(3, 3);
      DMatrixRMaj ARiccatiExpected = new DMatrixRMaj(6, 6);

      NativeCommonOps.multQuad(NTransposeExpected, R1InverseExpected, QRiccatiExpected);
      CommonOps_DDRM.scale(-1.0, QRiccatiExpected);
      CommonOps_DDRM.addEquals(QRiccatiExpected, Q1Expected);

      tempMatrix = new DMatrixRMaj(3, 6);
      CommonOps_DDRM.multTransB(R1InverseExpected, NExpected, tempMatrix);
      CommonOps_DDRM.mult(-1.0, BExpected, tempMatrix, ARiccatiExpected);
      CommonOps_DDRM.addEquals(ARiccatiExpected, AExpected);

      EjmlUnitTests.assertEquals(QRiccatiExpected, controller.QRiccati, epsilon);
      EjmlUnitTests.assertEquals(ARiccatiExpected, controller.ARiccati, epsilon);


      DMatrixRMaj P = new DMatrixRMaj(controller.P);
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


      tempMatrix.reshape(6, 3);
      DMatrixRMaj S1Dot = new DMatrixRMaj(6, 6);
      DMatrixRMaj BTranspose = new DMatrixRMaj(controller.B);
      CommonOps_DDRM.transpose(BTranspose);
      NativeCommonOps.multQuad(BTranspose, controller.R1Inverse, tempMatrix);
      NativeCommonOps.multQuad(P, tempMatrix, S1Dot);
      CommonOps_DDRM.addEquals(S1Dot, -1.0, controller.QRiccati);
      CommonOps_DDRM.multAdd(-1.0, P, controller.ARiccati, S1Dot);
      CommonOps_DDRM.multAddTransA(-1.0, controller.ARiccati, P, S1Dot);

      EjmlUnitTests.assertEquals(S1DotExpected, S1Dot, epsilon);
      EjmlUnitTests.assertEquals(new DMatrixRMaj(6, 6), S1Dot, epsilon);

      for (double time = 0.0; time <= 1.0; time += 0.001)
      {
         controller.computeS1Parameters();
         controller.computeS1AndK1(time);
         MatrixTestTools.assertMatrixEquals(P, controller.getCostHessian(), epsilon);
      }
   }

   /*
   @Test
   public void testComputingAndS1TwoContactsStartingInJump()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);

      double startTime = 0;
      double landingTime = 0.15;
      double finalTime = 1.0;
      Trajectory3D vrpTrajectory1 = new Trajectory3D(2);
      vrpTrajectory1.setLinear(startTime, landingTime, vrpStart, vrpEnd);
      Trajectory3D vrpTrajectory2 = new Trajectory3D(4);
      vrpTrajectory2.setLinear(landingTime, finalTime, vrpStart, vrpEnd);

      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory1);
      trajectories.add(vrpTrajectory2);

      SettableContactStateProvider flightContact = new SettableContactStateProvider();
      flightContact.getTimeInterval().setInterval(startTime, landingTime);
      flightContact.setContactState(ContactState.FLIGHT);

      SettableContactStateProvider landingContact = new SettableContactStateProvider();
      landingContact.setStartCopPosition(new Point2D(vrpStart));
      landingContact.setEndCopPosition(new Point2D(vrpEnd));
      landingContact.getTimeInterval().setInterval(landingTime, finalTime);
      landingContact.setContactState(ContactState.IN_CONTACT);

      List<SettableContactStateProvider> contactStates = new ArrayList<>();
      contactStates.add(flightContact);
      contactStates.add(landingContact);

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

      EjmlUnitTests.assertEquals(AExpected, controller.A, epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.B, epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.C, epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.D, epsilon);

      DMatrixRMaj QExpected = new DMatrixRMaj(3, 3);
      QExpected.set(0, 0, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(1, 1, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(2, 2, LQRMomentumController.defaultVrpTrackingWeight);

      DMatrixRMaj RExpected = new DMatrixRMaj(3, 3);
      RExpected.set(0, 0, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(1, 1, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(2, 2, LQRMomentumController.defaultMomentumRateWeight);

      EjmlUnitTests.assertEquals(QExpected, controller.Q, epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.R, epsilon);

      DMatrixRMaj Q1Expected = new DMatrixRMaj(3, 3);
      NativeCommonOps.multQuad(CExpected, QExpected, Q1Expected);

      DMatrixRMaj R1Expected = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InverseExpected = new DMatrixRMaj(3, 3);
      NativeCommonOps.multQuad(DExpected, QExpected, R1Expected);
      CommonOps_DDRM.addEquals(R1Expected, RExpected);
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(QExpected, DExpected, tempMatrix);
      CommonOps_DDRM.multTransA(CExpected, tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);
      EjmlUnitTests.assertEquals(NTransposeExpected, controller.NTranspose, epsilon);


      controller.computeP();

      EjmlUnitTests.assertEquals(AExpected, controller.A, epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.B, epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.C, epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.D, epsilon);

      EjmlUnitTests.assertEquals(QExpected, controller.Q, epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.R, epsilon);

      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);

      DMatrixRMaj QRiccatiExpected = new DMatrixRMaj(3, 3);
      DMatrixRMaj ARiccatiExpected = new DMatrixRMaj(6, 6);

      NativeCommonOps.multQuad(NTransposeExpected, R1InverseExpected, QRiccatiExpected);
      CommonOps_DDRM.scale(-1.0, QRiccatiExpected);
      CommonOps_DDRM.addEquals(QRiccatiExpected, Q1Expected);

      tempMatrix = new DMatrixRMaj(3, 6);
      CommonOps_DDRM.multTransB(R1InverseExpected, NExpected, tempMatrix);
      CommonOps_DDRM.mult(-1.0, BExpected, tempMatrix, ARiccatiExpected);
      CommonOps_DDRM.addEquals(ARiccatiExpected, AExpected);

      EjmlUnitTests.assertEquals(QRiccatiExpected, controller.QRiccati, epsilon);
      EjmlUnitTests.assertEquals(ARiccatiExpected, controller.ARiccati, epsilon);


      DMatrixRMaj P = new DMatrixRMaj(controller.P);
      //
      DMatrixRMaj NB = new DMatrixRMaj(NExpected);
      CommonOps_DDRM.transpose(NB);
      CommonOps_DDRM.multAddTransA(BExpected, P, NB);
      DMatrixRMaj PDotExpected = new DMatrixRMaj(6, 6);
      NativeCommonOps.multQuad(NB, R1InverseExpected, PDotExpected);
      CommonOps_DDRM.addEquals(PDotExpected, -1.0, Q1Expected);
      CommonOps_DDRM.multAdd(-1.0, P, AExpected, PDotExpected);
      CommonOps_DDRM.multAddTransA(-1.0, AExpected, P, PDotExpected);

      DMatrixRMaj H = new DMatrixRMaj(4, 4);
      H.set(0, 1, 1.0);
      H.set(1, 0, 0.5);
      H.set(1, 3, -1.0);
      H.set(2, 0, -0.5);
      H.set(2, 3, -0.5);
      H.set(3, 2, -1.0);


      tempMatrix.reshape(6, 3);
      DMatrixRMaj PDot = new DMatrixRMaj(6, 6);
      DMatrixRMaj BTranspose = new DMatrixRMaj(controller.B);
      CommonOps_DDRM.transpose(BTranspose);
      NativeCommonOps.multQuad(BTranspose, controller.R1Inverse, tempMatrix);
      NativeCommonOps.multQuad(P, tempMatrix, PDot);
      CommonOps_DDRM.addEquals(PDot, -1.0, controller.QRiccati);
      CommonOps_DDRM.multAdd(-1.0, P, controller.ARiccati, PDot);
      CommonOps_DDRM.multAddTransA(-1.0, controller.ARiccati, P, PDot);

      EjmlUnitTests.assertEquals(PDotExpected, PDot, epsilon);
      EjmlUnitTests.assertEquals(new DMatrixRMaj(6, 6), PDot, epsilon);

      DMatrixRMaj flightDynamicsExpected = new DMatrixRMaj(6, 6);

      DMatrixRMaj S1Expected = new DMatrixRMaj(6, 6);

      controller.computeS1Parameters();

      for (double time = startTime; time <= landingTime; time += 0.001)
      {
         double remainingTime = landingTime - time;
         CommonOps_DDRM.setIdentity(flightDynamicsExpected);
         flightDynamicsExpected.set(0, 3, remainingTime);
         flightDynamicsExpected.set(1, 4, remainingTime);
         flightDynamicsExpected.set(2, 5, remainingTime);

         DMatrixRMaj flightDynamics = new DMatrixRMaj(6, 6);
         CommonOps_DDRM.scale(remainingTime, controller.Ad1, flightDynamics);
         CommonOps_DDRM.addEquals(flightDynamics, controller.Ad2);

         // these are not the same functions, apparently
         DMatrixRMaj flightDynamics2 = new DMatrixRMaj(6, 6);
         CommonOps_DDRM.scale(time - startTime, controller.sigmas.get(0), flightDynamics2);
         CommonOps_DDRM.addEquals(flightDynamics2, controller.phis.get(0));

         MatrixTestTools.assertMatrixEquals(flightDynamicsExpected, flightDynamics, epsilon);
         MatrixTestTools.assertMatrixEquals(flightDynamicsExpected, flightDynamics2, epsilon);
      }

      for (double time = startTime; time <= landingTime; time += 0.001)
      {
         controller.computeS1Parameters();
         controller.computeS1AndK1(time);

         double remainingTime = landingTime - time;
         CommonOps_DDRM.setIdentity(flightDynamicsExpected);
         flightDynamicsExpected.set(0, 3, remainingTime);
         flightDynamicsExpected.set(1, 4, remainingTime);
         flightDynamicsExpected.set(2, 5, remainingTime);

         DMatrixRMaj flightDynamics2 = new DMatrixRMaj(6, 6);
         CommonOps_DDRM.scale(time - startTime, controller.sigmas.get(0), flightDynamics2);
         CommonOps_DDRM.addEquals(flightDynamics2, controller.phis.get(0));

         DMatrixRMaj S1Expected2 = new DMatrixRMaj(6, 6);
         NativeCommonOps.multQuad(flightDynamics2, P, S1Expected2);

         NativeCommonOps.multQuad(flightDynamicsExpected, P, S1Expected);

         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, S1Expected2, controller.getCostHessian(), epsilon);
         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, S1Expected, controller.getCostHessian(), epsilon);
      }

      for (double time = landingTime; time <= finalTime; time += 0.001)
      {
         controller.computeS1Parameters();
         controller.computeS1AndK1(time);

         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, P, controller.getCostHessian(), epsilon);
      }
   }

   @Test
   public void testComputingAndS1FourContactsStartingInJump()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega);

      Point3D vrpStart1 = new Point3D(1.0, 0.5, 1.0);
      Point3D vrpEnd1 = new Point3D(1.0, 0.5, 1.0);
      Point3D vrpStart2 = new Point3D(2.0, -0.5, 1.0);
      Point3D vrpEnd2 = new Point3D(2.0, -0.5, 1.0);

      double startTime = 0;
      double landingTime1 = 0.15;
      double liftOffTime1 = 1.0;
      double landingTime2 = 1.15;
      double finalTime = 2.0;
      Trajectory3D vrpTrajectory1 = new Trajectory3D(2);
      Trajectory3D vrpTrajectory2 = new Trajectory3D(4);
      Trajectory3D vrpTrajectory3 = new Trajectory3D(2);
      Trajectory3D vrpTrajectory4 = new Trajectory3D(4);
      vrpTrajectory1.setLinear(startTime, landingTime1, vrpStart1, vrpEnd1);
      vrpTrajectory2.setLinear(landingTime1, liftOffTime1, vrpStart1, vrpEnd1);
      vrpTrajectory3.setLinear(liftOffTime1, landingTime2, vrpStart2, vrpEnd2);
      vrpTrajectory4.setLinear(landingTime2, finalTime, vrpStart2, vrpEnd2);

      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory1);
      trajectories.add(vrpTrajectory2);
      trajectories.add(vrpTrajectory3);
      trajectories.add(vrpTrajectory4);

      SettableContactStateProvider flightContact1 = new SettableContactStateProvider();
      flightContact1.getTimeInterval().setInterval(startTime, landingTime1);
      flightContact1.setContactState(ContactState.FLIGHT);

      SettableContactStateProvider landingContact1 = new SettableContactStateProvider();
      landingContact1.setStartCopPosition(new Point2D(vrpStart1));
      landingContact1.setEndCopPosition(new Point2D(vrpEnd1));
      landingContact1.getTimeInterval().setInterval(landingTime1, liftOffTime1);
      landingContact1.setContactState(ContactState.IN_CONTACT);

      SettableContactStateProvider flightContact2 = new SettableContactStateProvider();
      flightContact2.getTimeInterval().setInterval(liftOffTime1, landingTime2);
      flightContact2.setContactState(ContactState.FLIGHT);

      SettableContactStateProvider landingContact2 = new SettableContactStateProvider();
      landingContact2.setStartCopPosition(new Point2D(vrpStart2));
      landingContact2.setEndCopPosition(new Point2D(vrpEnd2));
      landingContact2.getTimeInterval().setInterval(landingTime2, finalTime);
      landingContact2.setContactState(ContactState.IN_CONTACT);

      List<SettableContactStateProvider> contactStates = new ArrayList<>();
      contactStates.add(flightContact1);
      contactStates.add(landingContact1);
      contactStates.add(flightContact2);
      contactStates.add(landingContact2);

      controller.setVRPTrajectory(trajectories, contactStates);

      controller.computeP();

      DMatrixRMaj P = new DMatrixRMaj(controller.P);

      DMatrixRMaj flightDynamicsExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj nextFlightDynamicsExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj S1Expected = new DMatrixRMaj(6, 6);

      controller.computeS1Parameters();

      DMatrixRMaj s1EndOf1 = new DMatrixRMaj(6, 6);
      DMatrixRMaj s1StartOf2 = new DMatrixRMaj(6, 6);
      DMatrixRMaj s1EndOf2 = new DMatrixRMaj(6, 6);
      DMatrixRMaj s1StartOf3 = new DMatrixRMaj(6, 6);
      DMatrixRMaj s1EndOf3 = new DMatrixRMaj(6, 6);
      DMatrixRMaj s1StartOf4 = new DMatrixRMaj(6, 6);
      DMatrixRMaj s1EndOf4 = new DMatrixRMaj(6, 6);

      DMatrixRMaj functionEndOf1 = new DMatrixRMaj(6, 6);
      DMatrixRMaj functionStartOf2 = new DMatrixRMaj(6, 6);
      DMatrixRMaj functionEndOf2 = new DMatrixRMaj(6, 6);
      DMatrixRMaj functionStartOf3 = new DMatrixRMaj(6, 6);
      DMatrixRMaj functionEndOf3 = new DMatrixRMaj(6, 6);
      DMatrixRMaj functionStartOf4 = new DMatrixRMaj(6, 6);
      DMatrixRMaj functionEndOf4 = new DMatrixRMaj(6, 6);

      functionEndOf1.set(controller.phis.get(0));
      CommonOps_DDRM.addEquals(functionEndOf1, landingTime1, controller.sigmas.get(0));
      functionStartOf2.set(controller.phis.get(1));
      functionEndOf2.set(controller.phis.get(1));
      CommonOps_DDRM.addEquals(functionEndOf2, liftOffTime1 - landingTime1, controller.sigmas.get(1));
      functionStartOf3.set(controller.phis.get(2));
      functionEndOf3.set(controller.phis.get(2));
      CommonOps_DDRM.addEquals(functionEndOf3, landingTime2 - liftOffTime1, controller.sigmas.get(2));
      functionStartOf4.set(controller.phis.get(3));
      functionEndOf4.set(controller.phis.get(3));
      CommonOps_DDRM.addEquals(functionEndOf4, finalTime - landingTime2, controller.sigmas.get(3));

      NativeCommonOps.multQuad(functionEndOf1, P, s1EndOf1);
      NativeCommonOps.multQuad(functionStartOf2, P, s1StartOf2);
      NativeCommonOps.multQuad(functionEndOf2, P, s1EndOf2);
      NativeCommonOps.multQuad(functionStartOf3, P, s1StartOf3);
      NativeCommonOps.multQuad(functionEndOf3, P, s1EndOf3);
      NativeCommonOps.multQuad(functionStartOf4, P, s1StartOf4);
      NativeCommonOps.multQuad(functionEndOf4, P, s1EndOf4);

      MatrixTestTools.assertMatrixEquals(s1EndOf1, s1StartOf2, epsilon);
      MatrixTestTools.assertMatrixEquals(s1EndOf2, s1StartOf3, epsilon);
      MatrixTestTools.assertMatrixEquals(s1EndOf3, s1StartOf4, epsilon);
      MatrixTestTools.assertMatrixEquals(s1EndOf4, P, epsilon);

      DMatrixRMaj S1DynamicsExpected = new DMatrixRMaj(6, 6);
      S1DynamicsExpected.zero();

      // compute during first flight phase
      for (double time = 0; time <= landingTime1; time += 0.001)
      {
         controller.computeS1Parameters();
         controller.computeS1AndK1(time);

         double timeRemaining = landingTime1 - time;
         CommonOps_DDRM.setIdentity(flightDynamicsExpected);
         flightDynamicsExpected.set(0, 3, timeRemaining);
         flightDynamicsExpected.set(1, 4, timeRemaining);
         flightDynamicsExpected.set(2, 5, timeRemaining);

         double nextFlightTime = landingTime2 - liftOffTime1;
         CommonOps_DDRM.setIdentity(nextFlightDynamicsExpected);
         nextFlightDynamicsExpected.set(0, 3, nextFlightTime);
         nextFlightDynamicsExpected.set(1, 4, nextFlightTime);
         nextFlightDynamicsExpected.set(2, 5, nextFlightTime);

         DMatrixRMaj flightDynamics2 = new DMatrixRMaj(6, 6);
         CommonOps_DDRM.scale(time, controller.sigmas.get(0), flightDynamics2);
         CommonOps_DDRM.addEquals(flightDynamics2, controller.phis.get(0));

         DMatrixRMaj S1Expected2 = new DMatrixRMaj(6, 6);
         NativeCommonOps.multQuad(flightDynamics2, P, S1Expected2);

         DMatrixRMaj nextS1Expected = new DMatrixRMaj(6, 6);
         NativeCommonOps.multQuad(nextFlightDynamicsExpected, P, nextS1Expected);
         NativeCommonOps.multQuad(flightDynamicsExpected, nextS1Expected, S1Expected);

         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, S1Expected2, controller.getCostHessian(), epsilon);
         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, S1Expected, controller.getCostHessian(), epsilon);
      }

      // compute during first contact phase
      for (double time = landingTime1; time <= liftOffTime1; time += 0.001)
      {
         controller.computeS1Parameters();
         controller.computeS1AndK1(time);

         double nextFlightTime = landingTime2 - liftOffTime1;
         CommonOps_DDRM.setIdentity(flightDynamicsExpected);
         flightDynamicsExpected.set(0, 3, nextFlightTime);
         flightDynamicsExpected.set(1, 4, nextFlightTime);
         flightDynamicsExpected.set(2, 5, nextFlightTime);

         DMatrixRMaj flightDynamics2 = new DMatrixRMaj(6, 6);
         CommonOps_DDRM.scale(time - landingTime1, controller.sigmas.get(1), flightDynamics2);
         CommonOps_DDRM.addEquals(flightDynamics2, controller.phis.get(1));

         DMatrixRMaj S1Expected2 = new DMatrixRMaj(6, 6);
         NativeCommonOps.multQuad(flightDynamics2, P, S1Expected2);

         NativeCommonOps.multQuad(flightDynamicsExpected, P, S1Expected);

         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, S1Expected2, controller.getCostHessian(), epsilon);
         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, S1Expected, controller.getCostHessian(), epsilon);

         MatrixTestTools.assertMatrixEquals(S1DynamicsExpected, computeRiccatiDynamics(controller, S1Expected), epsilon);
      }

      // compute during last flight phase
      for (double time = liftOffTime1; time <= landingTime2; time += 0.001)
      {
         controller.computeS1Parameters();
         controller.computeS1AndK1(time);

         double remainingTime = landingTime2 - time;
         CommonOps_DDRM.setIdentity(flightDynamicsExpected);
         flightDynamicsExpected.set(0, 3, remainingTime);
         flightDynamicsExpected.set(1, 4, remainingTime);
         flightDynamicsExpected.set(2, 5, remainingTime);

         DMatrixRMaj flightDynamics2 = new DMatrixRMaj(6, 6);
         CommonOps_DDRM.scale(time - liftOffTime1, controller.sigmas.get(2), flightDynamics2);
         CommonOps_DDRM.addEquals(flightDynamics2, controller.phis.get(2));

         DMatrixRMaj S1Expected2 = new DMatrixRMaj(6, 6);
         NativeCommonOps.multQuad(flightDynamics2, P, S1Expected2);

         NativeCommonOps.multQuad(flightDynamicsExpected, P, S1Expected);

         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, S1Expected2, controller.getCostHessian(), epsilon);
         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, S1Expected, controller.getCostHessian(), epsilon);

         MatrixTestTools.assertMatrixEquals(S1DynamicsExpected, computeRiccatiDynamics(controller, S1Expected), epsilon);
      }

      // compute during last contact phase
      for (double time = landingTime2; time <= finalTime; time += 0.001)
      {
         controller.computeS1Parameters();
         controller.computeS1AndK1(time);

         MatrixTestTools.assertMatrixEquals("Failed at time t = " + time, P, controller.getCostHessian(), epsilon);

         MatrixTestTools.assertMatrixEquals(S1DynamicsExpected, computeRiccatiDynamics(controller, controller.getCostHessian()), epsilon);

      }
   }

   private static DMatrixRMaj computeRiccatiDynamics(LQRJumpMomentumController controller, DMatrixRMaj S1)
   {
      DMatrixRMaj M = new DMatrixRMaj(6, 6);
      DMatrixRMaj BTranspose = new DMatrixRMaj(3, 6);
      CommonOps_DDRM.transpose(controller.B, BTranspose);
      NativeCommonOps.multQuad(BTranspose, controller.R1, M);

      DMatrixRMaj S1Dot = new DMatrixRMaj(6, 6);
      NativeCommonOps.multQuad(S1, M, S1Dot);
      CommonOps_DDRM.scale(-1.0, S1Dot);

      CommonOps_DDRM.addEquals(S1Dot, controller.QRiccati);
      CommonOps_DDRM.multAddTransA(controller.ARiccati, S1, S1Dot);
      CommonOps_DDRM.multAdd(S1, controller.ARiccati, S1Dot);

      return S1Dot;
   }

   @Test
   public void testComputingS2FromTwoLinearTrajectoriesInContact()
   {
      LQRJumpMomentumController controller = new LQRJumpMomentumController(() -> omega);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpMiddle = new Point3D(0.6, 0.75, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory1 = new Trajectory3D(4);
      Trajectory3D vrpTrajectory2 = new Trajectory3D(4);
      double finalTime1 = 1.5;
      double finalTime2 = 3.1;
      vrpTrajectory1.setLinear(0.0, finalTime1, vrpStart, vrpMiddle);
      vrpTrajectory2.setLinear(0.0, finalTime2 - finalTime1, vrpMiddle, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory1);
      trajectories.add(vrpTrajectory2);

      SettableContactStateProvider contactState1 = new SettableContactStateProvider();
      contactState1.setStartCopPosition(new Point2D(vrpStart));
      contactState1.setEndCopPosition(new Point2D(vrpMiddle));
      contactState1.getTimeInterval().setInterval(0.0, finalTime1);
      contactState1.setContactState(ContactState.IN_CONTACT);

      SettableContactStateProvider contactState2 = new SettableContactStateProvider();
      contactState2.setStartCopPosition(new Point2D(vrpMiddle));
      contactState2.setEndCopPosition(new Point2D(vrpEnd));
      contactState2.getTimeInterval().setInterval(finalTime1, finalTime2);
      contactState2.setContactState(ContactState.IN_CONTACT);

      List<SettableContactStateProvider> contactStates = new ArrayList<>();
      contactStates.add(contactState1);
      contactStates.add(contactState2);

      DMatrixRMaj finalPosition = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories, contactStates);

      controller.computeS1Parameters();
      controller.computeS2Parameters();

      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NBExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(controller.Q, controller.D, tempMatrix);
      CommonOps_DDRM.multTransA(controller.C, tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

      CommonOps_DDRM.multTransA(controller.B, controller.P, NBExpected);
      CommonOps_DDRM.addEquals(NBExpected, NTransposeExpected);

      LinearSolver<DMatrixRMaj> solver = LinearSolverFactory.general(0, 0);

      DMatrixRMaj A2Expected = new DMatrixRMaj(6, 6);
      DMatrixRMaj A2InverseExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj B2Expected = new DMatrixRMaj(6, 3);

      CommonOps_DDRM.transpose(controller.A, A2Expected);
      CommonOps_DDRM.scale(-1.0, A2Expected);

      tempMatrix.reshape(3, 6);
      CommonOps_DDRM.multTransB(controller.R1Inverse, controller.B, tempMatrix);
      CommonOps_DDRM.multAddTransA(NBExpected, tempMatrix, A2Expected);

      DMatrixRMaj tempMatrix2 = new DMatrixRMaj(6, 3);
      tempMatrix.reshape(6, 3);
      CommonOps_DDRM.multTransA(NBExpected, controller.R1Inverse, tempMatrix);
      CommonOps_DDRM.transpose(controller.C, tempMatrix2);
      CommonOps_DDRM.multAdd(-1.0, tempMatrix, controller.D, tempMatrix2);
      CommonOps_DDRM.mult(2.0, tempMatrix2, controller.Q, B2Expected);

      solver.setA(A2Expected);
      solver.invert(A2InverseExpected);

      EjmlUnitTests.assertEquals(NBExpected, controller.NB, epsilon);
      EjmlUnitTests.assertEquals(A2Expected, controller.A2, epsilon);
      EjmlUnitTests.assertEquals(B2Expected, controller.B2, epsilon);
      EjmlUnitTests.assertEquals(A2InverseExpected, controller.A2Inverse, epsilon);


      DMatrixRMaj A2InverseB2 = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1InvDQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InvDQInverse = new DMatrixRMaj(3, 3);
      DMatrixRMaj DQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InvBTrans = new DMatrixRMaj(3, 6);
      DMatrixRMaj k2Method1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj yd = new DMatrixRMaj(3, 1);

      CommonOps_DDRM.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps_DDRM.mult(controller.D, controller.Q, DQ);
      CommonOps_DDRM.mult(controller.R1Inverse, DQ, R1InvDQ);
      CommonOps_DDRM.multTransB(controller.R1Inverse, controller.B, R1InvBTrans);
      solver.setA(R1InvDQ);
      solver.invert(R1InvDQInverse);

      assertEquals(2, controller.alphas.size());
      assertEquals(2, controller.betas.size());
      assertEquals(2, controller.betas.get(0).size());
      assertEquals(2, controller.betas.get(1).size());


      DMatrixRMaj beta11Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta12Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta21Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta22Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj coefficients = new DMatrixRMaj(3, 1);


      vrpTrajectory2.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta22Expected);

      vrpTrajectory2.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta22Expected, beta21Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta21Expected);

      EjmlUnitTests.assertEquals(beta21Expected, controller.betas.get(1).get(0), epsilon);
      EjmlUnitTests.assertEquals(beta22Expected, controller.betas.get(1).get(1), epsilon);


      vrpTrajectory1.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta12Expected);


      vrpTrajectory1.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta12Expected, beta11Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta11Expected);

      EjmlUnitTests.assertEquals(beta11Expected, controller.betas.get(0).get(0), epsilon);
      EjmlUnitTests.assertEquals(beta12Expected, controller.betas.get(0).get(1), epsilon);

      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DMatrixRMaj alpha2Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj alpha1Expected = new DMatrixRMaj(6, 1);

      DMatrixRMaj timeScaledDynamics = new DMatrixRMaj(6, 6);
      DMatrixRMaj matrixExponential = new DMatrixRMaj(6, 6);
      DMatrixRMaj betaSum = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.scale((finalTime2 - finalTime1), A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 0), beta21Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 1), beta22Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha2Expected);

      CommonOps_DDRM.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      betaSum.zero();
      CommonOps_DDRM.addEquals(betaSum, alpha2Expected);
      CommonOps_DDRM.addEquals(betaSum, beta21Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 0), beta11Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 1), beta12Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha1Expected);

      EjmlUnitTests.assertEquals(alpha2Expected, controller.alphas.get(1), epsilon);
      EjmlUnitTests.assertEquals(alpha1Expected, controller.alphas.get(0), epsilon);



      // check for continuity
      DMatrixRMaj relativeMiddleVRP = new DMatrixRMaj(3, 1);
      vrpMiddle.get(relativeMiddleVRP);
      CommonOps_DDRM.subtractEquals(relativeMiddleVRP, finalPosition);

      DMatrixRMaj constructedPositionEndOf1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj constructedPositionStartOf2 = new DMatrixRMaj(3, 1);
      DMatrixRMaj positionEndOf1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj positionStartOf2 = new DMatrixRMaj(3, 1);

      controller.relativeVRPTrajectories.get(0).getCoefficients(0, coefficients);
      CommonOps_DDRM.addEquals(constructedPositionEndOf1, MathTools.pow(finalTime1, 0), coefficients);
      controller.relativeVRPTrajectories.get(0).getCoefficients(1, coefficients);
      CommonOps_DDRM.addEquals(constructedPositionEndOf1, MathTools.pow(finalTime1, 1), coefficients);

      controller.relativeVRPTrajectories.get(0).compute(finalTime1);
      controller.relativeVRPTrajectories.get(1).compute(0.0);
      controller.relativeVRPTrajectories.get(0).getPosition().get(positionEndOf1);
      controller.relativeVRPTrajectories.get(1).getPosition().get(positionStartOf2);


      controller.relativeVRPTrajectories.get(1).getCoefficients(0, coefficients);
      CommonOps_DDRM.addEquals(constructedPositionStartOf2, MathTools.pow(0, 0), coefficients);
      controller.relativeVRPTrajectories.get(1).getCoefficients(1, coefficients);
      CommonOps_DDRM.addEquals(constructedPositionStartOf2, MathTools.pow(0, 1), coefficients);

      DMatrixRMaj s2EndOf1 = new DMatrixRMaj(6, 1);
      DMatrixRMaj s2StartOf2 = new DMatrixRMaj(6, 1);

      DMatrixRMaj k2EndOf1Method1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj k2StartOf2Method1 = new DMatrixRMaj(3, 1);

      DMatrixRMaj actualS2 = new DMatrixRMaj(6, 1);
      DMatrixRMaj actualk2 = new DMatrixRMaj(6, 1);

      controller.computeS1AndK1(finalTime1);
      controller.computeS2AndK2(finalTime1);
      actualk2.set(controller.k2);
      actualS2.set(controller.getCostJacobian());

      // s2 at end of 1
      CommonOps_DDRM.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, controller.alphas.get(0), s2EndOf1);
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 0), controller.betas.get(0).get(0));
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 1), controller.betas.get(0).get(1));

      // k2 method 1 end of 1
      CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2EndOf1, k2EndOf1Method1);
      CommonOps_DDRM.multAdd(R1InvDQ, relativeMiddleVRP, k2EndOf1Method1);

      // s2 at start of 1
      CommonOps_DDRM.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, controller.alphas.get(1), s2StartOf2);
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 0), controller.betas.get(1).get(0));
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 1), controller.betas.get(1).get(1));

      // k2 method 1 start of 2
      CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2StartOf2, k2StartOf2Method1);
      CommonOps_DDRM.multAdd(R1InvDQ, relativeMiddleVRP, k2StartOf2Method1);

      EjmlUnitTests.assertEquals(s2EndOf1, s2StartOf2, epsilon);
      EjmlUnitTests.assertEquals(actualS2, s2StartOf2, epsilon);
      MatrixTestTools.assertMatrixEquals(relativeMiddleVRP, positionEndOf1, epsilon);
      MatrixTestTools.assertMatrixEquals(relativeMiddleVRP, positionStartOf2, epsilon);
      MatrixTestTools.assertMatrixEquals(relativeMiddleVRP, constructedPositionEndOf1, epsilon);
      MatrixTestTools.assertMatrixEquals(relativeMiddleVRP, constructedPositionStartOf2, epsilon);
      MatrixTestTools.assertMatrixEquals(k2EndOf1Method1, k2StartOf2Method1, epsilon);
      MatrixTestTools.assertMatrixEquals(k2EndOf1Method1, actualk2, epsilon);

      // check the actual trajectory for every time step of the first trajectory
      for (double time = 0; time <= finalTime1; time += 0.01)
      {
         controller.computeS1AndK1(time);
         controller.computeS2AndK2(time);

         CommonOps_DDRM.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha1Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 0), beta11Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 1), beta12Expected);

         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledA2, epsilon);
         EjmlUnitTests.assertEquals(matrixExponential, controller.A2Exponential, epsilon);
         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alpha1Expected, controller.alphas.get(0), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory1.compute(time);
         vrpTrajectory1.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalPosition);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);


         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      // check the actual trajectory for every time step of the second trajectory
      for (double time = finalTime1; time <= finalTime2; time += 0.01)
      {
         double localTime = time - finalTime1;
         controller.computeS1AndK1(time);
         controller.computeS2AndK2(time);

         CommonOps_DDRM.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha2Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 0), beta21Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 1), beta22Expected);

         EjmlUnitTests.assertEquals(alpha2Expected, controller.alphas.get(1), epsilon);

         MatrixTestTools.assertMatrixEquals("Beta 1 calculation failed at time " + time, beta21Expected, controller.betas.get(1).get(0), epsilon);
         MatrixTestTools.assertMatrixEquals("Beta 2 calculation failed at time " + time, beta22Expected, controller.betas.get(1).get(1), epsilon);
         MatrixTestTools.assertMatrixEquals("S2 calculation failed at time " + time, s2Expected, controller.getCostJacobian(), epsilon);


         // validate the feedback solution
         vrpTrajectory2.compute(localTime);
         vrpTrajectory2.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalPosition);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      controller.computeS2AndK2(finalTime2);

      // should be zero at tf
      DMatrixRMaj zeroMatrix = new DMatrixRMaj(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }


    */

   /*
   @Test
   public void testComputingS2FromThreeCubicTrajectories()
   {
      LQRMomentumController controller = new LQRMomentumController(() -> omega);

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpMiddle = new Point3D(0.6, 0.75, 0.87);
      Point3D vrpMiddle2 = new Point3D(0.79, 0.88, 0.95);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory1 = new Trajectory3D(4);
      Trajectory3D vrpTrajectory2 = new Trajectory3D(4);
      Trajectory3D vrpTrajectory3 = new Trajectory3D(4);
      double finalTime1 = 1.5;
      double finalTime2 = 3.1;
      double finalTime3 = 3.97;
      vrpTrajectory1.setCubic(0.0, finalTime1, vrpStart, vrpMiddle);
      vrpTrajectory2.setCubic(0.0, finalTime2 - finalTime1, vrpMiddle, vrpMiddle2);
      vrpTrajectory3.setCubic(0.0, finalTime3 - finalTime2, vrpMiddle2, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory1);
      trajectories.add(vrpTrajectory2);
      trajectories.add(vrpTrajectory3);

      DMatrixRMaj finalPosition = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalPosition);

      controller.setVRPTrajectory(trajectories);

      // double up on the computation to make sure things get zeroed properly between calls
      controller.computeS1();
      controller.computeS1();
      controller.computeS2Parameters();
      controller.computeS2Parameters();

      DMatrixRMaj NExpected = new DMatrixRMaj(6, 3);
      DMatrixRMaj NBExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj NTransposeExpected = new DMatrixRMaj(3, 6);
      DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(controller.Q, controller.D, tempMatrix);
      CommonOps_DDRM.multTransA(controller.C, tempMatrix, NExpected);
      CommonOps_DDRM.transpose(NExpected, NTransposeExpected);

      CommonOps_DDRM.multTransA(controller.B, controller.getCostHessian(), NBExpected);
      CommonOps_DDRM.addEquals(NBExpected, NTransposeExpected);

      LinearSolver<DMatrixRMaj> solver = LinearSolverFactory.general(0, 0);

      DMatrixRMaj A2Expected = new DMatrixRMaj(6, 6);
      DMatrixRMaj A2InverseExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj B2Expected = new DMatrixRMaj(6, 3);

      CommonOps_DDRM.transpose(controller.A, A2Expected);
      CommonOps_DDRM.scale(-1.0, A2Expected);

      tempMatrix.reshape(3, 6);
      CommonOps_DDRM.multTransB(controller.R1Inverse, controller.B, tempMatrix);
      CommonOps_DDRM.multAddTransA(NBExpected, tempMatrix, A2Expected);

      DMatrixRMaj tempMatrix2 = new DMatrixRMaj(6, 3);
      tempMatrix.reshape(6, 3);
      CommonOps_DDRM.multTransA(NBExpected, controller.R1Inverse, tempMatrix);
      CommonOps_DDRM.transpose(controller.C, tempMatrix2);
      CommonOps_DDRM.multAdd(-1.0, tempMatrix, controller.D, tempMatrix2);
      CommonOps_DDRM.mult(2.0, tempMatrix2, controller.Q, B2Expected);

      solver.setA(A2Expected);
      solver.invert(A2InverseExpected);

      EjmlUnitTests.assertEquals(NBExpected, controller.NB, epsilon);
      EjmlUnitTests.assertEquals(A2Expected, controller.A2, epsilon);
      EjmlUnitTests.assertEquals(B2Expected, controller.B2, epsilon);
      EjmlUnitTests.assertEquals(A2InverseExpected, controller.A2Inverse, epsilon);


      DMatrixRMaj A2InverseB2 = new DMatrixRMaj(6, 3);
      DMatrixRMaj R1InvDQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj DQ = new DMatrixRMaj(3, 3);
      DMatrixRMaj R1InvBTrans = new DMatrixRMaj(3, 6);
      DMatrixRMaj k2Method1 = new DMatrixRMaj(3, 1);
      DMatrixRMaj k2Method2 = new DMatrixRMaj(3, 1);
      DMatrixRMaj yd = new DMatrixRMaj(3, 1);

      CommonOps_DDRM.mult(A2InverseExpected, B2Expected, A2InverseB2);
      CommonOps_DDRM.mult(controller.D, controller.Q, DQ);
      CommonOps_DDRM.mult(controller.R1Inverse, DQ, R1InvDQ);
      CommonOps_DDRM.multTransB(controller.R1Inverse, controller.B, R1InvBTrans);

      assertEquals(3, controller.alphas.size());
      assertEquals(3, controller.betas.size());
      assertEquals(3, controller.gammas.size());
      assertEquals(4, controller.betas.get(0).size());
      assertEquals(4, controller.betas.get(1).size());
      assertEquals(4, controller.betas.get(2).size());
      assertEquals(4, controller.gammas.get(0).size());
      assertEquals(4, controller.gammas.get(1).size());
      assertEquals(4, controller.gammas.get(2).size());

      DMatrixRMaj beta11Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta12Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta13Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta14Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta21Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta22Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta23Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta24Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta31Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta32Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta33Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj beta34Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj gamma11Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma12Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma13Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma14Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma21Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma22Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma23Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma24Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma31Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma32Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma33Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj gamma34Expected = new DMatrixRMaj(3, 1);
      DMatrixRMaj coefficients = new DMatrixRMaj(3, 1);

      vrpTrajectory3.getCoefficients(3, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta34Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma34Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta34Expected, gamma34Expected);

      vrpTrajectory3.getCoefficients(2, coefficients);
      CommonOps_DDRM.mult(3, A2InverseExpected, beta34Expected, beta33Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta33Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma33Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta33Expected, gamma33Expected);

      vrpTrajectory3.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(2, A2InverseExpected, beta33Expected, beta32Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta32Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma32Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta32Expected, gamma32Expected);

      vrpTrajectory3.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta32Expected, beta31Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta31Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma31Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta31Expected, gamma31Expected);

      EjmlUnitTests.assertEquals(beta34Expected, controller.betas.get(2).get(3), epsilon);
      EjmlUnitTests.assertEquals(beta33Expected, controller.betas.get(2).get(2), epsilon);
      EjmlUnitTests.assertEquals(beta32Expected, controller.betas.get(2).get(1), epsilon);
      EjmlUnitTests.assertEquals(beta31Expected, controller.betas.get(2).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma34Expected, controller.gammas.get(2).get(3), epsilon);
      EjmlUnitTests.assertEquals(gamma33Expected, controller.gammas.get(2).get(2), epsilon);
      EjmlUnitTests.assertEquals(gamma32Expected, controller.gammas.get(2).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma31Expected, controller.gammas.get(2).get(0), epsilon);


      vrpTrajectory2.getCoefficients(3, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta24Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma24Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta24Expected, gamma24Expected);

      vrpTrajectory2.getCoefficients(2, coefficients);
      CommonOps_DDRM.mult(3, A2InverseExpected, beta24Expected, beta23Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta23Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma23Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta23Expected, gamma23Expected);

      vrpTrajectory2.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(2, A2InverseExpected, beta23Expected, beta22Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta22Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma22Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta22Expected, gamma22Expected);

      vrpTrajectory2.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta22Expected, beta21Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta21Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma21Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta21Expected, gamma21Expected);

      EjmlUnitTests.assertEquals(beta24Expected, controller.betas.get(1).get(3), epsilon);
      EjmlUnitTests.assertEquals(beta23Expected, controller.betas.get(1).get(2), epsilon);
      EjmlUnitTests.assertEquals(beta22Expected, controller.betas.get(1).get(1), epsilon);
      EjmlUnitTests.assertEquals(beta21Expected, controller.betas.get(1).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma24Expected, controller.gammas.get(1).get(3), epsilon);
      EjmlUnitTests.assertEquals(gamma23Expected, controller.gammas.get(1).get(2), epsilon);
      EjmlUnitTests.assertEquals(gamma22Expected, controller.gammas.get(1).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma21Expected, controller.gammas.get(1).get(0), epsilon);

      vrpTrajectory1.getCoefficients(3, coefficients);
      CommonOps_DDRM.mult(-1, A2InverseB2, coefficients, beta14Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma14Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta14Expected, gamma14Expected);

      vrpTrajectory1.getCoefficients(2, coefficients);
      CommonOps_DDRM.mult(3, A2InverseExpected, beta14Expected, beta13Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta13Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma13Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta13Expected, gamma13Expected);

      vrpTrajectory1.getCoefficients(1, coefficients);
      CommonOps_DDRM.mult(2, A2InverseExpected, beta13Expected, beta12Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta12Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma12Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta12Expected, gamma12Expected);

      vrpTrajectory1.getCoefficients(0, coefficients);
      CommonOps_DDRM.subtractEquals(coefficients, finalPosition);
      CommonOps_DDRM.mult(A2InverseExpected, beta12Expected, beta11Expected);
      CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, beta11Expected);

      CommonOps_DDRM.mult(R1InvDQ, coefficients, gamma11Expected);
      CommonOps_DDRM.multAdd(-0.5, R1InvBTrans, beta11Expected, gamma11Expected);

      EjmlUnitTests.assertEquals(beta14Expected, controller.betas.get(0).get(3), epsilon);
      EjmlUnitTests.assertEquals(gamma14Expected, controller.gammas.get(0).get(3), epsilon);
      EjmlUnitTests.assertEquals(beta13Expected, controller.betas.get(0).get(2), epsilon);
      EjmlUnitTests.assertEquals(gamma13Expected, controller.gammas.get(0).get(2), epsilon);
      EjmlUnitTests.assertEquals(beta12Expected, controller.betas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(gamma12Expected, controller.gammas.get(0).get(1), epsilon);
      EjmlUnitTests.assertEquals(beta11Expected, controller.betas.get(0).get(0), epsilon);
      EjmlUnitTests.assertEquals(gamma11Expected, controller.gammas.get(0).get(0), epsilon);

      MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

      DMatrixRMaj alpha3Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj alpha2Expected = new DMatrixRMaj(6, 1);
      DMatrixRMaj alpha1Expected = new DMatrixRMaj(6, 1);

      DMatrixRMaj timeScaledDynamics = new DMatrixRMaj(6, 6);
      DMatrixRMaj matrixExponential = new DMatrixRMaj(6, 6);
      DMatrixRMaj betaSum = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.scale((finalTime3 - finalTime2), A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 0), beta31Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 1), beta32Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 2), beta33Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime3 - finalTime2, 3), beta34Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha3Expected);

      CommonOps_DDRM.scale((finalTime2 - finalTime1), A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      betaSum.zero();
      CommonOps_DDRM.addEquals(betaSum, alpha3Expected);
      CommonOps_DDRM.addEquals(betaSum, beta31Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 0), beta21Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 1), beta22Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 2), beta23Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime2 - finalTime1, 3), beta24Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha2Expected);

      CommonOps_DDRM.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      betaSum.zero();
      CommonOps_DDRM.addEquals(betaSum, alpha2Expected);
      CommonOps_DDRM.addEquals(betaSum, beta21Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 0), beta11Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 1), beta12Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 2), beta13Expected);
      CommonOps_DDRM.addEquals(betaSum, -1.0 * MathTools.pow(finalTime1, 3), beta14Expected);

      solver.setA(matrixExponential);
      solver.solve(betaSum, alpha1Expected);

      EjmlUnitTests.assertEquals(alpha3Expected, controller.alphas.get(2), epsilon);
      EjmlUnitTests.assertEquals(alpha2Expected, controller.alphas.get(1), epsilon);
      EjmlUnitTests.assertEquals(alpha1Expected, controller.alphas.get(0), epsilon);



      // check for continuity

      DMatrixRMaj s2EndOf1 = new DMatrixRMaj(6, 1);
      DMatrixRMaj s2StartOf2 = new DMatrixRMaj(6, 1);
      DMatrixRMaj s2EndOf2 = new DMatrixRMaj(6, 1);
      DMatrixRMaj s2StartOf3 = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.scale(finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, controller.alphas.get(0), s2EndOf1);
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 0), controller.betas.get(0).get(0));
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 1), controller.betas.get(0).get(1));
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 2), controller.betas.get(0).get(2));
      CommonOps_DDRM.addEquals(s2EndOf1, MathTools.pow(finalTime1, 3), controller.betas.get(0).get(3));

      CommonOps_DDRM.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, controller.alphas.get(1), s2StartOf2);
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 0), controller.betas.get(1).get(0));
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 1), controller.betas.get(1).get(1));
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 2), controller.betas.get(1).get(2));
      CommonOps_DDRM.addEquals(s2StartOf2, MathTools.pow(0, 2), controller.betas.get(1).get(3));

      EjmlUnitTests.assertEquals(s2EndOf1, s2StartOf2, epsilon);


      CommonOps_DDRM.scale(finalTime2 - finalTime1, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, controller.alphas.get(1), s2EndOf2);
      CommonOps_DDRM.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 0), controller.betas.get(1).get(0));
      CommonOps_DDRM.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 1), controller.betas.get(1).get(1));
      CommonOps_DDRM.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 2), controller.betas.get(1).get(2));
      CommonOps_DDRM.addEquals(s2EndOf2, MathTools.pow(finalTime2 - finalTime1, 3), controller.betas.get(1).get(3));

      CommonOps_DDRM.scale(0, A2Expected, timeScaledDynamics);
      matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);
      CommonOps_DDRM.mult(matrixExponential, controller.alphas.get(2), s2StartOf3);
      CommonOps_DDRM.addEquals(s2StartOf3, MathTools.pow(0, 0), controller.betas.get(2).get(0));
      CommonOps_DDRM.addEquals(s2StartOf3, MathTools.pow(0, 1), controller.betas.get(2).get(1));
      CommonOps_DDRM.addEquals(s2StartOf3, MathTools.pow(0, 2), controller.betas.get(2).get(2));
      CommonOps_DDRM.addEquals(s2StartOf3, MathTools.pow(0, 3), controller.betas.get(2).get(3));

      EjmlUnitTests.assertEquals(s2EndOf2, s2StartOf3, epsilon);

      DMatrixRMaj finalY = new DMatrixRMaj(3, 1);
      vrpEnd.get(finalY);

      // check trajectory

      for (double time = 0; time <= finalTime1; time += 0.01)
      {
         controller.computeS2(time);

         CommonOps_DDRM.scale(time, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha1Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 0), beta11Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 1), beta12Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 2), beta13Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(time, 3), beta14Expected);

         EjmlUnitTests.assertEquals(timeScaledDynamics, controller.timeScaledA2, epsilon);
         EjmlUnitTests.assertEquals(matrixExponential, controller.A2Exponential, epsilon);
         EjmlUnitTests.assertEquals(betaSum, controller.summedBetas, epsilon);
         EjmlUnitTests.assertEquals(alpha1Expected, controller.alphas.get(0), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory1.compute(time);
         vrpTrajectory1.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalY);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, controller.alphas.get(0), k2Method2);

         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 0), controller.gammas.get(0).get(0));
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 1), controller.gammas.get(0).get(1));
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 2), controller.gammas.get(0).get(2));
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(time, 3), controller.gammas.get(0).get(3));

         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      for (double time = finalTime1; time <= finalTime2; time += 0.01)
      {
         int j = 1;
         double localTime = time - finalTime1;
         controller.computeS2(time);

         CommonOps_DDRM.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha2Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 0), beta21Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 1), beta22Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 2), beta23Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 3), beta24Expected);

         EjmlUnitTests.assertEquals(alpha2Expected, controller.alphas.get(1), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory2.compute(localTime);
         vrpTrajectory2.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalY);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, controller.alphas.get(1), k2Method2);

         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 0), controller.gammas.get(1).get(0));
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 1), controller.gammas.get(1).get(1));
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 2), controller.gammas.get(1).get(2));
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 3), controller.gammas.get(1).get(3));

         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      for (double time = finalTime2; time <= finalTime3; time += 0.01)
      {
         double localTime = time - finalTime2;
         controller.computeS2(time);

         CommonOps_DDRM.scale(localTime, A2Expected, timeScaledDynamics);
         matrixExponentialCalculator.compute(matrixExponential, timeScaledDynamics);

         DMatrixRMaj s2Expected = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(matrixExponential, alpha3Expected, s2Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 0), beta31Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 1), beta32Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 2), beta33Expected);
         CommonOps_DDRM.addEquals(s2Expected, MathTools.pow(localTime, 3), beta34Expected);

         EjmlUnitTests.assertEquals(alpha3Expected, controller.alphas.get(2), epsilon);

         EjmlUnitTests.assertEquals(s2Expected, controller.getCostJacobian(), epsilon);

         vrpTrajectory3.compute(localTime);
         vrpTrajectory3.getPosition().get(yd);
         CommonOps_DDRM.subtractEquals(yd, finalY);

         CommonOps_DDRM.mult(-0.5, R1InvBTrans, s2Expected, k2Method1);
         CommonOps_DDRM.multAdd(R1InvDQ, yd, k2Method1);

         tempMatrix.reshape(3, 6);
         CommonOps_DDRM.mult(-0.5, R1InvBTrans, matrixExponential, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, controller.alphas.get(2), k2Method2);

         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 0), controller.gammas.get(2).get(0));
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 1), controller.gammas.get(2).get(1));
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 2), controller.gammas.get(2).get(2));
         CommonOps_DDRM.addEquals(k2Method2, MathTools.pow(localTime, 3), controller.gammas.get(2).get(3));

         DMatrixRMaj K1Expected = new DMatrixRMaj(3, 6);
         CommonOps_DDRM.mult(-1.0, controller.R1Inverse, NBExpected, K1Expected);

         EjmlUnitTests.assertEquals(k2Method1, k2Method2, epsilon);
         MatrixTestTools.assertMatrixEquals(k2Method1, controller.k2, epsilon);
         MatrixTestTools.assertMatrixEquals(K1Expected, controller.K1, epsilon);
      }

      controller.computeS2(finalTime3);

      // should be zero at tf
      DMatrixRMaj zeroMatrix = new DMatrixRMaj(6, 1);
      EjmlUnitTests.assertEquals(zeroMatrix, controller.getCostJacobian(), epsilon);
   }

   @Test
   public void testBasicTrajectoryTracking()
   {
      SimpleCoMTrajectoryPlanner planner = new SimpleCoMTrajectoryPlanner(() -> omega);
      LQRMomentumController controller = new LQRMomentumController(() -> omega);

      List<SettableContactStateProvider> contactSequence = new ArrayList<>();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();
      SettableContactStateProvider thirdContact = new SettableContactStateProvider();

      FramePoint2D vrpStart = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      FramePoint2D vrpMiddle = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.6, 0.75);
      FramePoint2D vrpMiddle2 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.79, 0.88);
      FramePoint2D vrpEnd = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, 0.5);

      double finalTime1 = 0.31;
      double finalTime2 = 0.65;
      double finalTime3 = 1.0;

      firstContact.getTimeInterval().setInterval(0.0, finalTime1);
      firstContact.setStartCopPosition(vrpStart);
      firstContact.setEndCopPosition(vrpMiddle);

      secondContact.getTimeInterval().setInterval(finalTime1, finalTime2);
      secondContact.setStartCopPosition(vrpMiddle);
      secondContact.setEndCopPosition(vrpMiddle2);

      thirdContact.getTimeInterval().setInterval(finalTime2, finalTime3);
      thirdContact.setStartCopPosition(vrpMiddle2);
      thirdContact.setEndCopPosition(vrpEnd);

      contactSequence.add(firstContact);
//      contactSequence.add(secondContact);
      //      contactSequence.add(thirdContact);

      double nominalHeight = 1.0;
      FramePoint3D initialComPosition = new FramePoint3D();
      initialComPosition.setZ(nominalHeight);
      planner.setNominalCoMHeight(nominalHeight);
      planner.setInitialCenterOfMassState(initialComPosition, new FrameVector3D());
      planner.solveForTrajectory(contactSequence);
      double time = 0.0;
      planner.compute(time);

      controller.setVRPTrajectory(planner.getVRPTrajectories());

      double dt = 5e-7;
      FramePoint3D comPosition = new FramePoint3D(planner.getDesiredCoMPosition());
      FrameVector3D comVelocity = new FrameVector3D(planner.getDesiredCoMVelocity());
      FrameVector3D previousVelocity = new FrameVector3D();
      FrameVector3D previousAcceleration = new FrameVector3D();

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      comPosition.get(currentState);
      comVelocity.get(3, currentState);
      controller.computeControlInput(currentState, time);


      for (; time <= finalTime1; time += dt)
      {
         comPosition.get(currentState);
         comVelocity.get(3, currentState);

         controller.computeControlInput(currentState, time);

         FrameVector3D acceleration = new FrameVector3D();
         acceleration.set(controller.getU());

         comVelocity.scaleAdd(0.5 * dt, previousAcceleration, comVelocity);
         comVelocity.scaleAdd(0.5 * dt, acceleration, comVelocity);
         comPosition.scaleAdd(0.5 * dt, previousVelocity, comPosition);
         comPosition.scaleAdd(0.5 * dt, comVelocity, comPosition);

         previousAcceleration.set(acceleration);
         previousVelocity.set(comVelocity);
      }

      FramePoint3D finalPosition = new FramePoint3D(vrpMiddle);
      finalPosition.addZ(nominalHeight);

      FramePoint3D icpPosition = new FramePoint3D();
      icpPosition.scaleAdd(1.0 / omega, comVelocity, comPosition);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalPosition, icpPosition, 1e-5);
   }

    */
}
