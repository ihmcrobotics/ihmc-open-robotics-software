package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.EuclideanModelPredictiveController;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.robotics.Assert.assertTrue;

public class LinearMPCQPSolverTest
{
   @Test
   public void testCommandOptimize()
   {
      FramePoint3D dcmObjective = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);
      FrameVector3D comStartVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.1, 0.2, 0.4);
      FramePoint3D comStartPosition = new FramePoint3D();

      double gravityZ = -9.81;
      double omega = 3.0;
      double omega2 = omega * omega;
      double mu = 0.8;
      double dt = 1e-3;
      double duration = 1.0;

      double timeAtStart = 0.0;
      double timeAtEnd = duration;

      double regularization = 1e-8;


      comStartPosition.scaleAdd(-1.0 / omega, comStartVelocity, dcmObjective);


      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());

      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(dcmObjective);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();
      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, mu);
      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);


      VRPPositionCommand vrpStartPositionCommand = new VRPPositionCommand();
      vrpStartPositionCommand.setObjective(dcmObjective);
      vrpStartPositionCommand.setTimeOfObjective(timeAtStart);
      vrpStartPositionCommand.setSegmentNumber(0);
      vrpStartPositionCommand.setOmega(omega);
      vrpStartPositionCommand.setWeight(1.0);
      vrpStartPositionCommand.addContactPlaneHelper(contactPlaneHelper);

      VRPVelocityCommand vrpStartVelocityCommand = new VRPVelocityCommand();
      vrpStartVelocityCommand.setObjective(new FrameVector3D());
      vrpStartVelocityCommand.setTimeOfObjective(timeAtStart);
      vrpStartVelocityCommand.setSegmentNumber(0);
      vrpStartVelocityCommand.setOmega(omega);
      vrpStartVelocityCommand.setWeight(1.0);
      vrpStartVelocityCommand.addContactPlaneHelper(contactPlaneHelper);

      VRPPositionCommand vrpEndPositionCommand = new VRPPositionCommand();
      vrpEndPositionCommand.setObjective(dcmObjective);
      vrpEndPositionCommand.setTimeOfObjective(timeAtEnd);
      vrpEndPositionCommand.setSegmentNumber(0);
      vrpEndPositionCommand.setOmega(omega);
      vrpEndPositionCommand.setWeight(1.0);
      vrpEndPositionCommand.addContactPlaneHelper(contactPlaneHelper);

      VRPVelocityCommand vrpEndVelocityCommand = new VRPVelocityCommand();
      vrpEndVelocityCommand.setObjective(new FrameVector3D());
      vrpEndVelocityCommand.setTimeOfObjective(timeAtEnd);
      vrpEndVelocityCommand.setSegmentNumber(0);
      vrpEndVelocityCommand.setOmega(omega);
      vrpEndVelocityCommand.setWeight(1.0);
      vrpEndVelocityCommand.addContactPlaneHelper(contactPlaneHelper);


      CoMPositionCommand comStartPositionCommand = new CoMPositionCommand();
      comStartPositionCommand.setObjective(comStartPosition);
      comStartPositionCommand.setTimeOfObjective(timeAtStart);
      comStartPositionCommand.setSegmentNumber(0);
      comStartPositionCommand.setOmega(omega);
      comStartPositionCommand.setWeight(1.0);
      comStartPositionCommand.addContactPlaneHelper(contactPlaneHelper);


      CoMVelocityCommand comStartVelocityCommand = new CoMVelocityCommand();
      comStartVelocityCommand.setObjective(comStartVelocity);
      comStartVelocityCommand.setTimeOfObjective(timeAtStart);
      comStartVelocityCommand.setSegmentNumber(0);
      comStartVelocityCommand.setOmega(omega);
      comStartVelocityCommand.setWeight(1.0);
      comStartVelocityCommand.addContactPlaneHelper(contactPlaneHelper);

      DCMPositionCommand dcmEndPositionCommand = new DCMPositionCommand();
      dcmEndPositionCommand.setObjective(dcmObjective);
      dcmEndPositionCommand.setTimeOfObjective(timeAtEnd);
      dcmEndPositionCommand.setSegmentNumber(0);
      dcmEndPositionCommand.setOmega(omega);
      dcmEndPositionCommand.setWeight(1.0);
      dcmEndPositionCommand.addContactPlaneHelper(contactPlaneHelper);


      DMatrixRMaj solverH_Expected = new DMatrixRMaj(indexHandler.getTotalProblemSize(), indexHandler.getTotalProblemSize());
      DMatrixRMaj solverf_Expected = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);

      DMatrixRMaj expectedVRPStartPositionObjective = new DMatrixRMaj(3, 1);
      DMatrixRMaj expectedVRPStartVelocityObjective = new DMatrixRMaj(3, 1);
      DMatrixRMaj expectedVRPEndPositionObjective = new DMatrixRMaj(3, 1);
      DMatrixRMaj expectedVRPEndVelocityObjective = new DMatrixRMaj(3, 1);
      DMatrixRMaj expectedCoMStartPositionObjective = new DMatrixRMaj(3, 1);
      DMatrixRMaj expectedCoMStartVelocityObjective = new DMatrixRMaj(3, 1);
      DMatrixRMaj expectedDCMEndPositionObjective = new DMatrixRMaj(3, 1);

      dcmObjective.get(expectedVRPStartPositionObjective);
      expectedVRPStartPositionObjective.add(2, 0, 0.5 * timeAtStart * timeAtStart * Math.abs(gravityZ));
      expectedVRPStartPositionObjective.add(2, 0, -1.0 / omega2 * Math.abs(gravityZ));

      expectedVRPStartVelocityObjective.add(2, 0, timeAtStart * Math.abs(gravityZ));

      dcmObjective.get(expectedVRPEndPositionObjective);
      expectedVRPEndPositionObjective.add(2, 0, 0.5 * timeAtEnd * timeAtEnd * Math.abs(gravityZ));
      expectedVRPEndPositionObjective.add(2, 0, -1.0 / omega2 * Math.abs(gravityZ));

      expectedVRPEndVelocityObjective.add(2, 0, timeAtEnd * Math.abs(gravityZ));

      comStartPosition.get(expectedCoMStartPositionObjective);
      expectedCoMStartPositionObjective.add(2, 0, 0.5 * timeAtStart * timeAtStart * Math.abs(gravityZ));

      comStartVelocity.get(expectedCoMStartVelocityObjective);
      expectedCoMStartVelocityObjective.add(2, 0, timeAtStart * Math.abs(gravityZ));

      dcmObjective.get(expectedDCMEndPositionObjective);
      expectedDCMEndPositionObjective.add(2, 0, 0.5 * timeAtEnd * timeAtEnd * Math.abs(gravityZ));
      expectedDCMEndPositionObjective.add(2, 0, 1.0 / omega * timeAtEnd * Math.abs(gravityZ));

      solver.initialize();

      solver.submitMPCValueObjective(vrpStartPositionCommand);
      EjmlUnitTests.assertEquals(expectedVRPStartPositionObjective, solver.qpInputTypeA.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getVRPPositionJacobian(timeAtStart, omega, rhoHelper), solver.qpInputTypeA.taskJacobian, 1e-4);
      addTask(solver.qpInputTypeA, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(vrpStartVelocityCommand);
      EjmlUnitTests.assertEquals(expectedVRPStartVelocityObjective, solver.qpInputTypeA.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getVRPVelocityJacobian(timeAtStart, omega, rhoHelper), solver.qpInputTypeA.taskJacobian, 1e-4);
      addTask(solver.qpInputTypeA, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(vrpEndPositionCommand);
      EjmlUnitTests.assertEquals(expectedVRPEndPositionObjective, solver.qpInputTypeA.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getVRPPositionJacobian(timeAtEnd, omega, rhoHelper), solver.qpInputTypeA.taskJacobian, 1e-4);
      addTask(solver.qpInputTypeA, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(vrpEndVelocityCommand);
      EjmlUnitTests.assertEquals(expectedVRPEndVelocityObjective, solver.qpInputTypeA.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getVRPVelocityJacobian(timeAtEnd, omega, rhoHelper), solver.qpInputTypeA.taskJacobian, 1e-4);
      addTask(solver.qpInputTypeA, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(comStartPositionCommand);
      EjmlUnitTests.assertEquals(expectedCoMStartPositionObjective, solver.qpInputTypeA.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getCoMPositionJacobian(timeAtStart, omega, rhoHelper), solver.qpInputTypeA.taskJacobian, 1e-4);
      addTask(solver.qpInputTypeA, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(comStartVelocityCommand);
      EjmlUnitTests.assertEquals(expectedCoMStartVelocityObjective, solver.qpInputTypeA.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getCoMVelocityJacobian(timeAtStart, omega, rhoHelper), solver.qpInputTypeA.taskJacobian, 1e-4);
      addTask(solver.qpInputTypeA, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(dcmEndPositionCommand);
      EjmlUnitTests.assertEquals(expectedDCMEndPositionObjective, solver.qpInputTypeA.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getDCMPositionJacobian(timeAtEnd, omega, rhoHelper), solver.qpInputTypeA.taskJacobian, 1e-4);
      addTask(solver.qpInputTypeA, solverH_Expected, solverf_Expected);

      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      FramePoint3D reconstructedCoMPositionAtStart = new FramePoint3D();
      FramePoint3D reconstructedCoMVelocityAtStart = new FramePoint3D();
      FramePoint3D reconstructedDCMAtStart = new FramePoint3D();
      FramePoint3D reconstructedDCMAtEnd = new FramePoint3D();
      FramePoint3D reconstructedVRPAtStart = new FramePoint3D();
      FramePoint3D reconstructedVRPAtEnd = new FramePoint3D();

      DMatrixRMaj solution = solver.getSolution();

      MatrixTools.addDiagonal(solverH_Expected, regularization);
      EjmlUnitTests.assertEquals(solverH_Expected, solver.solverInput_H, 1e-6);
      EjmlUnitTests.assertEquals(solverf_Expected, solver.solverInput_f, 1e-6);

      double c0_Start = timeAtStart;
      double c1_Start = 1.0;
      double c2_Start = Math.exp(omega * timeAtStart);
      double c3_Start = Math.exp(-omega * timeAtStart);
      double c4_Start = timeAtStart * timeAtStart * timeAtStart;
      double c5_Start = timeAtStart * timeAtStart;

      double c0Dot_Start = 1.0;
      double c1Dot_Start = 0.0;
      double c2Dot_Start = omega * Math.exp(omega * timeAtStart);
      double c3Dot_Start = -omega * Math.exp(-omega * timeAtStart);
      double c4Dot_Start = 3.0 * timeAtStart * timeAtStart;
      double c5Dot_Start = 2.0 * timeAtStart;

      double c0Ddot_Start = 0.0;
      double c1Ddot_Start = 0.0;
      double c2Ddot_Start = omega * omega * Math.exp(omega * timeAtStart);
      double c3Ddot_Start = omega * omega * Math.exp(-omega * timeAtStart);
      double c4Ddot_Start = 6.0 * timeAtStart;
      double c5Ddot_Start = 2.0;

      double c0_End = timeAtEnd;
      double c1_End = 1.0;
      double c2_End = Math.exp(omega * timeAtEnd);
      double c3_End = Math.exp(-omega * timeAtEnd);
      double c4_End = timeAtEnd * timeAtEnd * timeAtEnd;
      double c5_End = timeAtEnd * timeAtEnd;

      double c0Dot_End = 1.0;
      double c1Dot_End = 0.0;
      double c2Dot_End = omega * Math.exp(omega * timeAtEnd);
      double c3Dot_End = -omega * Math.exp(-omega * timeAtEnd);
      double c4Dot_End = 3.0 * timeAtEnd * timeAtEnd;
      double c5Dot_End = 2.0 * timeAtEnd;

      double c0Ddot_End = 0.0;
      double c1Ddot_End = 0.0;
      double c2Ddot_End = omega * omega * Math.exp(omega * timeAtEnd);
      double c3Ddot_End = omega * omega * Math.exp(-omega * timeAtEnd);
      double c4Ddot_End = 6.0 * timeAtEnd;
      double c5Ddot_End = 2.0;

      reconstructedCoMPositionAtStart.addX(c0_Start * solution.get(0, 0) + c1_Start * solution.get(1, 0));
      reconstructedCoMPositionAtStart.addY(c0_Start * solution.get(2, 0) + c1_Start * solution.get(3, 0));
      reconstructedCoMPositionAtStart.addZ(c0_Start * solution.get(4, 0) + c1_Start * solution.get(5, 0));

      reconstructedCoMVelocityAtStart.addX(c0Dot_Start * solution.get(0, 0) + c1Dot_Start * solution.get(1, 0));
      reconstructedCoMVelocityAtStart.addY(c0Dot_Start * solution.get(2, 0) + c1Dot_Start * solution.get(3, 0));
      reconstructedCoMVelocityAtStart.addZ(c0Dot_Start * solution.get(4, 0) + c1Dot_Start * solution.get(5, 0));

      reconstructedDCMAtStart.addX((c0_Start + 1.0 / omega * c0Dot_Start) * solution.get(0, 0) + (c1_Start + 1.0 / omega * c1Dot_Start) * solution.get(1, 0));
      reconstructedDCMAtStart.addY((c0_Start + 1.0 / omega * c0Dot_Start) * solution.get(2, 0) + (c1_Start + 1.0 / omega * c1Dot_Start) * solution.get(3, 0));
      reconstructedDCMAtStart.addZ((c0_Start + 1.0 / omega * c0Dot_Start) * solution.get(4, 0) + (c1_Start + 1.0 / omega * c1Dot_Start) * solution.get(5, 0));

      reconstructedVRPAtStart.addX((c0_Start - 1.0 / omega2 * c0Ddot_Start) * solution.get(0, 0) + (c1_Start - 1.0 / omega2 * c1Ddot_Start) * solution.get(1, 0));
      reconstructedVRPAtStart.addY((c0_Start - 1.0 / omega2 * c0Ddot_Start) * solution.get(2, 0) + (c1_Start - 1.0 / omega2 * c1Ddot_Start) * solution.get(3, 0));
      reconstructedVRPAtStart.addZ((c0_Start - 1.0 / omega2 * c0Ddot_Start) * solution.get(4, 0) + (c1_Start - 1.0 / omega2 * c1Ddot_Start) * solution.get(5, 0));

      reconstructedDCMAtEnd.addX((c0_End + 1.0 / omega * c0Dot_End) * solution.get(0, 0) + (c1_End + 1.0 / omega * c1Dot_End) * solution.get(1, 0));
      reconstructedDCMAtEnd.addY((c0_End + 1.0 / omega * c0Dot_End) * solution.get(2, 0) + (c1_End + 1.0 / omega * c1Dot_End) * solution.get(3, 0));
      reconstructedDCMAtEnd.addZ((c0_End + 1.0 / omega * c0Dot_End) * solution.get(4, 0) + (c1_End + 1.0 / omega * c1Dot_End) * solution.get(5, 0));

      reconstructedVRPAtEnd.addX((c0_End - 1.0 / omega2 * c0Ddot_End) * solution.get(0, 0) + (c1_End - 1.0 / omega2 * c1Ddot_End) * solution.get(1, 0));
      reconstructedVRPAtEnd.addY((c0_End - 1.0 / omega2 * c0Ddot_End) * solution.get(2, 0) + (c1_End - 1.0 / omega2 * c1Ddot_End) * solution.get(3, 0));
      reconstructedVRPAtEnd.addZ((c0_End - 1.0 / omega2 * c0Ddot_End) * solution.get(4, 0) + (c1_End - 1.0 / omega2 * c1Ddot_End) * solution.get(5, 0));

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = indexHandler.getRhoCoefficientStartIndex(0) + 4 * rhoIdx;

         double rhoStartPositionValue = c2_Start * solution.get(startIdx , 0);
         rhoStartPositionValue += c3_Start * solution.get(startIdx + 1, 0);
         rhoStartPositionValue += c4_Start * solution.get(startIdx + 2, 0);
         rhoStartPositionValue += c5_Start * solution.get(startIdx + 3, 0);

         double rhoStartVelocityValue = c2Dot_Start * solution.get(startIdx , 0);
         rhoStartVelocityValue += c3Dot_Start * solution.get(startIdx + 1, 0);
         rhoStartVelocityValue += c4Dot_Start * solution.get(startIdx + 2, 0);
         rhoStartVelocityValue += c5Dot_Start * solution.get(startIdx + 3, 0);

         double rhoStartAccelerationValue = c2Ddot_Start * solution.get(startIdx , 0);
         rhoStartAccelerationValue += c3Ddot_Start * solution.get(startIdx + 1, 0);
         rhoStartAccelerationValue += c4Ddot_Start * solution.get(startIdx + 2, 0);
         rhoStartAccelerationValue += c5Ddot_Start * solution.get(startIdx + 3, 0);

         double rhoEndPositionValue = c2_End * solution.get(startIdx , 0);
         rhoEndPositionValue += c3_End * solution.get(startIdx + 1, 0);
         rhoEndPositionValue += c4_End * solution.get(startIdx + 2, 0);
         rhoEndPositionValue += c5_End * solution.get(startIdx + 3, 0);

         double rhoEndVelocityValue = c2Dot_End * solution.get(startIdx , 0);
         rhoEndVelocityValue += c3Dot_End * solution.get(startIdx + 1, 0);
         rhoEndVelocityValue += c4Dot_End * solution.get(startIdx + 2, 0);
         rhoEndVelocityValue += c5Dot_End * solution.get(startIdx + 3, 0);

         double rhoEndAccelerationValue = c2Ddot_End * solution.get(startIdx , 0);
         rhoEndAccelerationValue += c3Ddot_End * solution.get(startIdx + 1, 0);
         rhoEndAccelerationValue += c4Ddot_End * solution.get(startIdx + 2, 0);
         rhoEndAccelerationValue += c5Ddot_End * solution.get(startIdx + 3, 0);

         reconstructedCoMPositionAtStart.scaleAdd(rhoStartPositionValue, rhoHelper.getBasisVector(rhoIdx), reconstructedCoMPositionAtStart);
         reconstructedCoMVelocityAtStart.scaleAdd(rhoStartVelocityValue, rhoHelper.getBasisVector(rhoIdx), reconstructedCoMVelocityAtStart);
         reconstructedDCMAtStart.scaleAdd((rhoStartPositionValue + 1.0 / omega * rhoStartVelocityValue), rhoHelper.getBasisVector(rhoIdx), reconstructedDCMAtStart);
         reconstructedVRPAtStart.scaleAdd((rhoStartPositionValue - 1.0 / omega2 * rhoStartAccelerationValue), rhoHelper.getBasisVector(rhoIdx), reconstructedVRPAtStart);
         reconstructedDCMAtEnd.scaleAdd((rhoEndPositionValue + 1.0 / omega * rhoEndVelocityValue), rhoHelper.getBasisVector(rhoIdx), reconstructedDCMAtEnd);
         reconstructedVRPAtEnd.scaleAdd((rhoEndPositionValue - 1.0 / omega2 * rhoEndAccelerationValue), rhoHelper.getBasisVector(rhoIdx), reconstructedVRPAtEnd);
      }

      double gravityPositionAtStart = 0.5 * timeAtStart * timeAtStart;
      double gravityVelocityAtStart = timeAtStart;
      double gravityAccelerationAtStart = 1.0;

      double gravityPositionAtEnd = 0.5 * timeAtEnd * timeAtEnd;
      double gravityVelocityAtEnd = timeAtEnd;
      double gravityAccelerationAtEnd = 1.0;

      reconstructedCoMPositionAtStart.scaleAdd(gravityPositionAtStart, gravityVector, reconstructedCoMPositionAtStart);
      reconstructedCoMVelocityAtStart.scaleAdd(gravityVelocityAtStart, gravityVector, reconstructedCoMVelocityAtStart);
      reconstructedDCMAtStart.scaleAdd(gravityPositionAtStart + 1.0 / omega * gravityVelocityAtStart, gravityVector, reconstructedDCMAtStart);
      reconstructedDCMAtEnd.scaleAdd(gravityPositionAtEnd + 1.0 / omega * gravityVelocityAtEnd, gravityVector, reconstructedDCMAtEnd);
      reconstructedVRPAtStart.scaleAdd(gravityPositionAtStart - 1.0 / omega2 * gravityAccelerationAtStart, gravityVector, reconstructedVRPAtStart);
      reconstructedVRPAtEnd.scaleAdd(gravityPositionAtEnd - 1.0 / omega2 * gravityAccelerationAtEnd, gravityVector, reconstructedVRPAtEnd);


      EuclidCoreTestTools.assertTuple3DEquals(comStartPosition, reconstructedCoMPositionAtStart, 5e-3);
      EuclidCoreTestTools.assertTuple3DEquals(comStartVelocity, reconstructedCoMVelocityAtStart, 5e-3);
      EuclidCoreTestTools.assertTuple3DEquals(dcmObjective, reconstructedDCMAtStart, 5e-3);
      EuclidCoreTestTools.assertTuple3DEquals(dcmObjective, reconstructedDCMAtEnd, 5e-3);
      EuclidCoreTestTools.assertTuple3DEquals(dcmObjective, reconstructedVRPAtStart, 7e-3);
      EuclidCoreTestTools.assertTuple3DEquals(dcmObjective, reconstructedVRPAtEnd, 8e-3);
   }

   @Test
   public void testJumpWithLowerAndUpperBounds()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      int bases = 3;
      MPCContactPlane contactPlaneHelper1 = new MPCContactPlane(4, bases, new ZeroConeRotationCalculator());
      MPCContactPlane contactPlaneHelper3 = new MPCContactPlane(4, bases, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(bases);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose = new FramePose3D();

//      ConvexPolygon2D contactPolygon = new ConvexPolygon2D();
//      contactPolygon.addVertex(0.0, 0.0);
//      contactPolygon.update();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper1.computeBasisVectors(contactPolygon, contactPose, mu);
      contactPlaneHelper3.computeBasisVectors(contactPolygon, contactPose, mu);

      FramePoint3D startPosition = new FramePoint3D(contactPose.getPosition());
      startPosition.setZ(0.75);

      FramePoint3D endPosition = new FramePoint3D(contactPose.getPosition());
      endPosition.setZ(1.0);

      FrameVector3D zeroVelocity = new FrameVector3D();


      indexHandler.initialize(
            i ->
            {
               if (i == 1)
                  return 0;
               else
                  return contactPolygon.getNumberOfVertices();
            }, 3);

      double firstSegmentDuration = 0.7;
      double secondSegmentDuration = 0.5;
      double thirdSegmentDuration = 0.9;
      double minRho = 0.001;
      double maxGs = 2.5;
      double maxForce = maxGs * Math.abs(gravityZ);
      double maxRho = maxForce / (contactPolygon.getNumberOfVertices() * bases) / mu;

      RhoBoundCommand rhoMinBoundsSegment1 = new RhoBoundCommand();
      rhoMinBoundsSegment1.setOmega(omega);
      rhoMinBoundsSegment1.setSegmentDuration(firstSegmentDuration);
      rhoMinBoundsSegment1.setSegmentNumber(0);
      rhoMinBoundsSegment1.addContactPlane(contactPlaneHelper1, minRho);
      rhoMinBoundsSegment1.setConstraintType(ConstraintType.GEQ_INEQUALITY);


      RhoBoundCommand rhoMinBoundsSegment3 = new RhoBoundCommand();
      rhoMinBoundsSegment3.setOmega(omega);
      rhoMinBoundsSegment3.setSegmentDuration(thirdSegmentDuration);
      rhoMinBoundsSegment3.setSegmentNumber(2);
      rhoMinBoundsSegment3.addContactPlane(contactPlaneHelper3, minRho);
      rhoMinBoundsSegment3.setConstraintType(ConstraintType.GEQ_INEQUALITY);

      RhoBoundCommand rhoMaxBoundsSegment1 = new RhoBoundCommand();
      rhoMaxBoundsSegment1.setOmega(omega);
      rhoMaxBoundsSegment1.setSegmentDuration(firstSegmentDuration);
      rhoMaxBoundsSegment1.setSegmentNumber(0);
      rhoMaxBoundsSegment1.addContactPlane(contactPlaneHelper1, maxRho);
      rhoMaxBoundsSegment1.setConstraintType(ConstraintType.LEQ_INEQUALITY);

      RhoBoundCommand rhoMaxBoundsSegment3 = new RhoBoundCommand();
      rhoMaxBoundsSegment3.setOmega(omega);
      rhoMaxBoundsSegment3.setSegmentDuration(thirdSegmentDuration);
      rhoMaxBoundsSegment3.setSegmentNumber(2);
      rhoMaxBoundsSegment3.addContactPlane(contactPlaneHelper3, maxRho);
      rhoMaxBoundsSegment3.setConstraintType(ConstraintType.LEQ_INEQUALITY);




      CoMPositionContinuityCommand positionContinuityCommand1 = new CoMPositionContinuityCommand();
      positionContinuityCommand1.setOmega(omega);
      positionContinuityCommand1.setConstraintType(ConstraintType.EQUALITY);
      positionContinuityCommand1.setFirstSegmentNumber(0);
      positionContinuityCommand1.setFirstSegmentDuration(firstSegmentDuration);
      positionContinuityCommand1.addFirstSegmentContactPlaneHelper(contactPlaneHelper1);
//      positionContinuityCommand.addSecondSegmentContactPlaneHelper(contactPlaneHelper3);

      CoMVelocityContinuityCommand velocityContinuityCommand1 = new CoMVelocityContinuityCommand();
      velocityContinuityCommand1.setOmega(omega);
      velocityContinuityCommand1.setConstraintType(ConstraintType.EQUALITY);
      velocityContinuityCommand1.setFirstSegmentNumber(0);
      velocityContinuityCommand1.setFirstSegmentDuration(firstSegmentDuration);
      velocityContinuityCommand1.addFirstSegmentContactPlaneHelper(contactPlaneHelper1);
//      velocityContinuityCommand.addSecondSegmentContactPlaneHelper(contactPlaneHelper3);

      CoMPositionContinuityCommand positionContinuityCommand2 = new CoMPositionContinuityCommand();
      positionContinuityCommand2.setOmega(omega);
      positionContinuityCommand2.setConstraintType(ConstraintType.EQUALITY);
      positionContinuityCommand2.setFirstSegmentNumber(1);
      positionContinuityCommand2.setFirstSegmentDuration(secondSegmentDuration);
      positionContinuityCommand2.addSecondSegmentContactPlaneHelper(contactPlaneHelper3);

      CoMVelocityContinuityCommand velocityContinuityCommand2 = new CoMVelocityContinuityCommand();
      velocityContinuityCommand2.setOmega(omega);
      velocityContinuityCommand2.setConstraintType(ConstraintType.EQUALITY);
      velocityContinuityCommand2.setFirstSegmentNumber(1);
      velocityContinuityCommand2.setFirstSegmentDuration(secondSegmentDuration);
      velocityContinuityCommand2.addSecondSegmentContactPlaneHelper(contactPlaneHelper3);

      CoMPositionCommand startComPositionCommand = new CoMPositionCommand();
      startComPositionCommand.setSegmentNumber(0);
      startComPositionCommand.setOmega(omega);
      startComPositionCommand.setTimeOfObjective(0.0);
      startComPositionCommand.setConstraintType(ConstraintType.EQUALITY);
      startComPositionCommand.setObjective(startPosition);
      startComPositionCommand.addContactPlaneHelper(contactPlaneHelper1);

      CoMPositionCommand endComPositionCommand = new CoMPositionCommand();
      endComPositionCommand.setSegmentNumber(2);
      endComPositionCommand.setOmega(omega);
      endComPositionCommand.setTimeOfObjective(thirdSegmentDuration);
      endComPositionCommand.setConstraintType(ConstraintType.EQUALITY);
      endComPositionCommand.setObjective(endPosition);
      endComPositionCommand.addContactPlaneHelper(contactPlaneHelper3);

      CoMVelocityCommand startComVelocityCommand = new CoMVelocityCommand();
      startComVelocityCommand.setSegmentNumber(0);
      startComVelocityCommand.setOmega(omega);
      startComVelocityCommand.setTimeOfObjective(0.0);
      startComVelocityCommand.setConstraintType(ConstraintType.EQUALITY);
      startComVelocityCommand.setObjective(zeroVelocity);
      startComVelocityCommand.addContactPlaneHelper(contactPlaneHelper1);

      CoMVelocityCommand endComVelocityCommand = new CoMVelocityCommand();
      endComVelocityCommand.setSegmentNumber(2);
      endComVelocityCommand.setOmega(omega);
      endComVelocityCommand.setTimeOfObjective(thirdSegmentDuration);
      endComVelocityCommand.setConstraintType(ConstraintType.EQUALITY);
      endComVelocityCommand.setObjective(zeroVelocity);
      endComVelocityCommand.addContactPlaneHelper(contactPlaneHelper3);

      VRPTrackingCommand startVRPCommand = new VRPTrackingCommand();
      startVRPCommand.setSegmentNumber(0);
      startVRPCommand.setOmega(omega);
      startVRPCommand.setSegmentDuration(firstSegmentDuration);
      startVRPCommand.setWeight(EuclideanModelPredictiveController.defaultVrpTrackingWeight);
      startVRPCommand.setStartVRP(startPosition);
      startVRPCommand.setEndVRP(startPosition);
      startVRPCommand.addContactPlaneHelper(contactPlaneHelper1);

      VRPTrackingCommand endVRPCommand = new VRPTrackingCommand();
      endVRPCommand.setSegmentNumber(2);
      endVRPCommand.setOmega(omega);
      endVRPCommand.setSegmentDuration(thirdSegmentDuration);
      endVRPCommand.setWeight(EuclideanModelPredictiveController.defaultVrpTrackingWeight);
      endVRPCommand.setStartVRP(endPosition);
      endVRPCommand.setEndVRP(endPosition);
      endVRPCommand.addContactPlaneHelper(contactPlaneHelper3);

      VRPPositionCommand endVRPPositionCommand = new VRPPositionCommand();
      endVRPPositionCommand.setSegmentNumber(2);
      endVRPPositionCommand.setOmega(omega);
      endVRPPositionCommand.setTimeOfObjective(thirdSegmentDuration);
      endVRPPositionCommand.setObjective(endPosition);
      endVRPPositionCommand.setConstraintType(ConstraintType.OBJECTIVE);
      endComPositionCommand.addContactPlaneHelper(contactPlaneHelper3);

      double regularization = 1e-5;
      solver.setMaxNumberOfIterations(1000);
      solver.initialize();

      solver.submitMPCCommand(rhoMinBoundsSegment1);
      solver.submitMPCCommand(rhoMinBoundsSegment3);
//      solver.submitMPCCommand(rhoMaxBoundsSegment1);
//      solver.submitMPCCommand(rhoMaxBoundsSegment3);

      solver.submitMPCCommand(startComPositionCommand);
      solver.submitMPCCommand(startComVelocityCommand);
      solver.submitMPCCommand(endComPositionCommand);
      solver.submitMPCCommand(endComVelocityCommand);

      solver.submitMPCCommand(startVRPCommand);
      solver.submitMPCCommand(endVRPCommand);
      solver.submitMPCCommand(endVRPPositionCommand);

      solver.submitMPCCommand(positionContinuityCommand1);
      solver.submitMPCCommand(velocityContinuityCommand1);
      solver.submitMPCCommand(positionContinuityCommand2);
      solver.submitMPCCommand(velocityContinuityCommand2);

      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      assertTrue(solver.solve());

      contactPlaneHelper1.computeContactForceCoefficientMatrix(solver.getSolution(), indexHandler.getRhoCoefficientStartIndex(0));
      contactPlaneHelper3.computeContactForceCoefficientMatrix(solver.getSolution(), indexHandler.getRhoCoefficientStartIndex(2));

      double epsilon = 1e-2;

      double timeStep = 0.05;
      for (double time = 0.0; time <= (firstSegmentDuration + timeStep / 10.0); time += timeStep)
      {
         contactPlaneHelper1.computeContactForce(omega, time);
         for (int contactPointIdx = 0; contactPointIdx < contactPlaneHelper1.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlaneHelper1.getContactPointHelper(contactPointIdx);
            for (int rhoIdx = 0; rhoIdx < contactPoint.getRhoSize(); rhoIdx++)
            {
               FrameVector3DReadOnly basis = contactPoint.getBasisMagnitude(rhoIdx);
               double force = basis.length();
               String minMessage = "Force 1 is " + force + " at time " + time + ", and is expected to be above " + minRho;
               String maxMessage = "Force 1 is " + force + " at time " + time + ", and is expected to be below " + maxRho;
               boolean minGood = force>= minRho - epsilon;
               boolean maxGood = force <= maxRho + epsilon;
               if (time == 0.0 || MathTools.epsilonEquals(time, firstSegmentDuration, timeStep / 10.0))
               {
                  assertTrue(minMessage, minGood);
//                  assertTrue(maxMessage, maxGood);
               }
               else
               {
                  if (!minGood)
                     LogTools.info(minMessage);
//                  if (!maxGood)
//                     LogTools.info(maxMessage);
               }
            }
         }
      }

      for (double time = 0.0; time <= (thirdSegmentDuration + timeStep / 10.0); time += timeStep)
      {
         contactPlaneHelper3.computeContactForce(omega, time);
         for (int contactPointIdx = 0; contactPointIdx < contactPlaneHelper3.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlaneHelper3.getContactPointHelper(contactPointIdx);
            for (int rhoIdx = 0; rhoIdx < contactPoint.getRhoSize(); rhoIdx++)
            {
               FrameVector3DReadOnly basis = contactPoint.getBasisMagnitude(rhoIdx);
               double force = basis.length();
               String minMessage = "Force 3 is " + force + " at time " + time + ", and is expected to be above " + minRho;
               String maxMessage = "Force 3 is " + force + " at time " + time + ", and is expected to be below " + maxRho;
               boolean minGood = force>= minRho - epsilon;
               boolean maxGood = force <= maxRho + epsilon;
               if (time == 0.0 || MathTools.epsilonEquals(time, thirdSegmentDuration, timeStep / 10.0))
               {
                  assertTrue(minMessage, minGood);
                  assertTrue(maxMessage, maxGood);
               }
               else
               {
                  if (!minGood)
                     LogTools.info(minMessage);
                  if (!maxGood)
                     LogTools.info(maxMessage);
               }
            }
         }
      }
   }

   @Test
   public void testJumpWithLowerAndUpperBoundsWithTwoContacts()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      int bases = 3;
      MPCContactPlane leftContactPlaneHelper1 = new MPCContactPlane(4, bases, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlaneHelper1 = new MPCContactPlane(4, bases, new ZeroConeRotationCalculator());
      MPCContactPlane leftContactPlaneHelper3 = new MPCContactPlane(4, bases, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlaneHelper3 = new MPCContactPlane(4, bases, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(bases);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();

      //      ConvexPolygon2D contactPolygon = new ConvexPolygon2D();
      //      contactPolygon.addVertex(0.0, 0.0);
      //      contactPolygon.update();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      leftContactPlaneHelper1.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlaneHelper1.computeBasisVectors(contactPolygon, rightContactPose, mu);
      leftContactPlaneHelper3.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlaneHelper3.computeBasisVectors(contactPolygon, rightContactPose, mu);

      FramePoint3D startPosition = new FramePoint3D();
      startPosition.interpolate(leftContactPose.getPosition(), rightContactPose.getPosition(), 0.5);
      startPosition.setZ(0.75);

      FramePoint3D endPosition = new FramePoint3D(leftContactPose.getPosition());
      endPosition.interpolate(leftContactPose.getPosition(), rightContactPose.getPosition(), 0.5);
      endPosition.setZ(1.0);

      FrameVector3D zeroVelocity = new FrameVector3D();


      indexHandler.initialize(
            i ->
            {
               if (i == 1)
                  return 0;
               else
                  return 2 * contactPolygon.getNumberOfVertices();
            }, 3);

      double firstSegmentDuration = 0.7;
      double secondSegmentDuration = 0.5;
      double thirdSegmentDuration = 0.9;
      double minRho = 0.001;
      double maxGs = 2.5;
      double maxForce = maxGs * Math.abs(gravityZ);
      double maxRho = maxForce / (contactPolygon.getNumberOfVertices() * bases) / mu;

      RhoBoundCommand rhoMinBoundsSegment1 = new RhoBoundCommand();
      rhoMinBoundsSegment1.setOmega(omega);
      rhoMinBoundsSegment1.setSegmentDuration(firstSegmentDuration);
      rhoMinBoundsSegment1.setSegmentNumber(0);
      rhoMinBoundsSegment1.addContactPlane(leftContactPlaneHelper1, minRho);
      rhoMinBoundsSegment1.addContactPlane(rightContactPlaneHelper1, minRho);
      rhoMinBoundsSegment1.setConstraintType(ConstraintType.GEQ_INEQUALITY);


      RhoBoundCommand rhoMinBoundsSegment3 = new RhoBoundCommand();
      rhoMinBoundsSegment3.setOmega(omega);
      rhoMinBoundsSegment3.setSegmentDuration(thirdSegmentDuration);
      rhoMinBoundsSegment3.setSegmentNumber(2);
      rhoMinBoundsSegment3.addContactPlane(leftContactPlaneHelper3, minRho);
      rhoMinBoundsSegment3.addContactPlane(rightContactPlaneHelper3, minRho);
      rhoMinBoundsSegment3.setConstraintType(ConstraintType.GEQ_INEQUALITY);

      RhoBoundCommand rhoMaxBoundsSegment1 = new RhoBoundCommand();
      rhoMaxBoundsSegment1.setOmega(omega);
      rhoMaxBoundsSegment1.setSegmentDuration(firstSegmentDuration);
      rhoMaxBoundsSegment1.setSegmentNumber(0);
      rhoMaxBoundsSegment1.addContactPlane(leftContactPlaneHelper1, maxRho);
      rhoMaxBoundsSegment1.addContactPlane(rightContactPlaneHelper1, maxRho);
      rhoMaxBoundsSegment1.setConstraintType(ConstraintType.LEQ_INEQUALITY);

      RhoBoundCommand rhoMaxBoundsSegment3 = new RhoBoundCommand();
      rhoMaxBoundsSegment3.setOmega(omega);
      rhoMaxBoundsSegment3.setSegmentDuration(thirdSegmentDuration);
      rhoMaxBoundsSegment3.setSegmentNumber(2);
      rhoMaxBoundsSegment3.addContactPlane(leftContactPlaneHelper3, maxRho);
      rhoMaxBoundsSegment3.addContactPlane(rightContactPlaneHelper3, maxRho);
      rhoMaxBoundsSegment3.setConstraintType(ConstraintType.LEQ_INEQUALITY);




      CoMPositionContinuityCommand positionContinuityCommand1 = new CoMPositionContinuityCommand();
      positionContinuityCommand1.setOmega(omega);
      positionContinuityCommand1.setConstraintType(ConstraintType.EQUALITY);
      positionContinuityCommand1.setFirstSegmentNumber(0);
      positionContinuityCommand1.setFirstSegmentDuration(firstSegmentDuration);
      positionContinuityCommand1.addFirstSegmentContactPlaneHelper(leftContactPlaneHelper1);
      positionContinuityCommand1.addFirstSegmentContactPlaneHelper(rightContactPlaneHelper1);
      //      positionContinuityCommand.addSecondSegmentContactPlaneHelper(contactPlaneHelper3);

      CoMVelocityContinuityCommand velocityContinuityCommand1 = new CoMVelocityContinuityCommand();
      velocityContinuityCommand1.setOmega(omega);
      velocityContinuityCommand1.setConstraintType(ConstraintType.EQUALITY);
      velocityContinuityCommand1.setFirstSegmentNumber(0);
      velocityContinuityCommand1.setFirstSegmentDuration(firstSegmentDuration);
      velocityContinuityCommand1.addFirstSegmentContactPlaneHelper(leftContactPlaneHelper1);
      velocityContinuityCommand1.addFirstSegmentContactPlaneHelper(rightContactPlaneHelper1);
      //      velocityContinuityCommand.addSecondSegmentContactPlaneHelper(contactPlaneHelper3);

      CoMPositionContinuityCommand positionContinuityCommand2 = new CoMPositionContinuityCommand();
      positionContinuityCommand2.setOmega(omega);
      positionContinuityCommand2.setConstraintType(ConstraintType.EQUALITY);
      positionContinuityCommand2.setFirstSegmentNumber(1);
      positionContinuityCommand2.setFirstSegmentDuration(secondSegmentDuration);
      positionContinuityCommand2.addSecondSegmentContactPlaneHelper(leftContactPlaneHelper3);
      positionContinuityCommand2.addSecondSegmentContactPlaneHelper(rightContactPlaneHelper3);

      CoMVelocityContinuityCommand velocityContinuityCommand2 = new CoMVelocityContinuityCommand();
      velocityContinuityCommand2.setOmega(omega);
      velocityContinuityCommand2.setConstraintType(ConstraintType.EQUALITY);
      velocityContinuityCommand2.setFirstSegmentNumber(1);
      velocityContinuityCommand2.setFirstSegmentDuration(secondSegmentDuration);
      velocityContinuityCommand2.addSecondSegmentContactPlaneHelper(leftContactPlaneHelper3);
      velocityContinuityCommand2.addSecondSegmentContactPlaneHelper(rightContactPlaneHelper3);

      CoMPositionCommand startComPositionCommand = new CoMPositionCommand();
      startComPositionCommand.setSegmentNumber(0);
      startComPositionCommand.setOmega(omega);
      startComPositionCommand.setTimeOfObjective(0.0);
      startComPositionCommand.setConstraintType(ConstraintType.EQUALITY);
      startComPositionCommand.setObjective(startPosition);
      startComPositionCommand.addContactPlaneHelper(leftContactPlaneHelper1);
      startComPositionCommand.addContactPlaneHelper(rightContactPlaneHelper1);

      CoMPositionCommand endComPositionCommand = new CoMPositionCommand();
      endComPositionCommand.setSegmentNumber(2);
      endComPositionCommand.setOmega(omega);
      endComPositionCommand.setTimeOfObjective(thirdSegmentDuration);
      endComPositionCommand.setConstraintType(ConstraintType.EQUALITY);
      endComPositionCommand.setObjective(endPosition);
      endComPositionCommand.addContactPlaneHelper(leftContactPlaneHelper3);
      endComPositionCommand.addContactPlaneHelper(rightContactPlaneHelper3);

      CoMVelocityCommand startComVelocityCommand = new CoMVelocityCommand();
      startComVelocityCommand.setSegmentNumber(0);
      startComVelocityCommand.setOmega(omega);
      startComVelocityCommand.setTimeOfObjective(0.0);
      startComVelocityCommand.setConstraintType(ConstraintType.EQUALITY);
      startComVelocityCommand.setObjective(zeroVelocity);
      startComVelocityCommand.addContactPlaneHelper(leftContactPlaneHelper1);
      startComVelocityCommand.addContactPlaneHelper(rightContactPlaneHelper1);

      CoMVelocityCommand endComVelocityCommand = new CoMVelocityCommand();
      endComVelocityCommand.setSegmentNumber(2);
      endComVelocityCommand.setOmega(omega);
      endComVelocityCommand.setTimeOfObjective(thirdSegmentDuration);
      endComVelocityCommand.setConstraintType(ConstraintType.EQUALITY);
      endComVelocityCommand.setObjective(zeroVelocity);
      endComVelocityCommand.addContactPlaneHelper(leftContactPlaneHelper3);
      endComVelocityCommand.addContactPlaneHelper(rightContactPlaneHelper3);

      VRPTrackingCommand startVRPCommand = new VRPTrackingCommand();
      startVRPCommand.setSegmentNumber(0);
      startVRPCommand.setOmega(omega);
      startVRPCommand.setSegmentDuration(firstSegmentDuration);
      startVRPCommand.setWeight(EuclideanModelPredictiveController.defaultVrpTrackingWeight);
      startVRPCommand.setStartVRP(startPosition);
      startVRPCommand.setEndVRP(startPosition);
      startVRPCommand.addContactPlaneHelper(leftContactPlaneHelper1);
      startVRPCommand.addContactPlaneHelper(rightContactPlaneHelper1);

      VRPTrackingCommand endVRPCommand = new VRPTrackingCommand();
      endVRPCommand.setSegmentNumber(2);
      endVRPCommand.setOmega(omega);
      endVRPCommand.setSegmentDuration(thirdSegmentDuration);
      endVRPCommand.setWeight(EuclideanModelPredictiveController.defaultVrpTrackingWeight);
      endVRPCommand.setStartVRP(endPosition);
      endVRPCommand.setEndVRP(endPosition);
      endVRPCommand.addContactPlaneHelper(leftContactPlaneHelper3);
      endVRPCommand.addContactPlaneHelper(rightContactPlaneHelper3);

      VRPPositionCommand endVRPPositionCommand = new VRPPositionCommand();
      endVRPPositionCommand.setSegmentNumber(2);
      endVRPPositionCommand.setOmega(omega);
      endVRPPositionCommand.setTimeOfObjective(thirdSegmentDuration);
      endVRPPositionCommand.setObjective(endPosition);
      endVRPPositionCommand.setConstraintType(ConstraintType.OBJECTIVE);
      endComPositionCommand.addContactPlaneHelper(leftContactPlaneHelper3);
      endComPositionCommand.addContactPlaneHelper(rightContactPlaneHelper3);

      double regularization = 1e-5;
      solver.setMaxNumberOfIterations(1000);
      solver.initialize();

      solver.submitMPCCommand(rhoMinBoundsSegment1);
      solver.submitMPCCommand(rhoMinBoundsSegment3);
      //      solver.submitMPCCommand(rhoMaxBoundsSegment1);
      //      solver.submitMPCCommand(rhoMaxBoundsSegment3);

      solver.submitMPCCommand(startComPositionCommand);
      solver.submitMPCCommand(startComVelocityCommand);
      solver.submitMPCCommand(endComPositionCommand);
      solver.submitMPCCommand(endComVelocityCommand);

      solver.submitMPCCommand(startVRPCommand);
      solver.submitMPCCommand(endVRPCommand);
      solver.submitMPCCommand(endVRPPositionCommand);

      solver.submitMPCCommand(positionContinuityCommand1);
      solver.submitMPCCommand(velocityContinuityCommand1);
      solver.submitMPCCommand(positionContinuityCommand2);
      solver.submitMPCCommand(velocityContinuityCommand2);

      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      assertTrue(solver.solve());

      leftContactPlaneHelper1.computeContactForceCoefficientMatrix(solver.getSolution(), indexHandler.getRhoCoefficientStartIndex(0));
      leftContactPlaneHelper3.computeContactForceCoefficientMatrix(solver.getSolution(), indexHandler.getRhoCoefficientStartIndex(2));

      double epsilon = 1e-2;

      double timeStep = 0.05;
      for (double time = 0.0; time <= (firstSegmentDuration + timeStep / 10.0); time += timeStep)
      {
         leftContactPlaneHelper1.computeContactForce(omega, time);
         for (int contactPointIdx = 0; contactPointIdx < leftContactPlaneHelper1.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = leftContactPlaneHelper1.getContactPointHelper(contactPointIdx);
            for (int rhoIdx = 0; rhoIdx < contactPoint.getRhoSize(); rhoIdx++)
            {
               FrameVector3DReadOnly basis = contactPoint.getBasisMagnitude(rhoIdx);
               double force = basis.length();
               String minMessage = "Force 1 is " + force + " at time " + time + ", and is expected to be above " + minRho;
               String maxMessage = "Force 1 is " + force + " at time " + time + ", and is expected to be below " + maxRho;
               boolean minGood = force>= minRho - epsilon;
               boolean maxGood = force <= maxRho + epsilon;
               if (time == 0.0 || MathTools.epsilonEquals(time, firstSegmentDuration, timeStep / 10.0))
               {
                  assertTrue(minMessage, minGood);
                  //                  assertTrue(maxMessage, maxGood);
               }
               else
               {
                  if (!minGood)
                     LogTools.info(minMessage);
                  //                  if (!maxGood)
                  //                     LogTools.info(maxMessage);
               }
            }
         }
      }

      for (double time = 0.0; time <= (thirdSegmentDuration + timeStep / 10.0); time += timeStep)
      {
         leftContactPlaneHelper3.computeContactForce(omega, time);
         for (int contactPointIdx = 0; contactPointIdx < leftContactPlaneHelper3.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = leftContactPlaneHelper3.getContactPointHelper(contactPointIdx);
            for (int rhoIdx = 0; rhoIdx < contactPoint.getRhoSize(); rhoIdx++)
            {
               FrameVector3DReadOnly basis = contactPoint.getBasisMagnitude(rhoIdx);
               double force = basis.length();
               String minMessage = "Force 3 is " + force + " at time " + time + ", and is expected to be above " + minRho;
               String maxMessage = "Force 3 is " + force + " at time " + time + ", and is expected to be below " + maxRho;
               boolean minGood = force>= minRho - epsilon;
               boolean maxGood = force <= maxRho + epsilon;
               if (time == 0.0 || MathTools.epsilonEquals(time, thirdSegmentDuration, timeStep / 10.0))
               {
                  assertTrue(minMessage, minGood);
                  assertTrue(maxMessage, maxGood);
               }
               else
               {
                  if (!minGood)
                     LogTools.info(minMessage);
                  if (!maxGood)
                     LogTools.info(maxMessage);
               }
            }
         }
      }
   }

   @Test
   public void testCommandOptimizeBeginningAndEnd2Segments()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper1 = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane contactPlaneHelper2 = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, mu);
      contactPlaneHelper1.computeBasisVectors(contactPolygon, contactPose, mu);
      contactPlaneHelper2.computeBasisVectors(contactPolygon, contactPose, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      double timeOfConstraint = 0.7;
      double minRho = 0.001;

      RhoAccelerationObjectiveCommand rhoCommandStart1 = new RhoAccelerationObjectiveCommand();
      rhoCommandStart1.setOmega(omega);
      rhoCommandStart1.setTimeOfObjective(0.0);
      rhoCommandStart1.setSegmentNumber(0);
      rhoCommandStart1.setConstraintType(ConstraintType.GEQ_INEQUALITY);
      rhoCommandStart1.setScalarObjective(minRho);
      rhoCommandStart1.setUseScalarObjective(true);
      rhoCommandStart1.addContactPlaneHelper(contactPlaneHelper1);

      RhoAccelerationObjectiveCommand rhoCommandEnd1 = new RhoAccelerationObjectiveCommand();
      rhoCommandEnd1.setOmega(omega);
      rhoCommandEnd1.setTimeOfObjective(timeOfConstraint);
      rhoCommandEnd1.setSegmentNumber(0);
      rhoCommandEnd1.setConstraintType(ConstraintType.GEQ_INEQUALITY);
      rhoCommandEnd1.setScalarObjective(minRho);
      rhoCommandEnd1.setUseScalarObjective(true);
      rhoCommandEnd1.addContactPlaneHelper(contactPlaneHelper1);

      RhoAccelerationObjectiveCommand rhoCommandStart2 = new RhoAccelerationObjectiveCommand();
      rhoCommandStart2.setOmega(omega);
      rhoCommandStart2.setTimeOfObjective(0.0);
      rhoCommandStart2.setSegmentNumber(1);
      rhoCommandStart2.setConstraintType(ConstraintType.GEQ_INEQUALITY);
      rhoCommandStart2.setScalarObjective(minRho);
      rhoCommandStart2.setUseScalarObjective(true);
      rhoCommandStart2.addContactPlaneHelper(contactPlaneHelper2);

      RhoAccelerationObjectiveCommand rhoCommandEnd2 = new RhoAccelerationObjectiveCommand();
      rhoCommandEnd2.setOmega(omega);
      rhoCommandEnd2.setTimeOfObjective(timeOfConstraint);
      rhoCommandEnd2.setSegmentNumber(1);
      rhoCommandEnd2.setConstraintType(ConstraintType.GEQ_INEQUALITY);
      rhoCommandEnd2.setScalarObjective(minRho);
      rhoCommandEnd2.setUseScalarObjective(true);
      rhoCommandEnd2.addContactPlaneHelper(contactPlaneHelper2);

      CoMPositionContinuityCommand positionContinuityCommand = new CoMPositionContinuityCommand();
      positionContinuityCommand.setOmega(omega);
      positionContinuityCommand.setConstraintType(ConstraintType.EQUALITY);
      positionContinuityCommand.setFirstSegmentNumber(0);
      positionContinuityCommand.setFirstSegmentDuration(timeOfConstraint);
      positionContinuityCommand.addFirstSegmentContactPlaneHelper(contactPlaneHelper1);
      positionContinuityCommand.addSecondSegmentContactPlaneHelper(contactPlaneHelper2);

      CoMVelocityContinuityCommand velocityContinuityCommand = new CoMVelocityContinuityCommand();
      velocityContinuityCommand.setOmega(omega);
      velocityContinuityCommand.setConstraintType(ConstraintType.EQUALITY);
      velocityContinuityCommand.setFirstSegmentNumber(0);
      velocityContinuityCommand.setFirstSegmentDuration(timeOfConstraint);
      velocityContinuityCommand.addFirstSegmentContactPlaneHelper(contactPlaneHelper1);
      velocityContinuityCommand.addSecondSegmentContactPlaneHelper(contactPlaneHelper2);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitRhoValueCommand(rhoCommandStart1);
      solver.submitRhoValueCommand(rhoCommandEnd1);
      solver.submitRhoValueCommand(rhoCommandStart2);
      solver.submitRhoValueCommand(rhoCommandEnd2);
      solver.submitContinuityObjective(positionContinuityCommand);
      solver.submitContinuityObjective(velocityContinuityCommand);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      DMatrixRMaj rhoValueVectorStart1 = new DMatrixRMaj(rhoHelper.getRhoSize(), 1);
      DMatrixRMaj rhoValueVectorStart2 = new DMatrixRMaj(rhoHelper.getRhoSize(), 1);
      DMatrixRMaj rhoValueVectorEnd1 = new DMatrixRMaj(rhoHelper.getRhoSize(), 1);
      DMatrixRMaj rhoValueVectorEnd2 = new DMatrixRMaj(rhoHelper.getRhoSize(), 1);

      DMatrixRMaj solution = solver.getSolution();
      DMatrixRMaj rhoSolution1 = new DMatrixRMaj(rhoHelper.getRhoSize() * 4, 1);
      DMatrixRMaj rhoSolution2 = new DMatrixRMaj(rhoHelper.getRhoSize() * 4, 1);

      MatrixTools.setMatrixBlock(rhoSolution1, 0, 0, solution, 12, 0, rhoHelper.getRhoSize() * 4, 1, 1.0);
      MatrixTools.setMatrixBlock(rhoSolution2, 0, 0, solution, 12 + 4 * rhoHelper.getRhoSize(), 0, rhoHelper.getRhoSize() * 4, 1, 1.0);

      helper.computeMatrices(0.0, omega);
      CommonOps_DDRM.mult(helper.getPositionJacobianMatrix(), rhoSolution1, rhoValueVectorStart1);
      CommonOps_DDRM.mult(helper.getPositionJacobianMatrix(), rhoSolution2, rhoValueVectorStart2);
      helper.computeMatrices(timeOfConstraint, omega);
      CommonOps_DDRM.mult(helper.getPositionJacobianMatrix(), rhoSolution1, rhoValueVectorEnd1);
      CommonOps_DDRM.mult(helper.getPositionJacobianMatrix(), rhoSolution2, rhoValueVectorEnd2);

      DMatrixRMaj inequalityObjectiveExpected = new DMatrixRMaj(4 * rhoHelper.getRhoSize(), 1);
      CommonOps_DDRM.fill(inequalityObjectiveExpected, minRho);

      DMatrixRMaj inequalityJacobianExpected = new DMatrixRMaj(4 * rhoHelper.getRhoSize(), indexHandler.getTotalProblemSize());
      DMatrixRMaj equalityJacobianExpected = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());

      double omega2 = omega * omega;

      double c0Start = 1.0;
      double c1Start = 1.0;
      double c2Start = 0.0;
      double c3Start = 0.0;
      double c4Start = 0.0;
      double c5Start = 1.0;
      double c0End = Math.exp(omega * timeOfConstraint);
      double c1End = Math.exp(-omega * timeOfConstraint);
      double c2End = timeOfConstraint * timeOfConstraint * timeOfConstraint;
      double c3End = timeOfConstraint * timeOfConstraint;
      double c4End = timeOfConstraint;
      double c5End = 1.0;

      double c0DotStart = omega;
      double c1DotStart = -omega;
      double c2DotStart = 0.0;
      double c3DotStart = 0.0;
      double c4DotStart = 1.0;
      double c5DotStart = 0.0;
      double c0DotEnd = omega * Math.exp(omega * timeOfConstraint);
      double c1DotEnd = -omega * Math.exp(-omega * timeOfConstraint);
      double c2DotEnd = 3.0 * timeOfConstraint * timeOfConstraint;
      double c3DotEnd = 2.0 * timeOfConstraint;
      double c4DotEnd = 1.0;
      double c5DotEnd = 0.0;

      double a0Start = omega2;
      double a0End = omega2 * Math.exp(omega * timeOfConstraint);
      double a1Start = omega2;
      double a1End = omega2 * Math.exp(-omega * timeOfConstraint);
      double a2Start = 0.0;
      double a2End = 6.0 * timeOfConstraint;
      double a3Start = 2.0;
      double a3End = 2.0;

      int startOf2 = indexHandler.getComCoefficientStartIndex(1);
      equalityJacobianExpected.set(0, 0, c4End);
      equalityJacobianExpected.set(0, 1, c5End);
      equalityJacobianExpected.set(1, 2, c4End);
      equalityJacobianExpected.set(1, 3, c5End);
      equalityJacobianExpected.set(2, 4, c4End);
      equalityJacobianExpected.set(2, 5, c5End);

      equalityJacobianExpected.set(0, startOf2 + 0, -c4Start);
      equalityJacobianExpected.set(0, startOf2 + 1, -c5Start);
      equalityJacobianExpected.set(1, startOf2 + 2, -c4Start);
      equalityJacobianExpected.set(1, startOf2 + 3, -c5Start);
      equalityJacobianExpected.set(2, startOf2 + 4, -c4Start);
      equalityJacobianExpected.set(2, startOf2 + 5, -c5Start);

      equalityJacobianExpected.set(3, 0, c4DotEnd);
      equalityJacobianExpected.set(3, 1, c5DotEnd);
      equalityJacobianExpected.set(4, 2, c4DotEnd);
      equalityJacobianExpected.set(4, 3, c5DotEnd);
      equalityJacobianExpected.set(5, 4, c4DotEnd);
      equalityJacobianExpected.set(5, 5, c5DotEnd);

      equalityJacobianExpected.set(3, startOf2 + 0, -c4DotStart);
      equalityJacobianExpected.set(3, startOf2 + 1, -c5DotStart);
      equalityJacobianExpected.set(4, startOf2 + 2, -c4DotStart);
      equalityJacobianExpected.set(4, startOf2 + 3, -c5DotStart);
      equalityJacobianExpected.set(5, startOf2 + 4, -c4DotStart);
      equalityJacobianExpected.set(5, startOf2 + 5, -c5DotStart);

      for (int rhoIdxStart1  = 0; rhoIdxStart1 < rhoHelper.getRhoSize(); rhoIdxStart1++)
      {
         int startColIdx1 = indexHandler.getRhoCoefficientStartIndex(0) + 4 * rhoIdxStart1;
         int startColIdx2 = indexHandler.getRhoCoefficientStartIndex(1) + 4 * rhoIdxStart1;

         equalityJacobianExpected.set(0, startColIdx1, rhoHelper.getBasisVector(rhoIdxStart1).getX() * c0End);
         equalityJacobianExpected.set(1, startColIdx1, rhoHelper.getBasisVector(rhoIdxStart1).getY() * c0End);
         equalityJacobianExpected.set(2, startColIdx1, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * c0End);

         equalityJacobianExpected.set(0, startColIdx1 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getX() * c1End);
         equalityJacobianExpected.set(1, startColIdx1 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getY() * c1End);
         equalityJacobianExpected.set(2, startColIdx1 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * c1End);

         equalityJacobianExpected.set(0, startColIdx1 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getX() * c2End);
         equalityJacobianExpected.set(1, startColIdx1 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getY() * c2End);
         equalityJacobianExpected.set(2, startColIdx1 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * c2End);

         equalityJacobianExpected.set(0, startColIdx1 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getX() * c3End);
         equalityJacobianExpected.set(1, startColIdx1 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getY() * c3End);
         equalityJacobianExpected.set(2, startColIdx1 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * c3End);

         equalityJacobianExpected.set(0, startColIdx2, rhoHelper.getBasisVector(rhoIdxStart1).getX() * -c0Start);
         equalityJacobianExpected.set(1, startColIdx2, rhoHelper.getBasisVector(rhoIdxStart1).getY() * -c0Start);
         equalityJacobianExpected.set(2, startColIdx2, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * -c0Start);

         equalityJacobianExpected.set(0, startColIdx2 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getX() * -c1Start);
         equalityJacobianExpected.set(1, startColIdx2 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getY() * -c1Start);
         equalityJacobianExpected.set(2, startColIdx2 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * -c1Start);

         equalityJacobianExpected.set(0, startColIdx2 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getX() * -c2Start);
         equalityJacobianExpected.set(1, startColIdx2 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getY() * -c2Start);
         equalityJacobianExpected.set(2, startColIdx2 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * -c2Start);

         equalityJacobianExpected.set(0, startColIdx2 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getX() * -c3Start);
         equalityJacobianExpected.set(1, startColIdx2 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getY() * -c3Start);
         equalityJacobianExpected.set(2, startColIdx2 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * -c3Start);

         equalityJacobianExpected.set(3, startColIdx1, rhoHelper.getBasisVector(rhoIdxStart1).getX() * c0DotEnd);
         equalityJacobianExpected.set(4, startColIdx1, rhoHelper.getBasisVector(rhoIdxStart1).getY() * c0DotEnd);
         equalityJacobianExpected.set(5, startColIdx1, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * c0DotEnd);

         equalityJacobianExpected.set(3, startColIdx1 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getX() * c1DotEnd);
         equalityJacobianExpected.set(4, startColIdx1 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getY() * c1DotEnd);
         equalityJacobianExpected.set(5, startColIdx1 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * c1DotEnd);

         equalityJacobianExpected.set(3, startColIdx1 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getX() * c2DotEnd);
         equalityJacobianExpected.set(4, startColIdx1 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getY() * c2DotEnd);
         equalityJacobianExpected.set(5, startColIdx1 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * c2DotEnd);

         equalityJacobianExpected.set(3, startColIdx1 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getX() * c3DotEnd);
         equalityJacobianExpected.set(4, startColIdx1 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getY() * c3DotEnd);
         equalityJacobianExpected.set(5, startColIdx1 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * c3DotEnd);

         equalityJacobianExpected.set(3, startColIdx2, rhoHelper.getBasisVector(rhoIdxStart1).getX() * -c0DotStart);
         equalityJacobianExpected.set(4, startColIdx2, rhoHelper.getBasisVector(rhoIdxStart1).getY() * -c0DotStart);
         equalityJacobianExpected.set(5, startColIdx2, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * -c0DotStart);

         equalityJacobianExpected.set(3, startColIdx2 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getX() * -c1DotStart);
         equalityJacobianExpected.set(4, startColIdx2 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getY() * -c1DotStart);
         equalityJacobianExpected.set(5, startColIdx2 + 1, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * -c1DotStart);

         equalityJacobianExpected.set(3, startColIdx2 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getX() * -c2DotStart);
         equalityJacobianExpected.set(4, startColIdx2 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getY() * -c2DotStart);
         equalityJacobianExpected.set(5, startColIdx2 + 2, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * -c2DotStart);

         equalityJacobianExpected.set(3, startColIdx2 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getX() * -c3DotStart);
         equalityJacobianExpected.set(4, startColIdx2 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getY() * -c3DotStart);
         equalityJacobianExpected.set(5, startColIdx2 + 3, rhoHelper.getBasisVector(rhoIdxStart1).getZ() * -c3DotStart);

         double rhoAccelerationValueStart1 = a0Start * solution.get(startColIdx1, 0);
         rhoAccelerationValueStart1 += a1Start * solution.get(startColIdx1 + 1, 0);
         rhoAccelerationValueStart1 += a2Start * solution.get(startColIdx1 + 2, 0);
         rhoAccelerationValueStart1 += a3Start * solution.get(startColIdx1 + 3, 0);
         double rhoAccelerationValueEnd1 = a0End * solution.get(startColIdx1, 0);
         rhoAccelerationValueEnd1 += a1End * solution.get(startColIdx1 + 1, 0);
         rhoAccelerationValueEnd1 += a2End * solution.get(startColIdx1 + 2, 0);
         rhoAccelerationValueEnd1 += a3End * solution.get(startColIdx1 + 3, 0);
         double rhoAccelerationValueStart2 = a0Start * solution.get(startColIdx2, 0);
         rhoAccelerationValueStart2 += a1Start * solution.get(startColIdx2 + 1, 0);
         rhoAccelerationValueStart2 += a2Start * solution.get(startColIdx2 + 2, 0);
         rhoAccelerationValueStart2 += a3Start * solution.get(startColIdx2 + 3, 0);
         double rhoAccelerationValueEnd2 = a0End * solution.get(startColIdx2, 0);
         rhoAccelerationValueEnd2 += a1End * solution.get(startColIdx2 + 1, 0);
         rhoAccelerationValueEnd2 += a2End * solution.get(startColIdx2 + 2, 0);
         rhoAccelerationValueEnd2 += a3End * solution.get(startColIdx2 + 3, 0);

         int rhoIdxEnd1 = rhoIdxStart1 + rhoHelper.getRhoSize();
         int rhoIdxStart2 = rhoIdxEnd1 + rhoHelper.getRhoSize();
         int rhoIdxEnd2 = rhoIdxStart2 + rhoHelper.getRhoSize();

//         assertTrue(rhoAccelerationValueStart1 >= rhoValueVectorStart1.get(rhoIdxStart1));
//         assertTrue(rhoAccelerationValueEnd1 >= rhoValueVectorEnd1.get(rhoIdxStart1));
//         assertTrue(rhoAccelerationValueStart2 >= rhoValueVectorStart2.get(rhoIdxStart1));
//         assertTrue(rhoAccelerationValueEnd2 >= rhoValueVectorEnd2.get(rhoIdxStart1));

         inequalityJacobianExpected.set(rhoIdxStart1, startColIdx1, a0Start);
         inequalityJacobianExpected.set(rhoIdxStart1, startColIdx1 + 1, a1Start);
         inequalityJacobianExpected.set(rhoIdxStart1, startColIdx1 + 2, a2Start);
         inequalityJacobianExpected.set(rhoIdxStart1, startColIdx1 + 3, a3Start);

         inequalityJacobianExpected.set(rhoIdxStart2, startColIdx2, a0Start);
         inequalityJacobianExpected.set(rhoIdxStart2, startColIdx2 + 1, a1Start);
         inequalityJacobianExpected.set(rhoIdxStart2, startColIdx2 + 2, a2Start);
         inequalityJacobianExpected.set(rhoIdxStart2, startColIdx2 + 3, a3Start);

         inequalityJacobianExpected.set(rhoIdxEnd1, startColIdx1, a0End);
         inequalityJacobianExpected.set(rhoIdxEnd1, startColIdx1 + 1, a1End);
         inequalityJacobianExpected.set(rhoIdxEnd1, startColIdx1 + 2, a2End);
         inequalityJacobianExpected.set(rhoIdxEnd1, startColIdx1 + 3, a3End);

         inequalityJacobianExpected.set(rhoIdxEnd2, startColIdx2, a0End);
         inequalityJacobianExpected.set(rhoIdxEnd2, startColIdx2 + 1, a1End);
         inequalityJacobianExpected.set(rhoIdxEnd2, startColIdx2 + 2, a2End);
         inequalityJacobianExpected.set(rhoIdxEnd2, startColIdx2 + 3, a3End);
      }

      EjmlUnitTests.assertEquals(equalityJacobianExpected, solver.solverInput_Aeq, 1e-5);


      CommonOps_DDRM.scale(-1.0, inequalityJacobianExpected);
      CommonOps_DDRM.scale(-1.0, inequalityObjectiveExpected);
      EjmlUnitTests.assertEquals(inequalityJacobianExpected, solver.solverInput_Ain, 1e-5);
      EjmlUnitTests.assertEquals(inequalityObjectiveExpected, solver.solverInput_bin, 1e-5);

      DMatrixRMaj solverInput_H_Expected = new DMatrixRMaj(inequalityJacobianExpected.getNumCols(), inequalityJacobianExpected.getNumCols());
      DMatrixRMaj solverInput_f_Expected = new DMatrixRMaj(inequalityJacobianExpected.getNumCols(), 1);

      MatrixTools.addDiagonal(solverInput_H_Expected, regularization);

      EjmlUnitTests.assertEquals(solverInput_H_Expected, solver.solverInput_H, 1e-10);
      EjmlUnitTests.assertEquals(solverInput_f_Expected, solver.solverInput_f, 1e-10);
   }

   private static void addTask(QPInputTypeA input, DMatrixRMaj hessianToPack, DMatrixRMaj gradientToPack)
   {
      addTaskToHessian(input.taskJacobian, hessianToPack);
      addTaskToGradient(input.taskJacobian, input.taskObjective, gradientToPack);
   }

   private static void addTaskToHessian(DMatrixRMaj jacobian, DMatrixRMaj hessianToPack)
   {
      DMatrixRMaj jTJ = new DMatrixRMaj(hessianToPack);
      CommonOps_DDRM.multInner(jacobian, jTJ);
      CommonOps_DDRM.addEquals(hessianToPack, jTJ);
   }

   private static void addTaskToGradient(DMatrixRMaj jacobian, DMatrixRMaj objective, DMatrixRMaj gradientToPack)
   {
      CommonOps_DDRM.multAddTransA(-1.0, jacobian, objective, gradientToPack);
   }
}
