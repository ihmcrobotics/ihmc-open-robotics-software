package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInput;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoMMPCQPSolverTest
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

      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
      CoMMPCQPSolver solver = new CoMMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

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


      DMatrixRMaj solverH_Expected = new DMatrixRMaj(6 + 4 * rhoHelper.getRhoSize(), 6 + 4 * rhoHelper.getRhoSize());
      DMatrixRMaj solverf_Expected = new DMatrixRMaj(6 + 4 * rhoHelper.getRhoSize(), 1);

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
      EjmlUnitTests.assertEquals(expectedVRPStartPositionObjective, solver.qpInput.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getVRPPositionJacobian(timeAtStart, omega, rhoHelper), solver.qpInput.taskJacobian, 1e-4);
      addTask(solver.qpInput, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(vrpStartVelocityCommand);
      EjmlUnitTests.assertEquals(expectedVRPStartVelocityObjective, solver.qpInput.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getVRPVelocityJacobian(timeAtStart, omega, rhoHelper), solver.qpInput.taskJacobian, 1e-4);
      addTask(solver.qpInput, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(vrpEndPositionCommand);
      EjmlUnitTests.assertEquals(expectedVRPEndPositionObjective, solver.qpInput.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getVRPPositionJacobian(timeAtEnd, omega, rhoHelper), solver.qpInput.taskJacobian, 1e-4);
      addTask(solver.qpInput, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(vrpEndVelocityCommand);
      EjmlUnitTests.assertEquals(expectedVRPEndVelocityObjective, solver.qpInput.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getVRPVelocityJacobian(timeAtEnd, omega, rhoHelper), solver.qpInput.taskJacobian, 1e-4);
      addTask(solver.qpInput, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(comStartPositionCommand);
      EjmlUnitTests.assertEquals(expectedCoMStartPositionObjective, solver.qpInput.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getCoMPositionJacobian(timeAtStart, omega, rhoHelper), solver.qpInput.taskJacobian, 1e-4);
      addTask(solver.qpInput, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(comStartVelocityCommand);
      EjmlUnitTests.assertEquals(expectedCoMStartVelocityObjective, solver.qpInput.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getCoMVelocityJacobian(timeAtStart, omega, rhoHelper), solver.qpInput.taskJacobian, 1e-4);
      addTask(solver.qpInput, solverH_Expected, solverf_Expected);

      solver.submitMPCValueObjective(dcmEndPositionCommand);
      EjmlUnitTests.assertEquals(expectedDCMEndPositionObjective, solver.qpInput.taskObjective, 1e-4);
      EjmlUnitTests.assertEquals(MPCTestHelper.getDCMPositionJacobian(timeAtEnd, omega, rhoHelper), solver.qpInput.taskJacobian, 1e-4);
      addTask(solver.qpInput, solverH_Expected, solverf_Expected);

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
         int startIdx = 6 + 4 * rhoIdx;

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
      EuclidCoreTestTools.assertTuple3DEquals(comStartVelocity, reconstructedCoMVelocityAtStart, 1e-3);
      EuclidCoreTestTools.assertTuple3DEquals(dcmObjective, reconstructedDCMAtStart, 5e-3);
      EuclidCoreTestTools.assertTuple3DEquals(dcmObjective, reconstructedDCMAtEnd, 5e-3);
      EuclidCoreTestTools.assertTuple3DEquals(dcmObjective, reconstructedVRPAtStart, 5e-3);
      EuclidCoreTestTools.assertTuple3DEquals(dcmObjective, reconstructedVRPAtEnd, 6e-3);
   }

   private static void addTask(QPInput input, DMatrixRMaj hessianToPack, DMatrixRMaj gradientToPack)
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
