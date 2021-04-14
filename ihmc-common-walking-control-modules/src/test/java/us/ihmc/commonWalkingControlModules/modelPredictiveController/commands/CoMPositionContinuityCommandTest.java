package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoMPositionContinuityCommandTest
{
   @Test
   public void testCommandOptimizeAsObjective()
   {
      FramePoint3D objectivePosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);

      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper1 = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      ContactStateMagnitudeToForceMatrixHelper rhoHelper2 = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());

      MPCContactPlane contactPlaneHelper1 = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane contactPlaneHelper2 = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, null, new YoRegistry("test"));

      FramePose3D contactPose1 = new FramePose3D();
      FramePose3D contactPose2 = new FramePose3D();
      contactPose1.getPosition().set(0.0, 0.0, 0.0);
      contactPose2.getPosition().set(objectivePosition.getX(), objectivePosition.getY(), 0.0);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      rhoHelper1.computeMatrices(contactPolygon, contactPose1, 1e-8, 1e-10, mu);
      rhoHelper2.computeMatrices(contactPolygon, contactPose2, 1e-8, 1e-10, mu);
      contactPlaneHelper1.computeBasisVectors(contactPolygon, contactPose1, mu);
      contactPlaneHelper2.computeBasisVectors(contactPolygon, contactPose2, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      double duration1 = 0.7;

      CoMPositionContinuityCommand command = new CoMPositionContinuityCommand();
      command.setFirstSegmentDuration(duration1);
      command.setFirstSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addFirstSegmentContactPlaneHelper(contactPlaneHelper1);
      command.addSecondSegmentContactPlaneHelper(contactPlaneHelper2);
      command.setConstraintType(ConstraintType.OBJECTIVE);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitContinuityObjective(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      DMatrixRMaj solvedObjectivePosition = new DMatrixRMaj(3, 1);
      FramePoint3D solvedObjectivePositionTuple = new FramePoint3D();

      FramePoint3D valueEndOf1 = new FramePoint3D();
      FramePoint3D valueStartOf2 = new FramePoint3D();

      DMatrixRMaj solution = solver.getSolution();

      CommonOps_DDRM.mult(solver.qpInputTypeA.taskJacobian, solution, solvedObjectivePosition);
      solvedObjectivePositionTuple.set(solvedObjectivePosition);
      solvedObjectivePositionTuple.scaleAdd(0.5 * duration1 * duration1, gravityVector, solvedObjectivePositionTuple);

      DMatrixRMaj taskObjectiveExpected = new DMatrixRMaj(3, 1);
      DMatrixRMaj achievedObjective = new DMatrixRMaj(3, 1);
      taskObjectiveExpected.add(2, 0, -0.5 * duration1 * duration1 * -Math.abs(gravityZ));

      DMatrixRMaj taskJacobianExpected = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
      MatrixMissingTools.addMatrixBlock(taskJacobianExpected, 0, indexHandler.getComCoefficientStartIndex(0), MPCTestHelper.getCoMPositionJacobian(duration1, omega, contactPlaneHelper1));
      MatrixMissingTools.addMatrixBlock(taskJacobianExpected, 0, indexHandler.getComCoefficientStartIndex(1), MPCTestHelper.getCoMPositionJacobian(0.0, omega, contactPlaneHelper2), -1.0);

      valueEndOf1.setX(duration1 * solution.get(0, 0) + solution.get(1, 0));
      valueEndOf1.setY(duration1 * solution.get(2, 0) + solution.get(3, 0));
      valueEndOf1.setZ(duration1 * solution.get(4, 0) + solution.get(5, 0));

      int startOf2 = indexHandler.getComCoefficientStartIndex(1);
      valueStartOf2.setX(0.0 * solution.get(startOf2, 0) + solution.get(startOf2 + 1, 0));
      valueStartOf2.setY(0.0 * solution.get(startOf2 + 2, 0) + solution.get(startOf2 + 3, 0));
      valueStartOf2.setZ(0.0 * solution.get(startOf2 + 4, 0) + solution.get(startOf2 + 5, 0));

      for (int rhoIdx  = 0; rhoIdx < rhoHelper1.getRhoSize(); rhoIdx++)
      {
         int startIdx = indexHandler.getRhoCoefficientStartIndex(0) + 4 * rhoIdx;
         double rhoValue = Math.exp(omega * duration1) * solution.get(startIdx, 0);
         rhoValue += Math.exp(-omega * duration1) * solution.get(startIdx + 1, 0);
         rhoValue += duration1 * duration1 * duration1 * solution.get(startIdx + 2, 0);
         rhoValue += duration1 * duration1 * solution.get(startIdx + 3, 0);

         valueEndOf1.scaleAdd(rhoValue, rhoHelper1.getBasisVector(rhoIdx), valueEndOf1);
      }
      valueEndOf1.scaleAdd(0.5 * duration1 * duration1, gravityVector, valueEndOf1);


      for (int rhoIdx  = 0; rhoIdx < rhoHelper2.getRhoSize(); rhoIdx++)
      {
         int startIdx = indexHandler.getRhoCoefficientStartIndex(1) + 4 * rhoIdx;
         double rhoValue = Math.exp(omega * 0.0) * solution.get(startIdx, 0);
         rhoValue += Math.exp(-omega * 0.0) * solution.get(startIdx + 1, 0);
         rhoValue += 0.0 * 0.0 * 0.0 * solution.get(startIdx + 2, 0);
         rhoValue += 0.0 * 0.0 * solution.get(startIdx + 3, 0);

         valueStartOf2.scaleAdd(rhoValue, rhoHelper2.getBasisVector(rhoIdx), valueStartOf2);
      }
      valueStartOf2.scaleAdd(0.5 * 0.0 * 0.0, gravityVector, valueStartOf2);

      EjmlUnitTests.assertEquals(taskJacobianExpected, solver.qpInputTypeA.taskJacobian, 1e-5);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.qpInputTypeA.taskObjective, 1e-5);

      CommonOps_DDRM.mult(taskJacobianExpected, solution, achievedObjective);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, achievedObjective, 1e-4);

      DMatrixRMaj solverInput_H_Expected = new DMatrixRMaj(taskJacobianExpected.getNumCols(), taskJacobianExpected.getNumCols());
      DMatrixRMaj solverInput_f_Expected = new DMatrixRMaj(taskJacobianExpected.getNumCols(), 1);

      CommonOps_DDRM.multInner(taskJacobianExpected, solverInput_H_Expected);
      CommonOps_DDRM.multTransA(-1.0, taskJacobianExpected, taskObjectiveExpected, solverInput_f_Expected);

      MatrixTools.addDiagonal(solverInput_H_Expected, regularization);

      EjmlUnitTests.assertEquals(solverInput_H_Expected, solver.solverInput_H, 1e-10);
      EjmlUnitTests.assertEquals(solverInput_f_Expected, solver.solverInput_f, 1e-10);

      FramePoint3D desiredValue = new FramePoint3D();
      EuclidCoreTestTools.assertTuple3DEquals(valueEndOf1, valueStartOf2, 1e-4);
      EuclidCoreTestTools.assertTuple3DEquals(desiredValue, solvedObjectivePositionTuple, 1e-4);
   }

   @Test
   public void testCommandOptimizeAsConstraint()
   {
      FramePoint3D objectivePosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);

      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper1 = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      ContactStateMagnitudeToForceMatrixHelper rhoHelper2 = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);

      MPCContactPlane contactPlaneHelper1 = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane contactPlaneHelper2 = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose1 = new FramePose3D();
      FramePose3D contactPose2 = new FramePose3D();
      contactPose1.getPosition().set(0.0, 0.0, 0.0);
      contactPose2.getPosition().set(objectivePosition.getX(), objectivePosition.getY(), 0.0);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      rhoHelper1.computeMatrices(contactPolygon, contactPose1, 1e-8, 1e-10, mu);
      rhoHelper2.computeMatrices(contactPolygon, contactPose2, 1e-8, 1e-10, mu);
      contactPlaneHelper1.computeBasisVectors(contactPolygon, contactPose1, mu);
      contactPlaneHelper2.computeBasisVectors(contactPolygon, contactPose2, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      double duration1 = 0.7;

      CoMPositionContinuityCommand command = new CoMPositionContinuityCommand();
      command.setFirstSegmentDuration(duration1);
      command.setFirstSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addFirstSegmentContactPlaneHelper(contactPlaneHelper1);
      command.addSecondSegmentContactPlaneHelper(contactPlaneHelper2);
      command.setConstraintType(ConstraintType.EQUALITY);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitContinuityObjective(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      DMatrixRMaj solvedObjectivePosition = new DMatrixRMaj(3, 1);
      FramePoint3D solvedObjectivePositionTuple = new FramePoint3D();

      FramePoint3D valueEndOf1 = new FramePoint3D();
      FramePoint3D valueStartOf2 = new FramePoint3D();

      DMatrixRMaj solution = solver.getSolution();

      CommonOps_DDRM.mult(solver.qpInputTypeA.taskJacobian, solution, solvedObjectivePosition);
      solvedObjectivePositionTuple.set(solvedObjectivePosition);
      solvedObjectivePositionTuple.scaleAdd(0.5 * duration1 * duration1, gravityVector, solvedObjectivePositionTuple);

      DMatrixRMaj taskObjectiveExpected = new DMatrixRMaj(3, 1);
      DMatrixRMaj achievedObjective = new DMatrixRMaj(3, 1);
      taskObjectiveExpected.add(2, 0, -0.5 * duration1 * duration1 * -Math.abs(gravityZ));

      DMatrixRMaj taskJacobianExpected = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(0), duration1, taskJacobianExpected, 0, 1.0);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(1), 0.0, taskJacobianExpected, 0, -1.0);

      helper.computeMatrices(duration1, omega);
      MatrixTools.multAddBlock(rhoHelper1.getLinearJacobianInWorldFrame(), helper.getPositionJacobianMatrix(), taskJacobianExpected, 0, indexHandler.getRhoCoefficientStartIndex(0));
      helper.computeMatrices(0.0, omega);
      MatrixTools.multAddBlock(-1.0, rhoHelper2.getLinearJacobianInWorldFrame(), helper.getPositionJacobianMatrix(), taskJacobianExpected, 0, indexHandler.getRhoCoefficientStartIndex(1));

      valueEndOf1.setX(duration1 * solution.get(0, 0) + solution.get(1, 0));
      valueEndOf1.setY(duration1 * solution.get(2, 0) + solution.get(3, 0));
      valueEndOf1.setZ(duration1 * solution.get(4, 0) + solution.get(5, 0));

      int startOf2 = indexHandler.getComCoefficientStartIndex(1);
      valueStartOf2.setX(0.0 * solution.get(startOf2, 0) + solution.get(startOf2 + 1, 0));
      valueStartOf2.setY(0.0 * solution.get(startOf2 + 2, 0) + solution.get(startOf2 + 3, 0));
      valueStartOf2.setZ(0.0 * solution.get(startOf2 + 4, 0) + solution.get(startOf2 + 5, 0));

      for (int rhoIdx  = 0; rhoIdx < rhoHelper1.getRhoSize(); rhoIdx++)
      {
         int startIdx = indexHandler.getRhoCoefficientStartIndex(0) + 4 * rhoIdx;
         double rhoValue = Math.exp(omega * duration1) * solution.get(startIdx, 0);
         rhoValue += Math.exp(-omega * duration1) * solution.get(startIdx + 1, 0);
         rhoValue += duration1 * duration1 * duration1 * solution.get(startIdx + 2, 0);
         rhoValue += duration1 * duration1 * solution.get(startIdx + 3, 0);

         valueEndOf1.scaleAdd(rhoValue, rhoHelper1.getBasisVector(rhoIdx), valueEndOf1);
      }
      valueEndOf1.scaleAdd(0.5 * duration1 * duration1, gravityVector, valueEndOf1);


      for (int rhoIdx  = 0; rhoIdx < rhoHelper2.getRhoSize(); rhoIdx++)
      {
         int startIdx = indexHandler.getRhoCoefficientStartIndex(1) + 4 * rhoIdx;
         double rhoValue = Math.exp(omega * 0.0) * solution.get(startIdx, 0);
         rhoValue += Math.exp(-omega * 0.0) * solution.get(startIdx + 1, 0);
         rhoValue += 0.0 * 0.0 * 0.0 * solution.get(startIdx + 2, 0);
         rhoValue += 0.0 * 0.0 * solution.get(startIdx + 3, 0);

         valueStartOf2.scaleAdd(rhoValue, rhoHelper2.getBasisVector(rhoIdx), valueStartOf2);
      }
      valueStartOf2.scaleAdd(0.5 * 0.0 * 0.0, gravityVector, valueStartOf2);

      EjmlUnitTests.assertEquals(taskJacobianExpected, solver.qpInputTypeA.taskJacobian, 1e-5);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.qpInputTypeA.taskObjective, 1e-5);

      CommonOps_DDRM.mult(taskJacobianExpected, solution, achievedObjective);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, achievedObjective, 1e-4);

      EjmlUnitTests.assertEquals(taskJacobianExpected, solver.solverInput_Aeq, 1e-10);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.solverOutput_beq, 1e-10);

      FramePoint3D desiredValue = new FramePoint3D();
      EuclidCoreTestTools.assertTuple3DEquals(valueEndOf1, valueStartOf2, 1e-4);
      EuclidCoreTestTools.assertTuple3DEquals(desiredValue, solvedObjectivePositionTuple, 1e-4);
   }

}
