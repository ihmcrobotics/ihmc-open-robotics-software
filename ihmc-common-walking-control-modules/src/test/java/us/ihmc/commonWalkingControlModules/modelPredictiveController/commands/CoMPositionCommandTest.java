package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
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
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.*;

public class CoMPositionCommandTest
{
   @Test
   public void testCommandOptimize()
   {
      FramePoint3D objectivePosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);

      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      int numberOfVectors = 4 * contactPolygon.getNumberOfVertices();
      int rhoCoefficients =  numberOfVectors * LinearMPCIndexHandler.coefficientsPerRho;
      int totalCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + rhoCoefficients;

      double timeOfConstraint = 0.7;

      CoMPositionCommand command = new CoMPositionCommand();
      command.setObjective(objectivePosition);
      command.setTimeOfObjective(timeOfConstraint);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addContactPlaneHelper(contactPlane);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitMPCValueObjective(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      FramePoint3D solvedPositionAtConstraint = new FramePoint3D();
      DMatrixRMaj solvedObjectivePosition = new DMatrixRMaj(3, 1);


      DMatrixRMaj solution = solver.getSolution();
      CommonOps_DDRM.mult(solver.qpInputTypeA.taskJacobian, solution, solvedObjectivePosition);

      DMatrixRMaj taskObjectiveExpected = new DMatrixRMaj(3, 1);
      DMatrixRMaj achievedObjective = new DMatrixRMaj(3, 1);
      objectivePosition.get(taskObjectiveExpected);
      taskObjectiveExpected.add(2, 0, -0.5 * timeOfConstraint * timeOfConstraint * -Math.abs(gravityZ));

      DMatrixRMaj taskJacobianExpected = MPCTestHelper.getCoMPositionJacobian(timeOfConstraint, omega, contactPlane);


      solvedPositionAtConstraint.set(solvedObjectivePosition);
      solvedPositionAtConstraint.scaleAdd(0.5 * timeOfConstraint * timeOfConstraint, gravityVector, solvedPositionAtConstraint);

      EjmlUnitTests.assertEquals(taskJacobianExpected, solver.qpInputTypeA.taskJacobian, 1e-5);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.qpInputTypeA.taskObjective, 1e-5);

      DMatrixRMaj solverInput_H_Expected = new DMatrixRMaj(totalCoefficients, totalCoefficients);
      DMatrixRMaj solverInput_f_Expected = new DMatrixRMaj(totalCoefficients, 1);

      CommonOps_DDRM.multInner(taskJacobianExpected, solverInput_H_Expected);
      CommonOps_DDRM.multTransA(-1.0, taskJacobianExpected, taskObjectiveExpected, solverInput_f_Expected);

      MatrixTools.addDiagonal(solverInput_H_Expected, regularization);

      EjmlUnitTests.assertEquals(solverInput_H_Expected, solver.solverInput_H, 1e-10);
      EjmlUnitTests.assertEquals(solverInput_f_Expected, solver.solverInput_f, 1e-10);

      CommonOps_DDRM.mult(taskJacobianExpected, solution, achievedObjective);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, achievedObjective, 1e-4);

      EuclidCoreTestTools.assertTuple3DEquals(objectivePosition, solvedPositionAtConstraint, 1e-4);
   }

   @Test
   public void testDoubleSubmission()
   {
      FramePoint3D objectivePosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);

      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double timeOfConstraint = 0.7;

      CoMPositionCommand command = new CoMPositionCommand();
      command.setObjective(objectivePosition);
      command.setTimeOfObjective(timeOfConstraint);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addContactPlaneHelper(contactPlaneHelper);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitMPCValueObjective(command);
      solver.submitMPCValueObjective(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      DMatrixRMaj rhoValueVector = new DMatrixRMaj(contactPlaneHelper.getRhoSize(), 1);

      DMatrixRMaj solution = solver.getSolution();

      CommonOps_DDRM.mult(MPCTestHelper.getContactValueJacobian(timeOfConstraint, omega, contactPlaneHelper.getRhoSize()), solution, rhoValueVector);

      DMatrixRMaj taskObjectiveExpected = new DMatrixRMaj(3, 1);
      DMatrixRMaj achievedObjective = new DMatrixRMaj(3, 1);
      objectivePosition.get(taskObjectiveExpected);
      taskObjectiveExpected.add(2, 0, -0.5 * timeOfConstraint * timeOfConstraint * -Math.abs(gravityZ));

      DMatrixRMaj taskJacobianExpected = MPCTestHelper.getCoMPositionJacobian(timeOfConstraint, omega, contactPlaneHelper);

      for (int rhoIdx  = 0; rhoIdx < contactPlaneHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;
         double rhoValue = Math.exp(omega * timeOfConstraint) * solution.get(startIdx, 0);
         rhoValue += Math.exp(-omega * timeOfConstraint) * solution.get(startIdx + 1, 0);
         rhoValue += timeOfConstraint * timeOfConstraint * timeOfConstraint * solution.get(startIdx + 2, 0);
         rhoValue += timeOfConstraint * timeOfConstraint * solution.get(startIdx + 3, 0);

         assertEquals(rhoValue, rhoValueVector.get(rhoIdx), 1e-5);
      }

      EjmlUnitTests.assertEquals(taskJacobianExpected, solver.qpInputTypeA.taskJacobian, 1e-5);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.qpInputTypeA.taskObjective, 1e-5);

      CommonOps_DDRM.mult(taskJacobianExpected, solution, achievedObjective);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, achievedObjective, 1e-4);

      DMatrixRMaj solverInput_H_Expected = new DMatrixRMaj(taskJacobianExpected.getNumCols(), taskJacobianExpected.getNumCols());
      DMatrixRMaj solverInput_f_Expected = new DMatrixRMaj(taskJacobianExpected.getNumCols(), 1);

      CommonOps_DDRM.multInner(taskJacobianExpected, solverInput_H_Expected);
      CommonOps_DDRM.scale(2.0, solverInput_H_Expected);
      CommonOps_DDRM.multTransA(-2.0, taskJacobianExpected, taskObjectiveExpected, solverInput_f_Expected);

      MatrixTools.addDiagonal(solverInput_H_Expected, regularization);

      EjmlUnitTests.assertEquals(solverInput_H_Expected, solver.solverInput_H, 1e-10);
      EjmlUnitTests.assertEquals(solverInput_f_Expected, solver.solverInput_f, 1e-10);


      EuclidCoreTestTools.assertTuple3DEquals(objectivePosition, MPCTestHelper.computeCoMPosition(timeOfConstraint, omega, gravityZ, solution, contactPlaneHelper), 1e-4);
   }


   @Test
   public void testCommandOptimizeSegment2()
   {
      FramePoint3D objectivePosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);

      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      int numberOfBases = 4 * contactPolygon.getNumberOfVertices();
      int basisCoefficients = numberOfBases * LinearMPCIndexHandler.coefficientsPerRho;
      double timeOfConstraint = 0.7;

      CoMPositionCommand command = new CoMPositionCommand();
      command.setObjective(objectivePosition);
      command.setTimeOfObjective(timeOfConstraint);
      command.setSegmentNumber(1);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addContactPlaneHelper(contactPlaneHelper);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitMPCValueObjective(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      FramePoint3D solvedPositionAtConstraint = new FramePoint3D();

      DMatrixRMaj solution = solver.getSolution();
      DMatrixRMaj secondSegmentSolution = new DMatrixRMaj(LinearMPCIndexHandler.comCoefficientsPerSegment + basisCoefficients, 1);
      DMatrixRMaj rhoSolution = new DMatrixRMaj(basisCoefficients, 1);

      MatrixTools.setMatrixBlock(secondSegmentSolution, 0, 0, solution, indexHandler.getComCoefficientStartIndex(1), 0, secondSegmentSolution.getNumRows(), 1, 1.0);
      MatrixTools.setMatrixBlock(rhoSolution, 0, 0, solution, indexHandler.getRhoCoefficientStartIndex(1), 0, indexHandler.getRhoCoefficientsInSegment(1), 1, 1.0);


      DMatrixRMaj taskObjectiveExpected = new DMatrixRMaj(3, 1);
      objectivePosition.get(taskObjectiveExpected);
      taskObjectiveExpected.add(2, 0, -0.5 * timeOfConstraint * timeOfConstraint * -Math.abs(gravityZ));


      for (int rhoIdx  = 0; rhoIdx < contactPlaneHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = indexHandler.getRhoCoefficientStartIndex(1) + 4 * rhoIdx;
         double rhoValue = Math.exp(omega * timeOfConstraint) * solution.get(startIdx, 0);
         rhoValue += Math.exp(-omega * timeOfConstraint) * solution.get(startIdx + 1, 0);
         rhoValue += timeOfConstraint * timeOfConstraint * timeOfConstraint * solution.get(startIdx + 2, 0);
         rhoValue += timeOfConstraint * timeOfConstraint * solution.get(startIdx + 3, 0);

//         assertEquals(rhoValue, rhoValueVector.get(rhoIdx), 1e-5);
         solvedPositionAtConstraint.scaleAdd(rhoValue, contactPlaneHelper.getBasisVector(rhoIdx), solvedPositionAtConstraint);

      }
      solvedPositionAtConstraint.scaleAdd(0.5 * timeOfConstraint * timeOfConstraint, gravityVector, solvedPositionAtConstraint);

      EjmlUnitTests.assertEquals(MPCTestHelper.getCoMPositionJacobian(timeOfConstraint, omega, contactPlaneHelper), solver.qpInputTypeA.taskJacobian, 1e-5);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.qpInputTypeA.taskObjective, 1e-5);

      EuclidCoreTestTools.assertTuple3DEquals(objectivePosition, MPCTestHelper.computeCoMPosition(timeOfConstraint, omega, gravityZ, secondSegmentSolution, contactPlaneHelper), 1e-4);
   }


}
