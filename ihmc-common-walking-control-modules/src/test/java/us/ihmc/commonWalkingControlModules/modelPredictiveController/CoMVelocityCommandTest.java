package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.CoMPositionCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.CoMVelocityCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class CoMVelocityCommandTest
{
   @Test
   public void testCommandOptimize()
   {
      FramePoint3D objectiveVelocity = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);

      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
      CoMMPCQPSolver solver = new CoMMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose = new FramePose3D();

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartCopPosition(new FramePoint3D());
      contact.setEndCopPosition(new FramePoint3D());

      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, mu);

      contactProviders.add(contact);

      indexHandler.initialize(contactProviders);

      double timeOfConstraint = 0.7;

      CoMVelocityCommand command = new CoMVelocityCommand();
      command.setObjective(objectiveVelocity);
      command.setTimeOfObjective(timeOfConstraint);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addRhoToForceMatrixHelper(rhoHelper);
      command.addJacobianMatrixHelper(helper);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitMPCValueObjective(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      FramePoint3D solvedVelocityAtConstraint = new FramePoint3D();
      FramePoint3D solvedObjectiveVelocityTuple = new FramePoint3D();
      DMatrixRMaj rhoValueVector = new DMatrixRMaj(rhoHelper.getRhoSize(), 1);
      DMatrixRMaj solvedObjectiveVelocity = new DMatrixRMaj(3, 1);

      DMatrixRMaj solution = solver.getSolution();
      DMatrixRMaj rhoSolution = new DMatrixRMaj(rhoHelper.getRhoSize() * 4, 1);
      solvedVelocityAtConstraint.addX(solution.get(0, 0));
      solvedVelocityAtConstraint.addY(solution.get(2, 0));
      solvedVelocityAtConstraint.addZ(solution.get(4, 0));

      MatrixTools.setMatrixBlock(rhoSolution, 0, 0, solution, 6, 0, rhoHelper.getRhoSize() * 4, 1, 1.0);

      CommonOps_DDRM.mult(helper.getVelocityJacobianMatrix(), rhoSolution, rhoValueVector);

      CommonOps_DDRM.mult(solver.qpInput.taskJacobian, solution, solvedObjectiveVelocity);
      solvedObjectiveVelocityTuple.set(solvedObjectiveVelocity);
      solvedObjectiveVelocityTuple.scaleAdd(timeOfConstraint, gravityVector, solvedObjectiveVelocityTuple);

      DMatrixRMaj taskObjectiveExpected = new DMatrixRMaj(3, 1);
      DMatrixRMaj achievedObjective = new DMatrixRMaj(3, 1);
      objectiveVelocity.get(taskObjectiveExpected);
      taskObjectiveExpected.add(2, 0, -timeOfConstraint * -Math.abs(gravityZ));

      DMatrixRMaj taskJacobianExpected = new DMatrixRMaj(3, 6 + rhoHelper.getRhoSize() * 4);
      taskJacobianExpected.set(0, 0, 1.0);
      taskJacobianExpected.set(1, 2, 1.0);
      taskJacobianExpected.set(2, 4, 1.0);



      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;
         double rhoValue = omega * Math.exp(omega * timeOfConstraint) * solution.get(startIdx, 0);
         rhoValue += -omega * Math.exp(-omega * timeOfConstraint) * solution.get(startIdx + 1, 0);
         rhoValue += 3.0 * timeOfConstraint * timeOfConstraint * solution.get(startIdx + 2, 0);
         rhoValue += 2.0 * timeOfConstraint * solution.get(startIdx + 3, 0);

         assertEquals(rhoValue, rhoValueVector.get(rhoIdx), 1e-5);
         solvedVelocityAtConstraint.scaleAdd(rhoValue, rhoHelper.getBasisVector(rhoIdx), solvedVelocityAtConstraint);

         taskJacobianExpected.set(0, startIdx, rhoHelper.getBasisVector(rhoIdx).getX() * omega * Math.exp(omega * timeOfConstraint));
         taskJacobianExpected.set(0, startIdx + 1, rhoHelper.getBasisVector(rhoIdx).getX() * -omega * Math.exp(-omega * timeOfConstraint));
         taskJacobianExpected.set(0, startIdx + 2, rhoHelper.getBasisVector(rhoIdx).getX() * 3.0 * timeOfConstraint * timeOfConstraint);
         taskJacobianExpected.set(0, startIdx + 3, rhoHelper.getBasisVector(rhoIdx).getX() * 2.0 * timeOfConstraint);
         taskJacobianExpected.set(1, startIdx, rhoHelper.getBasisVector(rhoIdx).getY() * omega * Math.exp(omega * timeOfConstraint));
         taskJacobianExpected.set(1, startIdx + 1, rhoHelper.getBasisVector(rhoIdx).getY() * -omega * Math.exp(-omega * timeOfConstraint));
         taskJacobianExpected.set(1, startIdx + 2, rhoHelper.getBasisVector(rhoIdx).getY() * 3.0 * timeOfConstraint * timeOfConstraint);
         taskJacobianExpected.set(1, startIdx + 3, rhoHelper.getBasisVector(rhoIdx).getY() * 2.0 * timeOfConstraint);
         taskJacobianExpected.set(2, startIdx, rhoHelper.getBasisVector(rhoIdx).getZ() * omega * Math.exp(omega * timeOfConstraint));
         taskJacobianExpected.set(2, startIdx + 1, rhoHelper.getBasisVector(rhoIdx).getZ() * -omega * Math.exp(-omega * timeOfConstraint));
         taskJacobianExpected.set(2, startIdx + 2, rhoHelper.getBasisVector(rhoIdx).getZ() * 3.0 * timeOfConstraint * timeOfConstraint);
         taskJacobianExpected.set(2, startIdx + 3, rhoHelper.getBasisVector(rhoIdx).getZ() * 2.0 * timeOfConstraint);
      }
      solvedVelocityAtConstraint.scaleAdd(timeOfConstraint, gravityVector, solvedVelocityAtConstraint);

      EjmlUnitTests.assertEquals(taskJacobianExpected, solver.qpInput.taskJacobian, 1e-5);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.qpInput.taskObjective, 1e-5);

      CommonOps_DDRM.mult(taskJacobianExpected, solution, achievedObjective);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, achievedObjective, 1e-4);

      DMatrixRMaj solverInput_H_Expected = new DMatrixRMaj(taskJacobianExpected.getNumCols(), taskJacobianExpected.getNumCols());
      DMatrixRMaj solverInput_f_Expected = new DMatrixRMaj(taskJacobianExpected.getNumCols(), 1);

      CommonOps_DDRM.multInner(taskJacobianExpected, solverInput_H_Expected);
      CommonOps_DDRM.multTransA(-1.0, taskJacobianExpected, taskObjectiveExpected, solverInput_f_Expected);

      MatrixTools.addDiagonal(solverInput_H_Expected, regularization);

      EjmlUnitTests.assertEquals(solverInput_H_Expected, solver.solverInput_H, 1e-10);
      EjmlUnitTests.assertEquals(solverInput_f_Expected, solver.solverInput_f, 1e-10);


      EuclidCoreTestTools.assertTuple3DEquals(objectiveVelocity, solvedObjectiveVelocityTuple, 1e-4);
      EuclidCoreTestTools.assertTuple3DEquals(objectiveVelocity, solvedVelocityAtConstraint, 1e-4);
   }

}
