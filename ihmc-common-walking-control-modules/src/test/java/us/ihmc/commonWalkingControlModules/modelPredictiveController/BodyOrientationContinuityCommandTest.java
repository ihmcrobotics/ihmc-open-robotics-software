package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.BodyOrientationContinuityCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.CoMPositionContinuityCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BodyOrientationContinuityCommandTest
{
   @Test
   public void testCommandOptimize()
   {
      FramePoint3D objectivePosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);

      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double mass = 1.5;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper1 = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      ContactStateMagnitudeToForceMatrixHelper rhoHelper2 = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);

      ContactPlaneHelper contactPlaneHelper1 = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());
      ContactPlaneHelper contactPlaneHelper2 = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
      CoMMPCQPSolver solver = new CoMMPCQPSolver(indexHandler, dt, mass, gravityZ, new YoRegistry("test"));

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

      BodyOrientationContinuityCommand command = new BodyOrientationContinuityCommand();
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
      solver.setOrientationCoefficientRegularization(regularization);

      solver.solve();

      DMatrixRMaj solvedObjectivePosition = new DMatrixRMaj(3, 1);
      FramePoint3D solvedObjectivePositionTuple = new FramePoint3D();

      FramePoint3D valueEndOf1 = new FramePoint3D();
      FramePoint3D valueStartOf2 = new FramePoint3D();

      DMatrixRMaj solution = solver.getSolution();

      CommonOps_DDRM.mult(solver.qpInputTypeA.taskJacobian, solution, solvedObjectivePosition);
      solvedObjectivePositionTuple.set(solvedObjectivePosition);

      DMatrixRMaj taskObjectiveExpected = new DMatrixRMaj(3, 1);
      DMatrixRMaj achievedObjective = new DMatrixRMaj(3, 1);

      DMatrixRMaj taskJacobianExpected = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
      OrientationCoefficientJacobianCalculator.calculateAngularJacobian(indexHandler.getYawCoefficientsStartIndex(0),
                                                                        indexHandler.getPitchCoefficientsStartIndex(0),
                                                                        indexHandler.getRollCoefficientsStartIndex(0),
                                                                        omega,
                                                                        duration1,
                                                                        taskJacobianExpected,
                                                                        0,
                                                                        1.0);
      OrientationCoefficientJacobianCalculator.calculateAngularJacobian(indexHandler.getYawCoefficientsStartIndex(1),
                                                                        indexHandler.getPitchCoefficientsStartIndex(1),
                                                                        indexHandler.getRollCoefficientsStartIndex(1),
                                                                        omega,
                                                                        0.0,
                                                                        taskJacobianExpected,
                                                                        0,
                                                                        -1.0);

      int startIndex0 = indexHandler.getOrientationCoefficientsStartIndex(0);
      valueEndOf1.setX(duration1 * duration1 * duration1 * solution.get(startIndex0, 0) + duration1 * duration1 * solution.get(startIndex0 + 1, 0)
                       + duration1 * solution.get(startIndex0 + 2, 0) + solution.get(startIndex0 + 3));
      valueEndOf1.setY(duration1 * duration1 * duration1 * solution.get(startIndex0 + 4, 0) + duration1 * duration1 * solution.get(startIndex0 + 5, 0)
                       + duration1 * solution.get(startIndex0 + 6, 0) + solution.get(startIndex0 + 7));
      valueEndOf1.setZ(duration1 * duration1 * duration1 * solution.get(startIndex0 + 8, 0) + duration1 * duration1 * solution.get(startIndex0 + 9, 0)
                       + duration1 * solution.get(startIndex0 + 10, 0) + solution.get(startIndex0 + 11));

      int startIndex1 = indexHandler.getOrientationCoefficientsStartIndex(0);

      valueStartOf2.setX(solution.get(startIndex1 + 3, 0));
      valueStartOf2.setY(solution.get(startIndex1 + 7, 0));
      valueStartOf2.setZ(solution.get(startIndex1 + 11, 0));

      EjmlUnitTests.assertEquals(taskJacobianExpected, solver.qpInputTypeA.taskJacobian, 1e-5);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.qpInputTypeA.taskObjective, 1e-5);

      CommonOps_DDRM.mult(taskJacobianExpected, solution, achievedObjective);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, achievedObjective, 1e-4);

      FramePoint3D desiredValue = new FramePoint3D();
      EuclidCoreTestTools.assertTuple3DEquals(valueEndOf1, valueStartOf2, 1e-4);
      EuclidCoreTestTools.assertTuple3DEquals(desiredValue, solvedObjectivePositionTuple, 1e-4);
   }
}
