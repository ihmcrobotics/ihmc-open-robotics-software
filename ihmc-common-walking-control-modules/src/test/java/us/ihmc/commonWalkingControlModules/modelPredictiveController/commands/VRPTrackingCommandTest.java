package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class VRPTrackingCommandTest
{
   @Test
   public void testCommandOptimize()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double omega2 = omega * omega;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper jacobianHelper = new CoefficientJacobianMatrixHelper(4, 4);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose1 = new FramePose3D();
      contactPose1.getPosition().set(0.0, 0.0, 0.0);

      FramePoint3D startVRP = new FramePoint3D();
      FramePoint3D endVRP = new FramePoint3D();
      startVRP.set(-0.05, -0.02, 1.0);
      endVRP.set(0.05, 0.02, 0.7);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose1, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double duration = 0.7;

      VRPTrackingCommand command = new VRPTrackingCommand();
      command.setStartVRP(startVRP);
      command.setEndVRP(endVRP);
      command.setTimeInterval(0.0, duration);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(10.0);
      command.addContactPlaneHelper(contactPlaneHelper);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitVRPTrackingCommand(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      NativeMatrix solution = solver.getSolution();
      DMatrixRMaj rhoSolution = new DMatrixRMaj(contactPlaneHelper.getRhoSize() * 4, 1);

      MatrixTools.setMatrixBlock(rhoSolution, 0, 0, solution, indexHandler.getRhoCoefficientStartIndex(0), 0, contactPlaneHelper.getCoefficientSize(), 1, 1.0);

      FramePoint3D assembledValue = new FramePoint3D();
      FramePoint3D expectedValue = new FramePoint3D();

      NativeMatrix solutionPosition = new NativeMatrix(3, 1);
      NativeMatrix jacobian = new NativeMatrix(3, indexHandler.getTotalProblemSize());

      for (double time = 0.0; time <= duration; time += 0.001)
      {
         assembledValue.setX(time * solution.get(0, 0) + solution.get(1, 0));
         assembledValue.setY(time * solution.get(2, 0) + solution.get(3, 0));
         assembledValue.setZ(time * solution.get(4, 0) + solution.get(5, 0));

         rhoHelper.computeMatrices(contactPolygon, contactPose1, 0, 0, mu);
         jacobianHelper.computeMatrices(time, omega);

         jacobian.zero();

         CoMCoefficientJacobianCalculator.calculateVRPJacobian(0, omega, time, jacobian, 0, 1.0);

         jacobian.multAddBlock(new NativeMatrix(rhoHelper.getLinearJacobianInWorldFrame()), new NativeMatrix(jacobianHelper.getPositionJacobianMatrix()), 0, 6);
         jacobian.multAddBlock(-1.0 / omega2, new NativeMatrix(rhoHelper.getLinearJacobianInWorldFrame()), new NativeMatrix(jacobianHelper.getAccelerationJacobianMatrix()), 0, 6);

         solutionPosition.mult(jacobian, solution);

         for (int pointIdx = 0; pointIdx < contactPlaneHelper.getNumberOfContactPoints(); pointIdx++)
         {
            MPCContactPoint pointHelper = contactPlaneHelper.getContactPointHelper(pointIdx);

            for (int rhoIdx = 0; rhoIdx < pointHelper.getRhoSize(); rhoIdx++)
            {
               int startIdx = indexHandler.getRhoCoefficientStartIndex(0) + pointIdx * 4 * 4 + 4 * rhoIdx;
               double rhoValue = Math.exp(omega * time) * solution.get(startIdx, 0);
               rhoValue += Math.exp(-omega * time) * solution.get(startIdx + 1, 0);
               rhoValue += time * time * time * solution.get(startIdx + 2, 0);
               rhoValue += time * time * solution.get(startIdx + 3, 0);

               rhoValue -= 1.0 / omega2 * Math.exp(omega * time) * solution.get(startIdx, 0);
               rhoValue -= 1.0 / omega2 * Math.exp(-omega * time) * solution.get(startIdx + 1, 0);
               rhoValue -= 1.0 / omega2 * 6.0 * time * solution.get(startIdx + 2, 0);
               rhoValue -= 1.0 / omega2 * 2.0 * solution.get(startIdx + 3, 0);

               assembledValue.scaleAdd(rhoValue, pointHelper.getBasisVector(rhoIdx), assembledValue);
            }
         }
         assembledValue.scaleAdd(0.5 * time * time - 1.0 / omega2, gravityVector, assembledValue);

         double alpha = time / duration;
         expectedValue.interpolate(startVRP, endVRP, alpha);

         EuclidCoreTestTools.assertTuple3DEquals(expectedValue, assembledValue, 5e-2);
      }
   }

   @Test
   public void testCubicCommandOptimize()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double omega2 = omega * omega;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper jacobianHelper = new CoefficientJacobianMatrixHelper(4, 4);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose1 = new FramePose3D();
      contactPose1.getPosition().set(0.0, 0.0, 0.0);

      FramePoint3D startVRP = new FramePoint3D();
      FramePoint3D endVRP = new FramePoint3D();
      startVRP.set(-0.05, -0.02, 1.0);
      endVRP.set(0.05, 0.02, 0.7);

      FrameVector3D startVRPVelocity = new FrameVector3D();
      FrameVector3D endVRPVelocity = new FrameVector3D();
      startVRPVelocity.set(-0.1, 0.075, -0.2);
      endVRPVelocity.set(0.15, -0.125, 0.05);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose1, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double duration = 0.7;

      VRPTrackingCommand command = new VRPTrackingCommand();
      command.setStartVRP(startVRP);
      command.setEndVRP(endVRP);
      command.setStartVRPVelocity(startVRPVelocity);
      command.setEndVRPVelocity(endVRPVelocity);
      command.setTimeInterval(0.0, duration);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(10.0);
      command.addContactPlaneHelper(contactPlaneHelper);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitVRPTrackingCommand(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      NativeMatrix solution = solver.getSolution();
      DMatrixRMaj rhoSolution = new DMatrixRMaj(contactPlaneHelper.getRhoSize() * 4, 1);

      MatrixTools.setMatrixBlock(rhoSolution, 0, 0, solution, indexHandler.getRhoCoefficientStartIndex(0), 0, contactPlaneHelper.getCoefficientSize(), 1, 1.0);

      FramePoint3D assembledValue = new FramePoint3D();

      NativeMatrix solutionPosition = new NativeMatrix(3, 1);
      NativeMatrix jacobian = new NativeMatrix(3, indexHandler.getTotalProblemSize());

      for (double time = 0.0; time <= duration; time += 0.001)
      {
         assembledValue.setX(time * solution.get(0, 0) + solution.get(1, 0));
         assembledValue.setY(time * solution.get(2, 0) + solution.get(3, 0));
         assembledValue.setZ(time * solution.get(4, 0) + solution.get(5, 0));

         rhoHelper.computeMatrices(contactPolygon, contactPose1, 0, 0, mu);
         jacobianHelper.computeMatrices(time, omega);

         jacobian.zero();

         CoMCoefficientJacobianCalculator.calculateVRPJacobian(0, omega, time, jacobian, 0, 1.0);

         jacobian.multAddBlock(new NativeMatrix(rhoHelper.getLinearJacobianInWorldFrame()), new NativeMatrix(jacobianHelper.getPositionJacobianMatrix()), 0, 6);
         jacobian.multAddBlock(-1.0 / omega2, new NativeMatrix(rhoHelper.getLinearJacobianInWorldFrame()), new NativeMatrix(jacobianHelper.getAccelerationJacobianMatrix()), 0, 6);

         solutionPosition.mult(jacobian, solution);

         Polynomial3D trajectory = new Polynomial3D(4);
         trajectory.setCubic(0.0, duration, startVRP, startVRPVelocity, endVRP, endVRPVelocity);

         for (int pointIdx = 0; pointIdx < contactPlaneHelper.getNumberOfContactPoints(); pointIdx++)
         {
            MPCContactPoint pointHelper = contactPlaneHelper.getContactPointHelper(pointIdx);

            for (int rhoIdx = 0; rhoIdx < pointHelper.getRhoSize(); rhoIdx++)
            {
               int startIdx = indexHandler.getRhoCoefficientStartIndex(0) + pointIdx * 4 * 4 + 4 * rhoIdx;
               double rhoValue = Math.exp(omega * time) * solution.get(startIdx, 0);
               rhoValue += Math.exp(-omega * time) * solution.get(startIdx + 1, 0);
               rhoValue += time * time * time * solution.get(startIdx + 2, 0);
               rhoValue += time * time * solution.get(startIdx + 3, 0);

               rhoValue -= 1.0 / omega2 * Math.exp(omega * time) * solution.get(startIdx, 0);
               rhoValue -= 1.0 / omega2 * Math.exp(-omega * time) * solution.get(startIdx + 1, 0);
               rhoValue -= 1.0 / omega2 * 6.0 * time * solution.get(startIdx + 2, 0);
               rhoValue -= 1.0 / omega2 * 2.0 * solution.get(startIdx + 3, 0);

               assembledValue.scaleAdd(rhoValue, pointHelper.getBasisVector(rhoIdx), assembledValue);
            }
         }
         assembledValue.scaleAdd(0.5 * time * time - 1.0 / omega2, gravityVector, assembledValue);
         trajectory.compute(time);

         EuclidCoreTestTools.assertTuple3DEquals(trajectory.getPosition(), assembledValue, 1e-3);
      }
   }

   @Test
   public void testCommandOptimizeNonZeroStart()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double omega2 = omega * omega;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper jacobianHelper = new CoefficientJacobianMatrixHelper(4, 4);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose1 = new FramePose3D();
      contactPose1.getPosition().set(0.0, 0.0, 0.0);

      FramePoint3D startVRP = new FramePoint3D();
      FramePoint3D endVRP = new FramePoint3D();
      startVRP.set(-0.05, -0.02, 1.0);
      endVRP.set(0.05, 0.02, 0.7);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose1, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double startTime = 0.35;
      double endTime = 0.7;

      VRPTrackingCommand command = new VRPTrackingCommand();
      command.setStartVRP(startVRP);
      command.setEndVRP(endVRP);
      command.setTimeInterval(startTime, endTime);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(10.0);
      command.addContactPlaneHelper(contactPlaneHelper);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitVRPTrackingCommand(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      NativeMatrix solution = solver.getSolution();
      DMatrixRMaj rhoSolution = new DMatrixRMaj(contactPlaneHelper.getRhoSize() * 4, 1);

      MatrixTools.setMatrixBlock(rhoSolution, 0, 0, solution, indexHandler.getRhoCoefficientStartIndex(0), 0, contactPlaneHelper.getCoefficientSize(), 1, 1.0);

      FramePoint3D assembledValue = new FramePoint3D();
      FramePoint3D expectedValue = new FramePoint3D();

      NativeMatrix solutionPosition = new NativeMatrix(3, 1);
      NativeMatrix jacobian = new NativeMatrix(3, indexHandler.getTotalProblemSize());

      Polynomial3D trajectory = new Polynomial3D(4);

      for (double time = startTime; time <= endTime; time += 0.001)
      {
         assembledValue.setX(time * solution.get(0, 0) + solution.get(1, 0));
         assembledValue.setY(time * solution.get(2, 0) + solution.get(3, 0));
         assembledValue.setZ(time * solution.get(4, 0) + solution.get(5, 0));

         rhoHelper.computeMatrices(contactPolygon, contactPose1, 0, 0, mu);
         jacobianHelper.computeMatrices(time, omega);

         jacobian.zero();

         CoMCoefficientJacobianCalculator.calculateVRPJacobian(0, omega, time, jacobian, 0, 1.0);

         jacobian.multAddBlock(new NativeMatrix(rhoHelper.getLinearJacobianInWorldFrame()), new NativeMatrix(jacobianHelper.getPositionJacobianMatrix()), 0, 6);
         jacobian.multAddBlock(-1.0 / omega2, new NativeMatrix(rhoHelper.getLinearJacobianInWorldFrame()), new NativeMatrix(jacobianHelper.getAccelerationJacobianMatrix()), 0, 6);

         solutionPosition.mult(jacobian, solution);

         for (int pointIdx = 0; pointIdx < contactPlaneHelper.getNumberOfContactPoints(); pointIdx++)
         {
            MPCContactPoint pointHelper = contactPlaneHelper.getContactPointHelper(pointIdx);

            for (int rhoIdx = 0; rhoIdx < pointHelper.getRhoSize(); rhoIdx++)
            {
               int startIdx = indexHandler.getRhoCoefficientStartIndex(0) + pointIdx * 4 * 4 + 4 * rhoIdx;
               double rhoValue = Math.exp(omega * time) * solution.get(startIdx, 0);
               rhoValue += Math.exp(-omega * time) * solution.get(startIdx + 1, 0);
               rhoValue += time * time * time * solution.get(startIdx + 2, 0);
               rhoValue += time * time * solution.get(startIdx + 3, 0);

               rhoValue -= 1.0 / omega2 * Math.exp(omega * time) * solution.get(startIdx, 0);
               rhoValue -= 1.0 / omega2 * Math.exp(-omega * time) * solution.get(startIdx + 1, 0);
               rhoValue -= 1.0 / omega2 * 6.0 * time * solution.get(startIdx + 2, 0);
               rhoValue -= 1.0 / omega2 * 2.0 * solution.get(startIdx + 3, 0);

               assembledValue.scaleAdd(rhoValue, pointHelper.getBasisVector(rhoIdx), assembledValue);
            }
         }
         assembledValue.scaleAdd(0.5 * time * time - 1.0 / omega2, gravityVector, assembledValue);

         trajectory.setLinear(startTime, endTime, startVRP, endVRP);
         trajectory.compute(time);

         EuclidCoreTestTools.assertTuple3DEquals(trajectory.getPosition(), assembledValue, 1e-2);
      }
   }

   @Test
   public void testCubicCommandOptimizeNonZeroStart()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double omega2 = omega * omega;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper jacobianHelper = new CoefficientJacobianMatrixHelper(4, 4);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose1 = new FramePose3D();
      contactPose1.getPosition().set(0.0, 0.0, 0.0);

      FramePoint3D startVRP = new FramePoint3D();
      FramePoint3D endVRP = new FramePoint3D();
      startVRP.set(-0.05, -0.02, 1.0);
      endVRP.set(0.05, 0.02, 0.7);

      FrameVector3D startVRPVelocity = new FrameVector3D();
      FrameVector3D endVRPVelocity = new FrameVector3D();
      startVRPVelocity.set(-0.1, 0.075, -0.2);
      endVRPVelocity.set(0.15, -0.125, 0.05);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose1, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double startTime = 0.3;
      double endTime = 0.7;

      VRPTrackingCommand command = new VRPTrackingCommand();
      command.setStartVRP(startVRP);
      command.setEndVRP(endVRP);
      command.setStartVRPVelocity(startVRPVelocity);
      command.setEndVRPVelocity(endVRPVelocity);
      command.setTimeInterval(startTime, endTime);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(10.0);
      command.addContactPlaneHelper(contactPlaneHelper);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitVRPTrackingCommand(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      NativeMatrix solution = solver.getSolution();
      DMatrixRMaj rhoSolution = new DMatrixRMaj(contactPlaneHelper.getRhoSize() * 4, 1);

      MatrixTools.setMatrixBlock(rhoSolution, 0, 0, solution, indexHandler.getRhoCoefficientStartIndex(0), 0, contactPlaneHelper.getCoefficientSize(), 1, 1.0);

      FramePoint3D assembledValue = new FramePoint3D();

      NativeMatrix solutionPosition = new NativeMatrix(3, 1);
      NativeMatrix jacobian = new NativeMatrix(3, indexHandler.getTotalProblemSize());

      for (double time = startTime; time <= endTime; time += 0.001)
      {
         assembledValue.setX(time * solution.get(0, 0) + solution.get(1, 0));
         assembledValue.setY(time * solution.get(2, 0) + solution.get(3, 0));
         assembledValue.setZ(time * solution.get(4, 0) + solution.get(5, 0));

         rhoHelper.computeMatrices(contactPolygon, contactPose1, 0, 0, mu);
         jacobianHelper.computeMatrices(time, omega);

         jacobian.zero();

         CoMCoefficientJacobianCalculator.calculateVRPJacobian(0, omega, time, jacobian, 0, 1.0);

         jacobian.multAddBlock(new NativeMatrix(rhoHelper.getLinearJacobianInWorldFrame()), new NativeMatrix(jacobianHelper.getPositionJacobianMatrix()), 0, 6);
         jacobian.multAddBlock(-1.0 / omega2, new NativeMatrix(rhoHelper.getLinearJacobianInWorldFrame()), new NativeMatrix(jacobianHelper.getAccelerationJacobianMatrix()), 0, 6);

         solutionPosition.mult(jacobian, solution);

         Polynomial3D trajectory = new Polynomial3D(4);
         trajectory.setCubic(startTime, endTime, startVRP, startVRPVelocity, endVRP, endVRPVelocity);

         for (int pointIdx = 0; pointIdx < contactPlaneHelper.getNumberOfContactPoints(); pointIdx++)
         {
            MPCContactPoint pointHelper = contactPlaneHelper.getContactPointHelper(pointIdx);

            for (int rhoIdx = 0; rhoIdx < pointHelper.getRhoSize(); rhoIdx++)
            {
               int startIdx = indexHandler.getRhoCoefficientStartIndex(0) + pointIdx * 4 * 4 + 4 * rhoIdx;
               double rhoValue = Math.exp(omega * time) * solution.get(startIdx, 0);
               rhoValue += Math.exp(-omega * time) * solution.get(startIdx + 1, 0);
               rhoValue += time * time * time * solution.get(startIdx + 2, 0);
               rhoValue += time * time * solution.get(startIdx + 3, 0);

               rhoValue -= 1.0 / omega2 * Math.exp(omega * time) * solution.get(startIdx, 0);
               rhoValue -= 1.0 / omega2 * Math.exp(-omega * time) * solution.get(startIdx + 1, 0);
               rhoValue -= 1.0 / omega2 * 6.0 * time * solution.get(startIdx + 2, 0);
               rhoValue -= 1.0 / omega2 * 2.0 * solution.get(startIdx + 3, 0);

               assembledValue.scaleAdd(rhoValue, pointHelper.getBasisVector(rhoIdx), assembledValue);
            }
         }
         assembledValue.scaleAdd(0.5 * time * time - 1.0 / omega2, gravityVector, assembledValue);
         trajectory.compute(time);

         EuclidCoreTestTools.assertTuple3DEquals(trajectory.getPosition(), assembledValue, 1e-3);
      }
   }
}
