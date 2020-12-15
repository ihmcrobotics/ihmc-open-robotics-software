package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.VRPPositionContinuityCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.matrixlib.MatrixTools;
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


      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
      CoMMPCQPSolver solver = new CoMMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose1 = new FramePose3D();
      contactPose1.getPosition().set(0.0, 0.0, 0.0);

      FramePoint3D startVRP = new FramePoint3D();
      FramePoint3D endVRP = new FramePoint3D();
      startVRP.set(-0.05, -0.02, 0.0);
      endVRP.set(0.05, 0.02, 0.0);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose1, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      double duration1 = 0.7;

      VRPTrackingCommand command = new VRPTrackingCommand();
      command.setSegmentDuration(duration1);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addContactPlaneHelper(contactPlaneHelper);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitVRPTrackingCommand(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      DMatrixRMaj solution = solver.getSolution();
      DMatrixRMaj rhoSolution = new DMatrixRMaj(contactPlaneHelper.getRhoSize()  * 4, 1);

      MatrixTools.setMatrixBlock(rhoSolution, 0, 0, solution, 12, 0, contactPlaneHelper.getRhoSize() * 4, 1, 1.0);


      DMatrixRMaj taskHessianExpected = new DMatrixRMaj(3, 2 * 6 + contactPlaneHelper.getRhoSize() * 4);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(0, duration1, taskHessianExpected, 0, 1.0);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(0, duration1, taskHessianExpected, 2, -1.0 / omega2);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(1, 0.0, taskHessianExpected, 0, -1.0);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(1, 0.0, taskHessianExpected, 2, 1.0 / omega2);



      FramePoint3D assembledValue = new FramePoint3D();
      FramePoint3D expectedValue = new FramePoint3D();


      for (double time = 0.0; time <= duration1; time += 0.001)
      {
         assembledValue.setX(time * solution.get(0, 0) + solution.get(1, 0));
         assembledValue.setY(time * solution.get(2, 0) + solution.get(3, 0));
         assembledValue.setZ(time * solution.get(4, 0) + solution.get(5, 0));

         for (int pointIdx = 0; pointIdx < contactPlaneHelper.getNumberOfContactPoints(); pointIdx++)
         {
            ContactPointHelper pointHelper = contactPlaneHelper.getContactPointHelper(pointIdx);

            for (int rhoIdx = 0; rhoIdx < pointHelper.getRhoSize(); rhoIdx++)
            {
               int startIdx = 12 + pointIdx * 4 * 4 + 4 * rhoIdx;
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

         double alpha = time / duration1;
         expectedValue.interpolate(startVRP, endVRP, alpha);

         EuclidCoreTestTools.assertTuple3DEquals(expectedValue, assembledValue, 1e-4);

      }
   }


}
