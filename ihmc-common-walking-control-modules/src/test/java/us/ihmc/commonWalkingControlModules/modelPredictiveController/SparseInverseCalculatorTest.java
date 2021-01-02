package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.ops.ConvertDMatrixStruct;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeC;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.robotics.Assert.assertEquals;

public class SparseInverseCalculatorTest
{
   @Disabled
   @Test
   public void testInverse()
   {
      FramePoint3D dcmObjective = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);
      FrameVector3D comStartVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.1, 0.2, 0.4);
      FramePoint3D comStartPosition = new FramePoint3D();
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double duration = 1.0;

      double timeAtStart = 0.0;
      double timeAtEnd = duration;

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(dcmObjective);

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();
      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);



      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);




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

      VRPTrackingCommand vrpTrackingCommand = new VRPTrackingCommand();
      vrpTrackingCommand.setStartVRP(dcmObjective);
      vrpTrackingCommand.setEndVRP(dcmObjective);
      vrpTrackingCommand.setSegmentDuration(duration);
      vrpTrackingCommand.setSegmentNumber(0);

      VRPTrackingCommand vrpTrackingCommand2 = new VRPTrackingCommand();
      vrpTrackingCommand2.setStartVRP(dcmObjective);
      vrpTrackingCommand2.setEndVRP(dcmObjective);
      vrpTrackingCommand2.setSegmentDuration(duration);
      vrpTrackingCommand2.setSegmentNumber(1);

      DCMPositionCommand dcmEndPositionCommand = new DCMPositionCommand();
      dcmEndPositionCommand.setObjective(dcmObjective);
      dcmEndPositionCommand.setTimeOfObjective(timeAtEnd);
      dcmEndPositionCommand.setSegmentNumber(1);
      dcmEndPositionCommand.setOmega(omega);
      dcmEndPositionCommand.setWeight(1.0);
      dcmEndPositionCommand.addContactPlaneHelper(contactPlaneHelper);

      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, 1e-3, gravityZ, new YoRegistry("registry"));
      QPInputTypeA inputA = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeC inputC = new QPInputTypeC(indexHandler.getTotalProblemSize());

      solver.initialize();
      inputCalculator.calculateValueObjective(inputA, comStartPositionCommand);
      solver.addInput(inputA);

      inputCalculator.calculateValueObjective(inputA, comStartVelocityCommand);
      solver.addInput(inputA);

      inputCalculator.calculateValueObjective(inputA, dcmEndPositionCommand);
      solver.addInput(inputA);

      inputCalculator.calculateVRPTrackingObjective(inputC, vrpTrackingCommand);
      solver.addInput(inputC);

      inputCalculator.calculateVRPTrackingObjective(inputC, vrpTrackingCommand2);
      solver.addInput(inputC);

      solver.addValueRegularization();

      DMatrixRMaj hessian = new DMatrixRMaj(solver.solverInput_H);

      // check that it's block diagonal
      for (int row = 0; row < indexHandler.getComCoefficientStartIndex(1); row++)
      {
         for (int col = indexHandler.getComCoefficientStartIndex(1); col < indexHandler.getTotalProblemSize(); col++)
         {
            assertEquals(0.0, hessian.get(row, col), 1e-7);
            assertEquals(0.0, hessian.get(col, row), 1e-7);
         }
      }

      DMatrixRMaj inverseHessian = new DMatrixRMaj(hessian);
      DMatrixRMaj inverseHessianB = new DMatrixRMaj(hessian);
      NativeCommonOps.invert(hessian, inverseHessian);

      int blockSize = 6 + indexHandler.getRhoCoefficientsInSegment(0);
      DMatrixRMaj hessianBlock1 = new DMatrixRMaj(blockSize, blockSize);
      DMatrixRMaj hessianBlock2 = new DMatrixRMaj(blockSize, blockSize);
      DMatrixRMaj inverseHessianBlock1 = new DMatrixRMaj(blockSize, blockSize);
      DMatrixRMaj inverseHessianBlock2 = new DMatrixRMaj(blockSize, blockSize);

      CommonOps_DDRM.extract(hessian, 0, blockSize, 0, blockSize, hessianBlock1, 0, 0);
      CommonOps_DDRM.extract(hessian, blockSize, 2 * blockSize, blockSize, 2 * blockSize, hessianBlock2, 0, 0);
      NativeCommonOps.invert(hessianBlock1, inverseHessianBlock1);
      NativeCommonOps.invert(hessianBlock2, inverseHessianBlock2);
      CommonOps_DDRM.insert(inverseHessianBlock1, inverseHessianB, 0, 0);
      CommonOps_DDRM.insert(inverseHessianBlock2, inverseHessianB, blockSize, blockSize);

      EjmlUnitTests.assertEquals(inverseHessian, inverseHessianB, 1e-3);

      DMatrixSparseCSC sparseHessian = new DMatrixSparseCSC(indexHandler.getTotalProblemSize(), indexHandler.getTotalProblemSize());
      DMatrixSparseCSC sparseHessianInverse = new DMatrixSparseCSC(indexHandler.getTotalProblemSize(), indexHandler.getTotalProblemSize());

      ConvertDMatrixStruct.convert(hessian, sparseHessian, 1e-10);

      SparseInverseCalculator inverseCalculator = new SparseInverseCalculator(indexHandler);
      inverseCalculator.computeInverse(sparseHessian, sparseHessianInverse);

      EjmlUnitTests.assertEquals(inverseHessianB, sparseHessianInverse, 5e-3);
   }

}
