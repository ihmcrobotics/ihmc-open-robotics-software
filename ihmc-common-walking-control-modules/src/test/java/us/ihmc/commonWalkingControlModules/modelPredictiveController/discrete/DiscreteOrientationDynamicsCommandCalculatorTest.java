package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteOrientationDynamicsCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class DiscreteOrientationDynamicsCommandCalculatorTest
{
   @Test
   public void computeSimpleSingleSegment()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;
      double mass = 1.5;
      double height = omega * omega / gravityZ;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      DiscreteMPCIndexHandler indexHandler = new DiscreteMPCIndexHandler(4, 0.25);

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2D contactPolygon = new ConvexPolygon2D();
      contactPolygon.addVertex(0.0, 0.0);
      contactPolygon.update();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);
      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, mu);

      FramePoint3D comPositionEstimate = new FramePoint3D();
      comPositionEstimate.set(contactPose.getPosition());
      comPositionEstimate.addZ(height);

      List<ContactPlaneProvider> contacts = new ArrayList<>();
      ContactPlaneProvider contactProvider = new ContactPlaneProvider();
      contacts.add(contactProvider);
      contactProvider.addContact(contactPose, contactPolygon);

      double duration = 0.7;
      contactProvider.getTimeInterval().setInterval(0.0, duration);

      indexHandler.initialize(contacts, duration);

      DiscreteOrientationDynamicsCommandCalculator calculator = new DiscreteOrientationDynamicsCommandCalculator(indexHandler, mass);


      double yaw = 0.1;
      double pitch = -0.1;
      double roll = 0.05;

      SpatialInertia spatialInertia = new SpatialInertia(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), 10, 10, 5, mass);

      FrameQuaternion orientation = new FrameQuaternion();
      orientation.setYawPitchRoll(yaw, pitch, roll);

      FrameVector3D angularVelocity = new FrameVector3D();
      angularVelocity.set(0.1, 0.2, 0.3);
      DMatrixRMaj angularVelocityVector = new DMatrixRMaj(3, 1);
      angularVelocity.get(angularVelocityVector);

      DiscreteOrientationDynamicsCommand command = new DiscreteOrientationDynamicsCommand();
      command.addContactPlaneHelper(0, contactPlaneHelper);
      command.addSegmentDuration(duration);
      command.setComPositionEstimate(comPositionEstimate);
      command.setOmega(omega);
      command.setOrientationEstimate(orientation);
      command.setAngularVelocityEstimate(angularVelocity);
      command.setBodyInertia(spatialInertia);

      calculator.compute(command);

      DMatrixRMaj Cmatrix = new DMatrixRMaj(3, 3);
      RotationMatrix rotation = new RotationMatrix();
      orientation.get(rotation);
      DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
      rotation.get(rotationMatrix);

      Cmatrix.set(0, 0, Math.cos(pitch) * Math.cos(yaw));
      Cmatrix.set(0, 1, -Math.sin(yaw));
      Cmatrix.set(1, 0, Math.cos(pitch) * Math.sin(yaw));
      Cmatrix.set(1, 1, Math.cos(yaw));
      Cmatrix.set(2, 1, -Math.sin(pitch));

      DMatrixRMaj A = new DMatrixRMaj(6, 6);
      DMatrixRMaj Ad = new DMatrixRMaj(6, 6);
      MatrixTools.setMatrixBlock(A, 0, 3, Cmatrix, 0, 0, 3, 3, 1.0);

      MatrixExponentialCalculator eAT = new MatrixExponentialCalculator(6);
      CommonOps_DDRM.scale(indexHandler.getOrientationDt(), A);
      eAT.compute(Ad, A);

      DMatrixRMaj rhsJacobian = new DMatrixRMaj(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), indexHandler.getTotalProblemSize());
      DMatrixRMaj lhsJacobian = new DMatrixRMaj(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), indexHandler.getTotalProblemSize());

      DMatrixRMaj rhsConstant = new DMatrixRMaj(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), 1);
      DMatrixRMaj lhsConstant = new DMatrixRMaj(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), 1);


      DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
      DMatrixRMaj inertiaTemp = new DMatrixRMaj(3, 3);
      DMatrixRMaj inertiaInWorld = new DMatrixRMaj(3, 3);
      DMatrixRMaj inertiaInWorldInverse = new DMatrixRMaj(3, 3);
      spatialInertia.getMomentOfInertia().get(inertia);
      CommonOps_DDRM.mult(rotationMatrix, inertia, inertiaTemp);
      CommonOps_DDRM.multTransB(inertiaTemp, rotationMatrix, inertiaInWorld);
      MatrixMissingTools.fast3x3Inverse(inertiaInWorld, inertiaInWorldInverse);


      DMatrixRMaj identity = CommonOps_DDRM.identity(6);

      // the first tick should be equal to the state transition matrix times the initial angular velocity.
      DMatrixRMaj firstTickConst = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(Cmatrix, angularVelocityVector, firstTickConst);
      MatrixTools.setMatrixBlock(rhsConstant, 0, 0, firstTickConst, 0, 0, 3, 1, 1.0);

      for (int tick = 0; tick < indexHandler.getTotalOrientationTicks() - 1; tick++)
      {
         int segment = indexHandler.getSegmentForTick(tick);
         int nextTickSegment = indexHandler.getSegmentForTick(tick + 1);
         int startIndexOfSegment = indexHandler.getOrientationStart(segment);
         int startIndexOfSegmentNextTick = indexHandler.getOrientationStart(nextTickSegment);
         int ticksIntoSegment = indexHandler.getOrientationTicksBeforeSegment(segment);
         int ticksIntoSegmentNext = indexHandler.getOrientationTicksBeforeSegment(nextTickSegment);
         int currentVariables = startIndexOfSegment + ticksIntoSegment * DiscreteMPCIndexHandler.orientationVariablesPerTick;
         int nextTickVariables = startIndexOfSegmentNextTick + ticksIntoSegmentNext * DiscreteMPCIndexHandler.orientationVariablesPerTick;

         double timeInSegment = tick * indexHandler.getOrientationDt();
         for (int i = 0; i < segment; i++)
            timeInSegment -= command.getSegmentDuration(i);

         if (tick > 0)
            MatrixTools.setMatrixBlock(rhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, currentVariables, Ad, 0, 0, 6, 6, 1.0);

         MatrixTools.setMatrixBlock(lhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, nextTickVariables, identity, 0, 0, 6, 6, 1.0);

         contactPlaneHelper.computeJacobians(timeInSegment, omega);
         DMatrixRMaj torqueJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         DMatrixRMaj angularAccelerationJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         int coefficientOffset = 0;
         for (int contactIdx = 0; contactIdx < contactPlaneHelper.getNumberOfContactPoints(); contactIdx++)
         {
            ContactPointHelper contactPoint = contactPlaneHelper.getContactPointHelper(contactIdx);
            Vector3D contactRelative = new Vector3D();
            contactRelative.sub(contactPoint.getBasisVectorOrigin(), comPositionEstimate);
            DMatrixRMaj contactLocationSkew = new DMatrixRMaj(3, 3);
            DiscreteOrientationDynamicsCommandCalculator.convertToSkewSymmetric(contactRelative, contactLocationSkew);
            DMatrixRMaj pointTorqueJacobian = new DMatrixRMaj(3, contactPoint.getCoefficientsSize());
            CommonOps_DDRM.mult(mass, contactLocationSkew, contactPoint.getLinearAccelerationJacobian(), pointTorqueJacobian);

            int startCol = indexHandler.getRhoCoefficientStartIndex(segment) + coefficientOffset;
            MatrixTools.addMatrixBlock(torqueJacobian, 0, startCol, pointTorqueJacobian, 0, 0, 3, contactPoint.getCoefficientsSize(), 1.0);

            coefficientOffset += contactPoint.getCoefficientsSize();
         }

         CommonOps_DDRM.mult(inertiaInWorldInverse, torqueJacobian, angularAccelerationJacobian);

         DMatrixRMaj B = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
         MatrixTools.setMatrixBlock(B, 3, 0, angularAccelerationJacobian, 0, 0, 3, indexHandler.getTotalProblemSize(), 1.0);

         DMatrixRMaj fullStateMatrix = new DMatrixRMaj(6 + indexHandler.getTotalProblemSize(), 6 + indexHandler.getTotalProblemSize());
         DMatrixRMaj fullStateExponential = new DMatrixRMaj(6 + indexHandler.getTotalProblemSize(), 6 + indexHandler.getTotalProblemSize());
         MatrixTools.setMatrixBlock(fullStateMatrix, 0, 0, A, 0, 0, 6, 6, 1.0);
         MatrixTools.setMatrixBlock(fullStateMatrix, 0, 6, B, 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);

         MatrixExponentialCalculator fullStateExponentialCalculator = new MatrixExponentialCalculator(6 + indexHandler.getTotalProblemSize());
         CommonOps_DDRM.scale(indexHandler.getOrientationDt(), fullStateMatrix);
         fullStateExponentialCalculator.compute(fullStateExponential, fullStateMatrix);

         DMatrixRMaj Bd = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
         MatrixTools.setMatrixBlock(Bd, 0, 0, fullStateExponential, 0, 6, 6, indexHandler.getTotalProblemSize(), 1.0);

         MatrixTools.addMatrixBlock(rhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, 0, Bd, 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
      }

      MatrixTestTools.assertMatrixEquals(rhsJacobian, calculator.getRhsJacobian(), 1e-5);
      MatrixTestTools.assertMatrixEquals(lhsJacobian, calculator.getLhsJacobian(), 1e-5);
   }

   @Test
   public void computeSingleSegment()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;
      double mass = 1.5;
      double height = omega * omega / gravityZ;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      DiscreteMPCIndexHandler indexHandler = new DiscreteMPCIndexHandler(4);

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);
      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, mu);

      FramePoint3D comPositionEstimate = new FramePoint3D();
      comPositionEstimate.set(contactPose.getPosition());
      comPositionEstimate.addZ(height);

      List<ContactPlaneProvider> contacts = new ArrayList<>();
      ContactPlaneProvider contactProvider = new ContactPlaneProvider();
      contacts.add(contactProvider);
      contactProvider.addContact(contactPose, contactPolygon);

      double duration = 0.7;
      contactProvider.getTimeInterval().setInterval(0.0, duration);

      indexHandler.initialize(contacts, duration);

      DiscreteOrientationDynamicsCommandCalculator calculator = new DiscreteOrientationDynamicsCommandCalculator(indexHandler, mass);


      double yaw = 0.1;
      double pitch = -0.1;
      double roll = 0.05;

      SpatialInertia spatialInertia = new SpatialInertia(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), 10, 10, 5, mass);

      FrameQuaternion orientation = new FrameQuaternion();
      orientation.setYawPitchRoll(yaw, pitch, roll);

      FrameVector3D angularVelocity = new FrameVector3D();
      angularVelocity.set(0.1, 0.2, 0.3);
      DMatrixRMaj angularVelocityVector = new DMatrixRMaj(3, 1);
      angularVelocity.get(angularVelocityVector);

      DiscreteOrientationDynamicsCommand command = new DiscreteOrientationDynamicsCommand();
      command.addContactPlaneHelper(0, contactPlaneHelper);
      command.addSegmentDuration(duration);
      command.setComPositionEstimate(comPositionEstimate);
      command.setOmega(omega);
      command.setOrientationEstimate(orientation);
      command.setAngularVelocityEstimate(angularVelocity);
      command.setBodyInertia(spatialInertia);

      calculator.compute(command);

      DMatrixRMaj Cmatrix = new DMatrixRMaj(3, 3);
      RotationMatrix rotation = new RotationMatrix();
      orientation.get(rotation);
      DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
      rotation.get(rotationMatrix);

      Cmatrix.set(0, 0, Math.cos(pitch) * Math.cos(yaw));
      Cmatrix.set(0, 1, -Math.sin(yaw));
      Cmatrix.set(1, 0, Math.cos(pitch) * Math.sin(yaw));
      Cmatrix.set(1, 1, Math.cos(yaw));
      Cmatrix.set(2, 1, -Math.sin(pitch));

      DMatrixRMaj A = new DMatrixRMaj(6, 6);
      DMatrixRMaj Ad = new DMatrixRMaj(6, 6);
      MatrixTools.setMatrixBlock(A, 0, 3, Cmatrix, 0, 0, 3, 3, 1.0);

      MatrixExponentialCalculator eAT = new MatrixExponentialCalculator(6);
      CommonOps_DDRM.scale(indexHandler.getOrientationDt(), A);
      eAT.compute(Ad, A);

      DMatrixRMaj rhsJacobian = new DMatrixRMaj(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), indexHandler.getTotalProblemSize());
      DMatrixRMaj lhsJacobian = new DMatrixRMaj(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), indexHandler.getTotalProblemSize());

      DMatrixRMaj rhsConstant = new DMatrixRMaj(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), 1);
      DMatrixRMaj lhsConstant = new DMatrixRMaj(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), 1);


      DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
      DMatrixRMaj inertiaTemp = new DMatrixRMaj(3, 3);
      DMatrixRMaj inertiaInWorld = new DMatrixRMaj(3, 3);
      DMatrixRMaj inertiaInWorldInverse = new DMatrixRMaj(3, 3);
      spatialInertia.getMomentOfInertia().get(inertia);
      CommonOps_DDRM.mult(rotationMatrix, inertia, inertiaTemp);
      CommonOps_DDRM.multTransB(inertiaTemp, rotationMatrix, inertiaInWorld);
      MatrixMissingTools.fast3x3Inverse(inertiaInWorld, inertiaInWorldInverse);


      DMatrixRMaj identity = CommonOps_DDRM.identity(6);

      // the first tick should be equal to the state transition matrix times the initial angular velocity.
      DMatrixRMaj firstTickConst = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(Cmatrix, angularVelocityVector, firstTickConst);
      MatrixTools.setMatrixBlock(rhsConstant, 0, 0, firstTickConst, 0, 0, 3, 1, 1.0);

      for (int tick = 0; tick < indexHandler.getTotalOrientationTicks() - 1; tick++)
      {
         int segment = indexHandler.getSegmentForTick(tick);
         int nextTickSegment = indexHandler.getSegmentForTick(tick + 1);
         int startIndexOfSegment = indexHandler.getOrientationStart(segment);
         int startIndexOfSegmentNextTick = indexHandler.getOrientationStart(nextTickSegment);
         int ticksIntoSegment = indexHandler.getOrientationTicksBeforeSegment(segment);
         int ticksIntoSegmentNext = indexHandler.getOrientationTicksBeforeSegment(nextTickSegment);
         int currentVariables = startIndexOfSegment + ticksIntoSegment * DiscreteMPCIndexHandler.orientationVariablesPerTick;
         int nextTickVariables = startIndexOfSegmentNextTick + ticksIntoSegmentNext * DiscreteMPCIndexHandler.orientationVariablesPerTick;

         double timeInSegment = tick * indexHandler.getOrientationDt();
         for (int i = 0; i < segment; i++)
            timeInSegment -= command.getSegmentDuration(i);

         if (tick > 0)
            MatrixTools.setMatrixBlock(rhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, currentVariables, Ad, 0, 0, 6, 6, 1.0);

         MatrixTools.setMatrixBlock(lhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, nextTickVariables, identity, 0, 0, 6, 6, 1.0);

         contactPlaneHelper.computeJacobians(timeInSegment, omega);
         DMatrixRMaj torqueJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         DMatrixRMaj angularAccelerationJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         int coefficientOffset = 0;
         for (int contactIdx = 0; contactIdx < contactPlaneHelper.getNumberOfContactPoints(); contactIdx++)
         {
            ContactPointHelper contactPoint = contactPlaneHelper.getContactPointHelper(contactIdx);
            Vector3D contactRelative = new Vector3D();
            contactRelative.sub(contactPoint.getBasisVectorOrigin(), comPositionEstimate);
            DMatrixRMaj contactLocationSkew = new DMatrixRMaj(3, 3);
            DiscreteOrientationDynamicsCommandCalculator.convertToSkewSymmetric(contactRelative, contactLocationSkew);
            DMatrixRMaj pointTorqueJacobian = new DMatrixRMaj(3, contactPoint.getCoefficientsSize());
            CommonOps_DDRM.mult(mass, contactLocationSkew, contactPoint.getLinearAccelerationJacobian(), pointTorqueJacobian);

            int startCol = indexHandler.getRhoCoefficientStartIndex(segment) + coefficientOffset;
            MatrixTools.addMatrixBlock(torqueJacobian, 0, startCol, pointTorqueJacobian, 0, 0, 3, contactPoint.getCoefficientsSize(), 1.0);

            coefficientOffset += contactPoint.getCoefficientsSize();
         }

         CommonOps_DDRM.mult(inertiaInWorldInverse, torqueJacobian, angularAccelerationJacobian);

         DMatrixRMaj B = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
         MatrixTools.setMatrixBlock(B, 3, 0, angularAccelerationJacobian, 0, 0, 3, indexHandler.getTotalProblemSize(), 1.0);

         DMatrixRMaj fullStateMatrix = new DMatrixRMaj(6 + indexHandler.getTotalProblemSize(), 6 + indexHandler.getTotalProblemSize());
         DMatrixRMaj fullStateExponential = new DMatrixRMaj(6 + indexHandler.getTotalProblemSize(), 6 + indexHandler.getTotalProblemSize());
         MatrixTools.setMatrixBlock(fullStateMatrix, 0, 0, A, 0, 0, 6, 6, 1.0);
         MatrixTools.setMatrixBlock(fullStateMatrix, 0, 6, B, 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);

         MatrixExponentialCalculator fullStateExponentialCalculator = new MatrixExponentialCalculator(6 + indexHandler.getTotalProblemSize());
         CommonOps_DDRM.scale(indexHandler.getOrientationDt(), fullStateMatrix);
         fullStateExponentialCalculator.compute(fullStateExponential, fullStateMatrix);

         DMatrixRMaj Bd = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
         MatrixTools.setMatrixBlock(Bd, 0, 0, fullStateExponential, 0, 6, 6, indexHandler.getTotalProblemSize(), 1.0);

         MatrixTools.addMatrixBlock(rhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, 0, Bd, 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
      }

      MatrixTestTools.assertMatrixEquals(rhsJacobian, calculator.getRhsJacobian(), 1e-5);
      MatrixTestTools.assertMatrixEquals(lhsJacobian, calculator.getLhsJacobian(), 1e-5);
   }
}
