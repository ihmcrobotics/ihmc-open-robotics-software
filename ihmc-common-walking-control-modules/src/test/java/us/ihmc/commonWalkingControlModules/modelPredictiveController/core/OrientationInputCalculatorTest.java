package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteOrientationCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class OrientationInputCalculatorTest
{
   @Test
   public void testCoMObjectiveOneSegment()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double mass = 10.0;
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));
         DMatrixRMaj fullCoefficientVector = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
         MatrixTools.setMatrixBlock(fullCoefficientVector, 0, 0, trajectoryCoefficients, 0, 0, numberOfTrajectoryCoefficients, 1, 1.0);

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));
         contactPlane.computeContactForce(omega, time);

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         FrameVector3DReadOnly comVelocity = MPCTestHelper.computeCoMVelocity(time, omega, gravityZ, trajectoryCoefficients, contactPlane);

         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularMomentumAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         FrameVector3D tempVector = new FrameVector3D();
         FrameVector3D expectedAngularErrorRateFromAngularError = new FrameVector3D();
         desiredBodyOrientation.inverseTransform(desiredBodyAngularMomentum, tempVector);
         expectedAngularErrorRateFromAngularError.cross(tempVector, angularErrorAtCurrentTick);
         momentOfInertia.inverseTransform(expectedAngularErrorRateFromAngularError);

         tempVector.cross(angularErrorAtCurrentTick, desiredBodyAngularVelocity);
         expectedAngularErrorRateFromAngularError.add(tempVector);

         FrameVector3D expectedAngularErrorRateFromAngularMomentum = new FrameVector3D(angularMomentumAtCurrentTick);
         desiredBodyOrientation.inverseTransform(expectedAngularErrorRateFromAngularMomentum);
         momentOfInertia.inverseTransform(expectedAngularErrorRateFromAngularMomentum);

         FrameVector3D coriolisForceEstimated = new FrameVector3D();
         FrameVector3D coriolisForce = new FrameVector3D();

         coriolisForceEstimated.cross(desiredCoMPosition, comVelocity);
         tempVector.sub(comPosition, desiredCoMPosition);
         tempVector.cross(desiredCoMVelocity);
         coriolisForceEstimated.add(tempVector);
         coriolisForceEstimated.scale(mass);

         coriolisForce.cross(comPosition, comVelocity);
         coriolisForce.scale(mass);

         EuclidCoreTestTools.assertVector3DGeometricallyEquals(coriolisForceEstimated, coriolisForceEstimated, 1e-5);


         FrameVector3D expectedAngularErrorRateFromCoriolis = new FrameVector3D(coriolisForceEstimated);
         desiredBodyOrientation.inverseTransform(expectedAngularErrorRateFromCoriolis);
         momentOfInertia.inverseTransform(expectedAngularErrorRateFromCoriolis);
         expectedAngularErrorRateFromCoriolis.scale(-1.0);

         FrameVector3D expectedAngularErrorRateFromInternalMomentum = new FrameVector3D(desiredInternalAngularMomentum);
         desiredBodyOrientation.inverseTransform(expectedAngularErrorRateFromInternalMomentum);
         momentOfInertia.inverseTransform(expectedAngularErrorRateFromInternalMomentum);
         expectedAngularErrorRateFromInternalMomentum.scale(-1.0);

         FrameVector3D expectedAngularErrorRate = new FrameVector3D();
         expectedAngularErrorRate.add(expectedAngularErrorRateFromAngularError);
         expectedAngularErrorRate.add(expectedAngularErrorRateFromAngularMomentum);
         expectedAngularErrorRate.add(expectedAngularErrorRateFromCoriolis);
         expectedAngularErrorRate.add(expectedAngularErrorRateFromInternalMomentum);

         FrameVector3D expectedTorqueFromGravity = new FrameVector3D();
         expectedTorqueFromGravity.cross(comPosition, gravityVector);
         expectedTorqueFromGravity.scale(mass);

         FrameVector3D expectedTorqueFromContactPlane = new FrameVector3D();

         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);
            contactPoint.computeContactForce(omega, time);

            FrameVector3D expectedTorqueFromContactPoint = new FrameVector3D();
            expectedTorqueFromContactPoint.cross(contactPoint.getBasisVectorOrigin(), contactPoint.getContactAcceleration());
            expectedTorqueFromContactPoint.scale(mass);

            expectedTorqueFromContactPlane.add(expectedTorqueFromContactPoint);
         }

         FrameVector3D expectedTorqueInternal = new FrameVector3D(desiredInternalAngularMomentumRate);
         expectedTorqueInternal.scale(-1.0);

         FrameVector3D expectedTorque = new FrameVector3D();
         expectedTorque.add(expectedTorqueInternal);
         expectedTorque.add(expectedTorqueFromGravity);
         expectedTorque.add(expectedTorqueFromContactPlane);

         DMatrixRMaj compactStateVector = new DMatrixRMaj(6, 1);
         angularErrorAtCurrentTick.get(compactStateVector);
         angularMomentumAtCurrentTick.get(3, compactStateVector);

         DMatrixRMaj justAngularErrorState = new DMatrixRMaj(6, 1);
         DMatrixRMaj justAngularMomentumState = new DMatrixRMaj(6, 1);
         angularErrorAtCurrentTick.get(justAngularErrorState);
         angularMomentumAtCurrentTick.get(3, justAngularMomentumState);

         DMatrixRMaj expectedRateVectorFromAngularError = new DMatrixRMaj(6, 1);
         DMatrixRMaj expectedRateVectorFromAngularMomentum = new DMatrixRMaj(6, 1);
         DMatrixRMaj expectedRateVector = new DMatrixRMaj(6, 1);
         expectedAngularErrorRate.get(expectedRateVector);
         expectedTorque.get(3, expectedRateVector);
         expectedAngularErrorRateFromAngularError.get(expectedRateVectorFromAngularError);
         expectedAngularErrorRateFromAngularMomentum.get(expectedRateVectorFromAngularMomentum);

         DMatrixRMaj rateVector = new DMatrixRMaj(6, 1);
         DMatrixRMaj rateVectorFromAngularError = new DMatrixRMaj(6, 1);
         DMatrixRMaj rateVectorFromAngularMomentum = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), compactStateVector, rateVector);
         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), justAngularErrorState, rateVectorFromAngularError);
         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), justAngularMomentumState, rateVectorFromAngularMomentum);
         CommonOps_DDRM.multAdd(inputCalculator.getContinuousBMatrix(), fullCoefficientVector, rateVector);
         CommonOps_DDRM.addEquals(rateVector, inputCalculator.getContinuousCMatrix());

         MatrixTestTools.assertMatrixEquals(expectedRateVectorFromAngularError, rateVectorFromAngularError, 1e-6);
         MatrixTestTools.assertMatrixEquals(expectedRateVectorFromAngularMomentum, rateVectorFromAngularMomentum, 1e-6);

         // check the B matrix for ending empty
         for (int col = indexHandler.getOrientationStartIndices(0); col < indexHandler.getTotalProblemSize(); col++)
         {
            for (int row = 0; row < 6; row++)
               assertEquals(0.0, inputCalculator.getContinuousBMatrix().get(row, col), 1e-6);
         }

         FrameVector3D expectedErrorRateFromBMatrix = new FrameVector3D();
         FrameVector3D expectedTorqueFromBMatrix = new FrameVector3D();
         FramePoint3D constructedPosition = new FramePoint3D(MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane));
         FrameVector3D constructedVelocity = new FrameVector3D(MPCTestHelper.computeCoMVelocity(time, omega, gravityZ, trajectoryCoefficients, contactPlane));
         constructedPosition.subZ(0.5 * time * time * gravityZ);
         constructedVelocity.subZ(time * gravityZ);

         expectedTorqueFromBMatrix.add(expectedTorqueFromContactPlane);
         expectedTorqueFromBMatrix.add(expectedTorqueFromGravity);

         expectedErrorRateFromBMatrix.cross(constructedVelocity, desiredCoMPosition);
         tempVector.cross(desiredCoMVelocity, constructedPosition);
         expectedErrorRateFromBMatrix.add(tempVector);
         desiredBodyOrientation.inverseTransform(expectedErrorRateFromBMatrix);
         momentOfInertia.inverseTransform(expectedErrorRateFromBMatrix);
         expectedErrorRateFromBMatrix.scale(mass);

         DMatrixRMaj expectedRateFromBMatrix = new DMatrixRMaj(6, 1);
         expectedErrorRateFromBMatrix.get(expectedRateFromBMatrix);
         expectedTorqueFromBMatrix.get(3, expectedRateFromBMatrix);

         DMatrixRMaj rateFromBMatrix = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), fullCoefficientVector, rateFromBMatrix);
         MatrixTestTools.assertMatrixEquals(expectedRateFromBMatrix, rateFromBMatrix, 1e-6);


         FrameVector3D expectedErrorRateFromCMatrix = new FrameVector3D();
         FrameVector3D expectedTorqueFromCMatrix = new FrameVector3D();

         expectedErrorRateFromCMatrix.cross(desiredCoMPosition, desiredCoMVelocity);
         expectedErrorRateFromCMatrix.scale(mass);
         expectedErrorRateFromCMatrix.sub(desiredBodyAngularMomentum);
         desiredBodyOrientation.inverseTransform(expectedErrorRateFromCMatrix);
         momentOfInertia.inverseTransform(expectedErrorRateFromCMatrix);

         DMatrixRMaj a0Expected = new DMatrixRMaj(3, 1);
         expectedErrorRateFromCMatrix.get(a0Expected);
         MatrixTestTools.assertMatrixEquals(a0Expected, inputCalculator.getA0(), 1e-6);

         FrameVector3D residualErrorRate = new FrameVector3D();
         FramePoint3D residualPosition = new FramePoint3D(gravityVector);
         FrameVector3D residualVelocity = new FrameVector3D(gravityVector);
         residualPosition.scale(0.5 * time * time);
         residualVelocity.scale(time);

         residualErrorRate.cross(residualVelocity, desiredCoMPosition);
         tempVector.cross(desiredCoMVelocity, residualPosition);
         residualErrorRate.add(tempVector);
         desiredBodyOrientation.inverseTransform(residualErrorRate);
         momentOfInertia.inverseTransform(residualErrorRate);
         residualErrorRate.scale(mass);

         expectedErrorRateFromCMatrix.add(residualErrorRate);

         expectedTorqueFromCMatrix.set(desiredInternalAngularMomentumRate);
         expectedTorqueFromCMatrix.scale(-1.0);

         DMatrixRMaj expectedRateFromCMatrix = new DMatrixRMaj(6, 1);
         expectedErrorRateFromCMatrix.get(expectedRateFromCMatrix);
         expectedTorqueFromCMatrix.get(3, expectedRateFromCMatrix);

         MatrixTestTools.assertMatrixEquals(expectedRateFromCMatrix, inputCalculator.getContinuousCMatrix(), 1e-6);

         FrameVector3D expectedErrorRateFromContact = new FrameVector3D();
         expectedErrorRateFromContact.add(expectedAngularErrorRateFromCoriolis);
         expectedErrorRateFromContact.add(expectedAngularErrorRateFromInternalMomentum);
         DMatrixRMaj expectedRateFromContact = new DMatrixRMaj(6, 1);
         expectedErrorRateFromContact.get(expectedRateFromContact);
         expectedTorque.get(3, expectedRateFromContact);

         DMatrixRMaj rateFromContact = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), fullCoefficientVector, rateFromContact);
         CommonOps_DDRM.addEquals(rateFromContact, inputCalculator.getContinuousCMatrix());

         DMatrixRMaj constructedRateFromContact = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.add(expectedRateFromBMatrix, expectedRateFromCMatrix, constructedRateFromContact);


         MatrixTestTools.assertMatrixEquals(constructedRateFromContact, rateFromContact, 1e-6);

         CommonOps_DDRM.addEquals(constructedRateFromContact, expectedRateVectorFromAngularError);
         CommonOps_DDRM.addEquals(constructedRateFromContact, expectedRateVectorFromAngularMomentum);
         MatrixTestTools.assertMatrixEquals(constructedRateFromContact, rateVector, 5e-1);
      }
   }

   @Test
   public void testJacobians()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mass = 10.0;
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;
      double mu = 0.8;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
      gravity.get(gravityVector);

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         Matrix3D skewDesiredCoMPosition = new Matrix3D();
         Matrix3D skewDesiredCoMVelocity = new Matrix3D();

         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, skewDesiredCoMPosition);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, skewDesiredCoMVelocity);

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);

         FrameVector3D a0 = new FrameVector3D();
         a0.cross(desiredCoMPosition, desiredCoMVelocity);
         a0.scale(mass);
         a0.sub(desiredBodyAngularMomentum);
         desiredBodyOrientation.inverseTransform(a0);
         momentOfInertia.inverseTransform(a0);

         Matrix3D a3 = new Matrix3D();
         desiredBodyOrientation.get(a3);
         a3.invert();
         momentOfInertia.inverseTransform(a3);

         Matrix3DBasics a1 = new Matrix3D();
         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, a1);
         a1.preMultiply(a3);
         a1.scale(mass);

         Matrix3DBasics a2 = new Matrix3D();
         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, a2);
         a2.preMultiply(a3);
         a2.scale(-mass);

         Matrix3D tempMatrix = new Matrix3D();
         MatrixMissingTools.toSkewSymmetricMatrix(desiredBodyAngularVelocity, tempMatrix);
         tempMatrix.scale(-1.0);
         Vector3D a4Vector = new Vector3D(desiredBodyAngularMomentum);
         desiredBodyOrientation.inverseTransform(a4Vector);
         Matrix3D a4 = new Matrix3D();
         MatrixMissingTools.toSkewSymmetricMatrix(a4Vector, a4);
         momentOfInertia.inverseTransform(a4);
         a4.add(tempMatrix);

         DMatrixRMaj a0Vector = new DMatrixRMaj(3, 1);
         a0.get(a0Vector);

         DMatrixRMaj a1Matrix = new DMatrixRMaj(3, 3);
         a1.get(a1Matrix);

         DMatrixRMaj a2Matrix = new DMatrixRMaj(3, 3);
         a2.get(a2Matrix);

         DMatrixRMaj a3Matrix = new DMatrixRMaj(3, 3);
         a3.get(a3Matrix);

         DMatrixRMaj a4Matrix = new DMatrixRMaj(3, 3);
         a4.get(a4Matrix);

         DMatrixRMaj Amatrix = new DMatrixRMaj(6, 6);
         DMatrixRMaj Bmatrix = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
         DMatrixRMaj Cmatrix = new DMatrixRMaj(6, 1);

         MatrixTools.setMatrixBlock(Amatrix, 0, 0, a4Matrix, 0, 0, 3, 3, 1.0);
         MatrixTools.setMatrixBlock(Amatrix, 0, 3, a3Matrix, 0, 0, 3, 3, 1.0);

         DMatrixRMaj skewGravityVector = new DMatrixRMaj(3, 3);
         MatrixMissingTools.toSkewSymmetricMatrix(gravity, skewGravityVector);

         DMatrixRMaj contactTorqueJacobian = MPCTestHelper.getContactTorqueJacobian(mass, time, omega, new FramePoint3D(), contactPlane);

         MatrixTools.multAddBlock(-mass, skewGravityVector, MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlane), Bmatrix, 3, 0);
         MatrixTools.addMatrixBlock(Bmatrix, 3, indexHandler.getRhoCoefficientStartIndex(0), contactTorqueJacobian, 0, 0, 3, indexHandler.getRhoCoefficientsInSegment(0), 1.0);

         MatrixTools.multAddBlock(a1Matrix, MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlane), Bmatrix, 0, 0);
         MatrixTools.multAddBlock(a2Matrix, MPCTestHelper.getCoMVelocityJacobian(time, omega, contactPlane), Bmatrix, 0, 0);

         desiredInternalAngularMomentumRate.get(3, Cmatrix);
         CommonOps_DDRM.scale(-1.0, Cmatrix);
         MatrixTools.setMatrixBlock(Cmatrix, 0, 0, a0Vector, 0, 0, 3, 1, 1.0);
         MatrixTools.multAddBlock(0.5 * time * time, a1Matrix, gravityVector, Cmatrix, 0, 0);
         MatrixTools.multAddBlock(time, a2Matrix, gravityVector, Cmatrix, 0, 0);

         DMatrixRMaj Admatrix = new DMatrixRMaj(6, 6);
         DMatrixRMaj Bdmatrix = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
         DMatrixRMaj Cdmatrix = new DMatrixRMaj(6, 1);

         DiscretizationCalculator discretizationCalculator = inputCalculator.getDiscretizationCalculator();
         discretizationCalculator.compute(Amatrix, Bmatrix, Cmatrix, Admatrix, Bdmatrix, Cdmatrix, tickDuration);

         DMatrixRMaj jacobian = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
         DMatrixRMaj objective = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
         MatrixTools.setMatrixBlock(jacobian, 0, 0, Bdmatrix, 0, 0, 6, indexHandler.getTotalProblemSize(), -1.0);
         MatrixTools.setMatrixBlock(jacobian, 0, indexHandler.getOrientationTickStartIndex(nextTickId - 1), Admatrix, 0, 0, 6, 6, -1.0);
         MatrixTools.setMatrixBlock(jacobian, 0, indexHandler.getOrientationTickStartIndex(nextTickId), CommonOps_DDRM.identity(6), 0, 0, 6, 6, 1.0);

         objective.set(Cdmatrix);

         MatrixTestTools.assertMatrixEquals(Amatrix, inputCalculator.getContinuousAMatrix(), 1e-6);
         MatrixTestTools.assertMatrixEquals(Bmatrix, inputCalculator.getContinuousBMatrix(), 1e-6);
         MatrixTestTools.assertMatrixEquals(Cmatrix, inputCalculator.getContinuousCMatrix(), 1e-6);

         MatrixTestTools.assertMatrixEquals(Admatrix, inputCalculator.getDiscreteAMatrix(), 1e-6);
         MatrixTestTools.assertMatrixEquals(Bdmatrix, inputCalculator.getDiscreteBMatrix(), 1e-6);
         MatrixTestTools.assertMatrixEquals(Cdmatrix, inputCalculator.getDiscreteCMatrix(), 1e-6);

         MatrixTestTools.assertMatrixEquals(jacobian, qpInput.getTaskJacobian(), 1e-6);
         MatrixTestTools.assertMatrixEquals(objective, qpInput.getTaskObjective(), 1e-6);
      }
   }

   @Test
   public void testA4Calculation()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mass = 10.0;
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;
      double mu = 0.8;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
      gravity.get(gravityVector);

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         Matrix3D skewDesiredCoMPosition = new Matrix3D();
         Matrix3D skewDesiredCoMVelocity = new Matrix3D();

         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, skewDesiredCoMPosition);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, skewDesiredCoMVelocity);

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);


         FrameVector3D axisAngleErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         DMatrixRMaj axisAngleError = new DMatrixRMaj(3, 1);
         axisAngleErrorAtCurrentTick.get(axisAngleError);

         Matrix3D tempMatrix = new Matrix3D();
         MatrixMissingTools.toSkewSymmetricMatrix(desiredBodyAngularVelocity, tempMatrix);
         tempMatrix.scale(-1.0);
         Vector3D a4Vector = new Vector3D(desiredBodyAngularMomentum);
         desiredBodyOrientation.inverseTransform(a4Vector);
         Matrix3D a4 = new Matrix3D();
         MatrixMissingTools.toSkewSymmetricMatrix(a4Vector, a4);
         momentOfInertia.inverseTransform(a4);
         a4.add(tempMatrix);


         DMatrixRMaj a4Matrix = new DMatrixRMaj(3, 3);
         a4.get(a4Matrix);

         FrameVector3D axisAngleRateAtCurrentTickFromCurrentError = new FrameVector3D();
         axisAngleRateAtCurrentTickFromCurrentError.cross(axisAngleErrorAtCurrentTick, desiredBodyAngularVelocity);

         Matrix3D desiredRotationMatrix = new Matrix3D();
         desiredBodyOrientation.get(desiredRotationMatrix);

         FrameVector3D tempVector = new FrameVector3D();
         desiredRotationMatrix.inverseTransform(desiredBodyAngularMomentum);
         tempVector.cross(desiredBodyAngularMomentum, axisAngleErrorAtCurrentTick);
         momentOfInertia.inverseTransform(tempVector);
         axisAngleRateAtCurrentTickFromCurrentError.add(tempVector);

         DMatrixRMaj fullState = new DMatrixRMaj(6, 1);
         DMatrixRMaj fullStateRate = new DMatrixRMaj(6, 1);
         axisAngleErrorAtCurrentTick.get(fullState);

         DMatrixRMaj expectedResultType1 = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedResultType2 = new DMatrixRMaj(3, 1);

         DMatrixRMaj fullExpectedResult = new DMatrixRMaj(6, 1);


         CommonOps_DDRM.mult(a4Matrix, axisAngleError, expectedResultType1);
         axisAngleRateAtCurrentTickFromCurrentError.get(expectedResultType2);
         axisAngleRateAtCurrentTickFromCurrentError.get(fullExpectedResult);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, expectedResultType2, 1e-6);

         DMatrixRMaj results = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(inputCalculator.getA4(), axisAngleError, results);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, results, 1e-6);

         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), fullState, fullStateRate);
         MatrixTestTools.assertMatrixEquals(fullExpectedResult, fullStateRate, 1e-6);

      }
   }

   @Test
   public void testA3Calculation()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mass = 10.0;
      double orientationPreviewWindowLength = 0.75;
      double mu = 0.8;
      double tickDuration = 0.1;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
      gravity.get(gravityVector);

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         Matrix3D skewDesiredCoMPosition = new Matrix3D();
         Matrix3D skewDesiredCoMVelocity = new Matrix3D();

         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, skewDesiredCoMPosition);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, skewDesiredCoMVelocity);

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);


         FrameVector3D angularMomentumOfBodyAboutOriginAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         DMatrixRMaj angularMomentum = new DMatrixRMaj(3, 1);
         angularMomentumOfBodyAboutOriginAtCurrentTick.get(angularMomentum);

         Matrix3D a3 = new Matrix3D();
         desiredBodyOrientation.get(a3);
         a3.invert();
         momentOfInertia.inverseTransform(a3);


         DMatrixRMaj a3Matrix = new DMatrixRMaj(3, 3);
         a3.get(a3Matrix);

         FrameVector3D axisAngleRateAtCurrentTickFromCurrentAngularMomentum = new FrameVector3D();

         Matrix3D desiredRotationMatrix = new Matrix3D();
         desiredBodyOrientation.get(desiredRotationMatrix);


         FrameVector3D tempVector = new FrameVector3D();
         desiredRotationMatrix.inverseTransform(angularMomentumOfBodyAboutOriginAtCurrentTick, tempVector);
         momentOfInertia.inverseTransform(tempVector);
         axisAngleRateAtCurrentTickFromCurrentAngularMomentum.set(tempVector);

         DMatrixRMaj expectedResultType1 = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedResultType2 = new DMatrixRMaj(3, 1);

         CommonOps_DDRM.mult(a3Matrix, angularMomentum, expectedResultType1);
         axisAngleRateAtCurrentTickFromCurrentAngularMomentum.get(expectedResultType2);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, expectedResultType2, 1e-6);

         DMatrixRMaj results = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(inputCalculator.getA3(), angularMomentum, results);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, results, 1e-6);

         DMatrixRMaj fullState = new DMatrixRMaj(6, 1);
         angularMomentumOfBodyAboutOriginAtCurrentTick.get(3, fullState);

         DMatrixRMaj fullStateRateExpected = new DMatrixRMaj(6, 1);
         axisAngleRateAtCurrentTickFromCurrentAngularMomentum.get(fullStateRateExpected);

         DMatrixRMaj fullStateRate = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), fullState, fullStateRate);

         MatrixTestTools.assertMatrixEquals(fullStateRateExpected, fullStateRate, 1e-6);
      }
   }

   @Test
   public void testA1Calculation()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mass = 10.0;
      double orientationPreviewWindowLength = 0.75;
      double mu = 0.8;
      double tickDuration = 0.1;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
      gravity.get(gravityVector);

      FramePose3D contactPose = new FramePose3D();
      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         FrameVector3DReadOnly comVelocity = MPCTestHelper.computeCoMVelocity(time, omega, gravityZ, trajectoryCoefficients, contactPlane);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj comVelocityVector = new DMatrixRMaj(3, 1);

         comPosition.get(comPositionVector);
         comVelocity.get(comVelocityVector);


         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);


         FrameVector3D angularMomentumOfBodyAboutOriginAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         DMatrixRMaj angularMomentum = new DMatrixRMaj(3, 1);
         angularMomentumOfBodyAboutOriginAtCurrentTick.get(angularMomentum);

         Matrix3D rotationMatrix = new Matrix3D();
         desiredBodyOrientation.get(rotationMatrix);

         FrameVector3D expectedResult = new FrameVector3D();
         expectedResult.cross(desiredCoMVelocity, comPosition);
         expectedResult.scale(mass);

         rotationMatrix.inverseTransform(expectedResult);
         momentOfInertia.inverseTransform(expectedResult);

         DMatrixRMaj expectedResultType1 = new DMatrixRMaj(3, 1);
         expectedResult.get(expectedResultType1);



         Matrix3D a1 = new Matrix3D();
         DMatrixRMaj a1Matrix = new DMatrixRMaj(3, 3);

         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, a1);

         Vector3D intermediateResult1 = new Vector3D();
         Vector3D intermediateResult2 = new Vector3D();
         intermediateResult1.cross(desiredCoMVelocity, comPosition);
         a1.transform(comPosition, intermediateResult2);

         EuclidCoreTestTools.assertTuple3DEquals(intermediateResult1, intermediateResult2, 1e-6);

         a1.preMultiplyTransposeOther(rotationMatrix);
         a1.preMultiplyInvertOther(momentOfInertia);
         a1.scale(mass);

         a1.get(a1Matrix);



         DMatrixRMaj expectedResultType3 = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(a1Matrix, comPositionVector, expectedResultType3);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, expectedResultType3, 1e-6);
         MatrixTestTools.assertMatrixEquals(a1Matrix, inputCalculator.getA1(), 1e-6);


         DMatrixRMaj resultingJacobian = new DMatrixRMaj(3, numberOfTrajectoryCoefficients);
         CommonOps_DDRM.mult(a1Matrix, MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlane), resultingJacobian);

         DMatrixRMaj expectedResultType2 = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(resultingJacobian, trajectoryCoefficients, expectedResultType2);
         CommonOps_DDRM.multAdd(0.5 * time * time, a1Matrix, gravityVector, expectedResultType2);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, expectedResultType2, 1e-6);
      }
   }

   @Test
   public void testA1AndA2Calculation()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mass = 10.0;
      double orientationPreviewWindowLength = 0.75;
      double mu = 0.8;
      double tickDuration = 0.1;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
      gravity.get(gravityVector);

      FramePose3D contactPose = new FramePose3D();
      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);


      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         DMatrixRMaj fullVector = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         MatrixTools.setMatrixBlock(fullVector, 0, 0, trajectoryCoefficients, 0, 0, numberOfTrajectoryCoefficients, 1, 1.0);

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         FrameVector3DReadOnly comVelocity = MPCTestHelper.computeCoMVelocity(time, omega, gravityZ, trajectoryCoefficients, contactPlane);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj comVelocityVector = new DMatrixRMaj(3, 1);

         comPosition.get(comPositionVector);
         comVelocity.get(comVelocityVector);


         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);


         FrameVector3D angularMomentumOfBodyAboutOriginAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         DMatrixRMaj angularMomentum = new DMatrixRMaj(3, 1);
         angularMomentumOfBodyAboutOriginAtCurrentTick.get(angularMomentum);

         Matrix3D rotationMatrix = new Matrix3D();
         desiredBodyOrientation.get(rotationMatrix);

         FrameVector3D expectedResult = new FrameVector3D();
         expectedResult.cross(desiredCoMPosition, comVelocity);

         FrameVector3D temp = new FrameVector3D();
         temp.cross(comPosition, desiredCoMVelocity);

         expectedResult.add(temp);

         expectedResult.scale(-mass);

         rotationMatrix.inverseTransform(expectedResult);
         momentOfInertia.inverseTransform(expectedResult);

         DMatrixRMaj expectedResultType1 = new DMatrixRMaj(3, 1);
         expectedResult.get(expectedResultType1);



         Matrix3D a2 = new Matrix3D();
         DMatrixRMaj a2Matrix = new DMatrixRMaj(3, 3);

         Matrix3D a1 = new Matrix3D();
         DMatrixRMaj a1Matrix = new DMatrixRMaj(3, 3);

         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, a1);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, a2);

         a1.preMultiplyTransposeOther(rotationMatrix);
         a1.preMultiplyInvertOther(momentOfInertia);
         a1.scale(mass);

         a1.get(a1Matrix);


         a2.preMultiplyTransposeOther(rotationMatrix);
         a2.preMultiplyInvertOther(momentOfInertia);
         a2.scale(-mass);

         a2.get(a2Matrix);


         DMatrixRMaj expectedResultType3 = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(a2Matrix, comVelocityVector, expectedResultType3);
         CommonOps_DDRM.multAdd(a1Matrix, comPositionVector, expectedResultType3);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, expectedResultType3, 1e-6);
         MatrixTestTools.assertMatrixEquals(a1Matrix, inputCalculator.getA1(), 1e-6);
         MatrixTestTools.assertMatrixEquals(a2Matrix, inputCalculator.getA2(), 1e-6);

         DMatrixRMaj resultingRate = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), fullVector, resultingRate);

         DMatrixRMaj expectedResultType4 = new DMatrixRMaj(3, 1);
         MatrixTools.setMatrixBlock(expectedResultType4, 0, 0, resultingRate, 0, 0, 3, 1, 1.0);


         DMatrixRMaj resultingJacobian = new DMatrixRMaj(3, numberOfTrajectoryCoefficients);
         CommonOps_DDRM.mult(a2Matrix, MPCTestHelper.getCoMVelocityJacobian(time, omega, contactPlane), resultingJacobian);
         CommonOps_DDRM.multAdd(a1Matrix, MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlane), resultingJacobian);

         DMatrixRMaj expectedResultType2 = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(resultingJacobian, trajectoryCoefficients, expectedResultType2);

         MatrixTestTools.assertMatrixEquals(expectedResultType2, expectedResultType4, 1e-6);


         CommonOps_DDRM.multAdd(0.5 * time * time, a1Matrix, gravityVector, expectedResultType2);
         CommonOps_DDRM.multAdd(time, a2Matrix, gravityVector, expectedResultType2);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, expectedResultType2, 1e-6);
      }
   }

   @Test
   public void testA2Calculation()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mass = 10.0;
      double orientationPreviewWindowLength = 0.75;
      double mu = 0.8;
      double tickDuration = 0.1;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
      gravity.get(gravityVector);

      FramePose3D contactPose = new FramePose3D();
      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         FrameVector3DReadOnly comVelocity = MPCTestHelper.computeCoMVelocity(time, omega, gravityZ, trajectoryCoefficients, contactPlane);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj comVelocityVector = new DMatrixRMaj(3, 1);

         comPosition.get(comPositionVector);
         comVelocity.get(comVelocityVector);


         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);


         FrameVector3D angularMomentumOfBodyAboutOriginAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         DMatrixRMaj angularMomentum = new DMatrixRMaj(3, 1);
         angularMomentumOfBodyAboutOriginAtCurrentTick.get(angularMomentum);

         Matrix3D rotationMatrix = new Matrix3D();
         desiredBodyOrientation.get(rotationMatrix);

         FrameVector3D expectedResult = new FrameVector3D();
         expectedResult.cross(desiredCoMPosition, comVelocity);
         expectedResult.scale(-mass);

         rotationMatrix.inverseTransform(expectedResult);
         momentOfInertia.inverseTransform(expectedResult);

         DMatrixRMaj expectedResultType1 = new DMatrixRMaj(3, 1);
         expectedResult.get(expectedResultType1);



         Matrix3D a2 = new Matrix3D();
         DMatrixRMaj a2Matrix = new DMatrixRMaj(3, 3);

         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, a2);

         a2.preMultiplyTransposeOther(rotationMatrix);
         a2.preMultiplyInvertOther(momentOfInertia);
         a2.scale(-mass);

         a2.get(a2Matrix);


         DMatrixRMaj expectedResultType3 = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(a2Matrix, comVelocityVector, expectedResultType3);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, expectedResultType3, 1e-6);
         MatrixTestTools.assertMatrixEquals(a2Matrix, inputCalculator.getA2(), 1e-6);


         DMatrixRMaj resultingJacobian = new DMatrixRMaj(3, numberOfTrajectoryCoefficients);
         CommonOps_DDRM.mult(a2Matrix, MPCTestHelper.getCoMVelocityJacobian(time, omega, contactPlane), resultingJacobian);

         DMatrixRMaj expectedResultType2 = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(resultingJacobian, trajectoryCoefficients, expectedResultType2);
         CommonOps_DDRM.multAdd(time, a2Matrix, gravityVector, expectedResultType2);

         MatrixTestTools.assertMatrixEquals(expectedResultType1, expectedResultType2, 1e-6);
      }
   }

   @Test
   public void testA0Calculation()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mass = 10.0;
      double mu = 0.8;
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
      gravity.get(gravityVector);

      FramePose3D contactPose = new FramePose3D();
      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         FrameVector3DReadOnly comVelocity = MPCTestHelper.computeCoMVelocity(time, omega, gravityZ, trajectoryCoefficients, contactPlane);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj comVelocityVector = new DMatrixRMaj(3, 1);

         comPosition.get(comPositionVector);
         comVelocity.get(comVelocityVector);


         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);


         FrameVector3D angularMomentumOfBodyAboutOriginAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         DMatrixRMaj angularMomentum = new DMatrixRMaj(3, 1);
         angularMomentumOfBodyAboutOriginAtCurrentTick.get(angularMomentum);

         Matrix3D rotationMatrix = new Matrix3D();
         desiredBodyOrientation.get(rotationMatrix);

         FrameVector3D expectedResult = new FrameVector3D();
         expectedResult.cross(desiredCoMPosition, comVelocity);
         expectedResult.scale(-mass);

         rotationMatrix.inverseTransform(expectedResult);
         momentOfInertia.inverseTransform(expectedResult);

         DMatrixRMaj expectedResultType1 = new DMatrixRMaj(3, 1);
         expectedResult.get(expectedResultType1);



         Vector3D a0 = new Vector3D();
         DMatrixRMaj a0Matrix = new DMatrixRMaj(3, 1);

         a0.cross(desiredCoMPosition, desiredCoMVelocity);
         a0.scale(mass);
         a0.sub(desiredBodyAngularMomentum);

         rotationMatrix.inverseTransform(a0);
         momentOfInertia.inverseTransform(a0);

         a0.get(a0Matrix);

         MatrixTestTools.assertMatrixEquals(a0Matrix, inputCalculator.getA0(), 1e-6);
      }
   }

   @Test
   public void testA0A1AndA2Calculation()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mass = 10.0;
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;
      double mu = 0.8;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
      gravity.get(gravityVector);

      FramePose3D contactPose = new FramePose3D();
      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         DMatrixRMaj fullVector = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         MatrixTools.setMatrixBlock(fullVector, 0, 0, trajectoryCoefficients, 0, 0, numberOfTrajectoryCoefficients, 1, 1.0);

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         FrameVector3DReadOnly comVelocity = MPCTestHelper.computeCoMVelocity(time, omega, gravityZ, trajectoryCoefficients, contactPlane);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj comVelocityVector = new DMatrixRMaj(3, 1);

         comPosition.get(comPositionVector);
         comVelocity.get(comVelocityVector);


         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);


         FrameVector3D angularMomentumOfBodyAboutOriginAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         DMatrixRMaj angularMomentum = new DMatrixRMaj(3, 1);
         angularMomentumOfBodyAboutOriginAtCurrentTick.get(angularMomentum);

         Matrix3D rotationMatrix = new Matrix3D();
         desiredBodyOrientation.get(rotationMatrix);

         FrameVector3D expectedResult = new FrameVector3D();
         expectedResult.cross(desiredCoMPosition, comVelocity);

         FrameVector3D temp = new FrameVector3D();
         temp.cross(comPosition, desiredCoMVelocity);

         expectedResult.add(temp);

         expectedResult.scale(-mass);

         rotationMatrix.inverseTransform(expectedResult);
         momentOfInertia.inverseTransform(expectedResult);

         DMatrixRMaj expectedResultType1 = new DMatrixRMaj(3, 1);
         expectedResult.get(expectedResultType1);



         Matrix3D a2 = new Matrix3D();
         DMatrixRMaj a2Matrix = new DMatrixRMaj(3, 3);

         Matrix3D a1 = new Matrix3D();
         DMatrixRMaj a1Matrix = new DMatrixRMaj(3, 3);

         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, a1);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, a2);

         a1.preMultiplyTransposeOther(rotationMatrix);
         a1.preMultiplyInvertOther(momentOfInertia);
         a1.scale(mass);

         a1.get(a1Matrix);

         a2.preMultiplyTransposeOther(rotationMatrix);
         a2.preMultiplyInvertOther(momentOfInertia);
         a2.scale(-mass);

         a2.get(a2Matrix);

         Vector3D a0 = new Vector3D();
         DMatrixRMaj a0Matrix = new DMatrixRMaj(3, 1);

         a0.cross(desiredCoMPosition, desiredCoMVelocity);
         a0.scale(mass);
         a0.sub(desiredBodyAngularMomentum);

         rotationMatrix.inverseTransform(a0);
         momentOfInertia.inverseTransform(a0);

         a0.get(a0Matrix);

         DMatrixRMaj topOfConstantExpected = new DMatrixRMaj(3, 1);
         topOfConstantExpected.set(a0Matrix);
         CommonOps_DDRM.multAdd(0.5 * time * time, a1Matrix, gravityVector, topOfConstantExpected);
         CommonOps_DDRM.multAdd(time, a2Matrix, gravityVector, topOfConstantExpected);

         DMatrixRMaj topOfConstant = new DMatrixRMaj(3, 1);
         MatrixTools.setMatrixBlock(topOfConstant, 0, 0, inputCalculator.getContinuousCMatrix(), 0, 0, 3, 1, 1.0);

         MatrixTestTools.assertMatrixEquals(topOfConstantExpected, topOfConstant, 1e-6);
      }
   }

   @Test
   public void testRemainingMatrices()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mass = 10.0;
      double mu = 0.8;
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
      gravity.get(gravityVector);

      FramePose3D contactPose = new FramePose3D();
      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         DMatrixRMaj fullVector = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         MatrixTools.setMatrixBlock(fullVector, 0, 0, trajectoryCoefficients, 0, 0, numberOfTrajectoryCoefficients, 1, 1.0);

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         FrameVector3DReadOnly comVelocity = MPCTestHelper.computeCoMVelocity(time, omega, gravityZ, trajectoryCoefficients, contactPlane);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj comVelocityVector = new DMatrixRMaj(3, 1);

         comPosition.get(comPositionVector);
         comVelocity.get(comVelocityVector);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         Matrix3D skewDesiredCoMPosition = new Matrix3D();
         Matrix3D skewDesiredCoMVelocity = new Matrix3D();

         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, skewDesiredCoMPosition);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, skewDesiredCoMVelocity);

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(desiredNetAngularMomentum);
         command.setDesiredInternalAngularMomentum(desiredInternalAngularMomentum);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);


         FrameVector3D expectedMomentumRate = new FrameVector3D();
         expectedMomentumRate.cross(comPosition, gravity);
         expectedMomentumRate.scale(mass);

         FrameVector3D summedTorque = new FrameVector3D();
         contactPlane.computeContactForce(omega, time);
         for (int contactPointIndex = 0; contactPointIndex < contactPlane.getNumberOfContactPoints(); contactPointIndex++)
         {
            FramePoint3D contactLocation = new FramePoint3D(contactPlane.getContactPointHelper(contactPointIndex).getBasisVectorOrigin());
            contactLocation.changeFrame(ReferenceFrame.getWorldFrame());

            FrameVector3D pointTorque = new FrameVector3D();
            pointTorque.cross(contactLocation, contactPlane.getContactPointHelper(contactPointIndex).getContactAcceleration());
            pointTorque.scale(mass);

            summedTorque.add(pointTorque);
         }

         expectedMomentumRate.add(summedTorque);

         DMatrixRMaj lowerC = new DMatrixRMaj(3, 1);
         MatrixTools.setMatrixBlock(lowerC, 0, 0, inputCalculator.getContinuousCMatrix(), 3, 0, 3, 1, 1.0);

         DMatrixRMaj lowerCExpected = new DMatrixRMaj(3, 1);
         desiredInternalAngularMomentumRate.get(lowerCExpected);
         CommonOps_DDRM.scale(-1.0, lowerCExpected);

         MatrixTestTools.assertMatrixEquals(lowerCExpected, lowerC, 1e-6);

         DMatrixRMaj expectedLowerBProduct = new DMatrixRMaj(3, 1);
         expectedMomentumRate.get(expectedLowerBProduct);

         DMatrixRMaj bProduct = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), fullVector, bProduct);
         DMatrixRMaj lowerBProduct = new DMatrixRMaj(3, 1);
         MatrixTools.setMatrixBlock(lowerBProduct, 0, 0, bProduct, 3, 0, 3, 1, 1.0);

         MatrixTestTools.assertMatrixEquals(expectedLowerBProduct, lowerBProduct, 1e-6);
      }
   }
}
