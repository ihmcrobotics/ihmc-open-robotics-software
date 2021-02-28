package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteAngularVelocityOrientationCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteMomentumOrientationCommand;
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

public class AngularVelocityOrientationInputCalculatorTest
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
      AngularVelocityOrientationInputCalculator inputCalculator = new AngularVelocityOrientationInputCalculator(indexHandler, mass, gravityZ);

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

         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
         desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         DiscreteAngularVelocityOrientationCommand command = new DiscreteAngularVelocityOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMAcceleration(desiredCoMAcceleration);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentumRate(desiredNetAngularMomentumRate);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);
//
         inputCalculator.compute(qpInput, command);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         FrameVector3D angularVelocityErrorRate = new FrameVector3D();
         FrameVector3D angularVelocityErrorRateFromAngularError = new FrameVector3D();
         FrameVector3D angularVelocityErrorRateFromAngularVelocityError = new FrameVector3D();
         FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();

         FrameVector3D tempVector = new FrameVector3D(desiredBodyAngularMomentumRate);
         FrameVector3D tempVector2 = new FrameVector3D(desiredBodyAngularMomentumRate);
         desiredBodyOrientation.inverseTransform(tempVector);
         angularVelocityErrorRateFromAngularError.cross(tempVector, angularErrorAtCurrentTick);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromAngularError);

         Matrix3D tempMatrix = new Matrix3D();
         tempVector.set(desiredBodyAngularVelocity);
         momentOfInertia.inverseTransform(tempVector);
         MatrixMissingTools.toSkewSymmetricMatrix(tempVector, tempMatrix);

         tempMatrix.transform(angularVelocityErrorAtCurrentTick, angularVelocityErrorRateFromAngularVelocityError);
         momentOfInertia.transform(angularVelocityErrorAtCurrentTick, tempVector);
         tempVector2.cross(desiredBodyAngularVelocity, tempVector);
         angularVelocityErrorRateFromAngularVelocityError.sub(tempVector2);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromAngularVelocityError);

         FrameVector3D desiredContactPointForce = new FrameVector3D(desiredCoMAcceleration);
         desiredContactPointForce.scale(mass / contactPlane.getNumberOfContactPoints());

         angularVelocityErrorRateFromContact.cross(desiredCoMAcceleration, comPosition);
         angularVelocityErrorRateFromContact.scale(-mass);

         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);
            FrameVector3D relativeContactPointLocation = new FrameVector3D();
            relativeContactPointLocation.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);

            FrameVector3D torqueFromContact = new FrameVector3D();
            torqueFromContact.cross(relativeContactPointLocation, contactPoint.getContactAcceleration());
            torqueFromContact.scale(mass);

            FrameVector3D coriolisForce = new FrameVector3D();
            coriolisForce.cross(desiredContactPointForce, contactPoint.getBasisVectorOrigin());
            torqueFromContact.add(coriolisForce);

            angularVelocityErrorRateFromContact.add(torqueFromContact);
         }

         desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

         angularVelocityErrorRate.set(angularVelocityErrorRateFromAngularError);
         angularVelocityErrorRate.add(angularVelocityErrorRateFromAngularVelocityError);
         angularVelocityErrorRate.add(angularVelocityErrorRateFromContact);

         FrameVector3D angularErrorRate = new FrameVector3D();
         FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();
         FrameVector3D angularErrorRateFromAngularVelocityError = new FrameVector3D();

         angularErrorRateFromAngularError.cross(desiredBodyAngularVelocity, angularErrorAtCurrentTick);
         angularErrorRateFromAngularError.scale(-1.0);

         angularErrorRateFromAngularVelocityError.set(angularVelocityErrorAtCurrentTick);

         angularErrorRate.add(angularErrorRateFromAngularError, angularErrorRateFromAngularVelocityError);

         DMatrixRMaj expectedRateVector = new DMatrixRMaj(6, 1);
         angularErrorRate.get(expectedRateVector);
         angularVelocityErrorRate.get(3, expectedRateVector);

         DMatrixRMaj stateVector = new DMatrixRMaj(6, 1);
         DMatrixRMaj angleErrorStateVector = new DMatrixRMaj(6, 1);
         DMatrixRMaj velocityErrorStateVector = new DMatrixRMaj(6, 1);

         DMatrixRMaj rateVector = new DMatrixRMaj(6, 1);
         DMatrixRMaj rateFromAngleError = new DMatrixRMaj(6, 1);
         DMatrixRMaj rateFromVelocityError = new DMatrixRMaj(6, 1);
         DMatrixRMaj rateFromContact = new DMatrixRMaj(6, 1);

         DMatrixRMaj angleErrorRateFromAngleError = new DMatrixRMaj(3, 1);
         DMatrixRMaj velocityErrorRateFromAngleError = new DMatrixRMaj(3, 1);
         DMatrixRMaj angleErrorRateFromVelocityError = new DMatrixRMaj(3, 1);
         DMatrixRMaj velocityErrorRateFromVelocityError = new DMatrixRMaj(3, 1);
         DMatrixRMaj velocityErrorRateFromContact = new DMatrixRMaj(3, 1);

         angularErrorAtCurrentTick.get(stateVector);
         angularErrorAtCurrentTick.get(angleErrorStateVector);
         angularVelocityErrorAtCurrentTick.get(3, stateVector);
         angularVelocityErrorAtCurrentTick.get(3, velocityErrorStateVector);

         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), stateVector, rateVector);
         CommonOps_DDRM.multAdd(inputCalculator.getContinuousBMatrix(), fullCoefficientVector, rateVector);
         CommonOps_DDRM.addEquals(rateVector, inputCalculator.getContinuousCMatrix());

         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), angleErrorStateVector, rateFromAngleError);
         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), velocityErrorStateVector, rateFromVelocityError);

         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), fullCoefficientVector, rateFromContact);
         CommonOps_DDRM.addEquals(rateFromContact, inputCalculator.getContinuousCMatrix());

         MatrixTools.setMatrixBlock(angleErrorRateFromAngleError, 0, 0, rateFromAngleError, 0, 0, 3, 1, 1.0);
         MatrixTools.setMatrixBlock(velocityErrorRateFromAngleError, 0, 0, rateFromAngleError, 3, 0, 3, 1, 1.0);

         MatrixTools.setMatrixBlock(angleErrorRateFromVelocityError, 0, 0, rateFromVelocityError, 0, 0, 3, 1, 1.0);
         MatrixTools.setMatrixBlock(velocityErrorRateFromVelocityError, 0, 0, rateFromVelocityError, 3, 0, 3, 1, 1.0);

         MatrixTools.setMatrixBlock(velocityErrorRateFromContact, 0, 0, rateFromContact, 3, 0, 3, 1, 1.0);

         DMatrixRMaj expectedAngleErrorRateFromAngleError = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedVelocityErrorRateFromAngleError = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedAngleErrorRateFromVelocityError = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedVelocityErrorRateFromVelocityError = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedVelocityErrorRateFromContact = new DMatrixRMaj(3, 1);

         angularErrorRateFromAngularError.get(expectedAngleErrorRateFromAngleError);
         angularErrorRateFromAngularVelocityError.get(expectedAngleErrorRateFromVelocityError);
         angularVelocityErrorRateFromAngularError.get(expectedVelocityErrorRateFromAngleError);
         angularVelocityErrorRateFromAngularVelocityError.get(expectedVelocityErrorRateFromVelocityError);
         angularVelocityErrorRateFromContact.get(expectedVelocityErrorRateFromContact);

         MatrixTestTools.assertMatrixEquals(expectedAngleErrorRateFromAngleError, angleErrorRateFromAngleError, 1e-6);
         MatrixTestTools.assertMatrixEquals(expectedVelocityErrorRateFromAngleError, velocityErrorRateFromAngleError, 1e-6);
         MatrixTestTools.assertMatrixEquals(expectedAngleErrorRateFromVelocityError, angleErrorRateFromVelocityError, 1e-6);
         MatrixTestTools.assertMatrixEquals(expectedVelocityErrorRateFromVelocityError, velocityErrorRateFromVelocityError, 1e-6);

         MatrixTestTools.assertMatrixEquals(expectedVelocityErrorRateFromContact, velocityErrorRateFromContact, 1e-6);

         MatrixTestTools.assertMatrixEquals(expectedRateVector, rateVector, 1e-6);
      }
   }

   @Test
   public void testB2()
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
      AngularVelocityOrientationInputCalculator inputCalculator = new AngularVelocityOrientationInputCalculator(indexHandler, mass, gravityZ);

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

         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
         desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMAcceleration = new FrameVector3D();

         DiscreteAngularVelocityOrientationCommand command = new DiscreteAngularVelocityOrientationCommand();
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredCoMAcceleration(desiredCoMAcceleration);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentumRate(desiredNetAngularMomentumRate);
         command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
         command.addContactPlaneHelper(contactPlane);

         inputCalculator.compute(qpInput, command);

         FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();
         DMatrixRMaj contactForces = new DMatrixRMaj(12, 1);

         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);
            FrameVector3D relativeContactPointLocation = new FrameVector3D();
            relativeContactPointLocation.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);

            FrameVector3D torqueFromContact = new FrameVector3D();
            torqueFromContact.cross(relativeContactPointLocation, contactPoint.getContactAcceleration());
            torqueFromContact.scale(mass);

            angularVelocityErrorRateFromContact.add(torqueFromContact);

            contactPoint.getContactAcceleration().get(3 * contactPointIdx, contactForces);
         }
         CommonOps_DDRM.scale(mass, contactForces);

         desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

         DMatrixRMaj rate = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), fullCoefficientVector, rate);

         DMatrixRMaj expectedAngularRateErrorRateFromContact = new DMatrixRMaj(3, 1);
         angularVelocityErrorRateFromContact.get(expectedAngularRateErrorRateFromContact);

         DMatrixRMaj angularRateErrorRateFromContact = new DMatrixRMaj(3, 1);
         MatrixTools.setMatrixBlock(angularRateErrorRateFromContact, 0, 0, rate, 3, 0, 3, 1, 1.0);


         DMatrixRMaj easyExpectedAngularAcceleration = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(inputCalculator.getB2(), contactForces, easyExpectedAngularAcceleration);
         MatrixTestTools.assertMatrixEquals(easyExpectedAngularAcceleration, angularRateErrorRateFromContact, 1e-6);

         MatrixTestTools.assertMatrixEquals(expectedAngularRateErrorRateFromContact, angularRateErrorRateFromContact, 1e-6);
      }
   }


}
