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
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class AngularVelocityOrientationInputCalculatorTest
{
   private static final double gravityZ = -9.81;
   private static final double omega = 3.0;
   private static final double mu = 0.8;
   private static final double mass = 10.0;

   private static final double Ixx = 1.0;
   private static final double Iyy = 1.0;
   private static final double Izz = 1.0;

   private static final Matrix3D momentOfInertia = new Matrix3D();
   static
   {
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);
   }

   @Test
   public void testCoMObjectiveOneSegment()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

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
         desiredContactPointForce.sub(gravityVector);

         angularVelocityErrorRateFromContact.cross(desiredContactPointForce, comPosition);
         angularVelocityErrorRateFromContact.scale(-mass);

         desiredContactPointForce.scale(mass / contactPlane.getNumberOfContactPoints());

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
         CommonOps_DDRM.multAdd(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateVector);
         CommonOps_DDRM.addEquals(rateVector, inputCalculator.getContinuousCMatrix());

         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), angleErrorStateVector, rateFromAngleError);
         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), velocityErrorStateVector, rateFromVelocityError);

         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateFromContact);
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

         MatrixTestTools.assertMatrixEquals(expectedVelocityErrorRateFromContact, velocityErrorRateFromContact, 1e-5);

         MatrixTestTools.assertMatrixEquals(expectedRateVector, rateVector, 1e-5);
      }
   }

   @Test
   public void testCoMObjectiveOneSegmentInFlight()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      AngularVelocityOrientationInputCalculator inputCalculator = new AngularVelocityOrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoCoefficients = 0;
      int comCoefficients = 6;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, null);

         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
         desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

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

         FrameVector3D desiredContactForce = new FrameVector3D(desiredCoMAcceleration);
         desiredContactForce.subZ(gravityZ);
         angularVelocityErrorRateFromContact.cross(desiredContactForce, comPosition);
         angularVelocityErrorRateFromContact.scale(-mass);

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
         CommonOps_DDRM.multAdd(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateVector);
         CommonOps_DDRM.addEquals(rateVector, inputCalculator.getContinuousCMatrix());

         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), angleErrorStateVector, rateFromAngleError);
         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), velocityErrorStateVector, rateFromVelocityError);

         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateFromContact);
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

         MatrixTestTools.assertMatrixEquals(expectedVelocityErrorRateFromContact, velocityErrorRateFromContact, 1e-5);

         MatrixTestTools.assertMatrixEquals(expectedRateVector, rateVector, 1e-5);
      }
   }

   @Test
   public void testCoMObjectiveOneSegmentAgainstExactSolution()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

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

         AxisAngle axisAngle = new AxisAngle();
         axisAngle.setRotationVector(angularErrorAtCurrentTick);

         FrameQuaternion currentOrientation = new FrameQuaternion(desiredBodyOrientation);
         currentOrientation.append(axisAngle);

         FrameVector3D currentBodyAngularVelocity = new FrameVector3D();
         currentBodyAngularVelocity.add(desiredBodyAngularVelocity, angularVelocityErrorAtCurrentTick);

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
         desiredContactPointForce.addZ(Math.abs(gravityZ));
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
         DMatrixRMaj rateErrorVector = new DMatrixRMaj(6, 1);

         angularErrorAtCurrentTick.get(stateVector);
         angularVelocityErrorAtCurrentTick.get(3, stateVector);

         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), stateVector, rateErrorVector);
         CommonOps_DDRM.multAdd(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateErrorVector);
         CommonOps_DDRM.addEquals(rateErrorVector, inputCalculator.getContinuousCMatrix());

         MatrixTestTools.assertMatrixEquals(expectedRateVector, rateErrorVector, 1e-5);

         Matrix3D currentRotation = new Matrix3D();
         Matrix3D desiredRotation = new Matrix3D();

         currentOrientation.get(currentRotation);
         desiredBodyOrientation.get(desiredRotation);

         Matrix3D skewCurrentAngularVelocity = new Matrix3D();
         Matrix3D skewDesiredAngularVelocity = new Matrix3D();

         MatrixMissingTools.toSkewSymmetricMatrix(currentBodyAngularVelocity, skewCurrentAngularVelocity);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredBodyAngularVelocity, skewDesiredAngularVelocity);

         Matrix3D currentRotationRate = new Matrix3D();
         Matrix3D desiredRotationRate = new Matrix3D();

         currentRotationRate.set(currentRotation);
         currentRotationRate.multiply(skewCurrentAngularVelocity);

         desiredRotationRate.set(desiredRotation);
         desiredRotationRate.multiply(skewDesiredAngularVelocity);

         Vector3D currentRotationRateVector = new Vector3D();
         Vector3D desiredRotationRateVector = new Vector3D();
         MatrixMissingTools.fromSkewSymmetricMatrix(currentRotationRate, currentRotationRateVector);
         MatrixMissingTools.fromSkewSymmetricMatrix(desiredRotationRate, desiredRotationRateVector);

         Vector3D rotationRateVectorError = new Vector3D();
         rotationRateVectorError.sub(currentRotationRateVector, desiredRotationRateVector);

         EuclidCoreTestTools.assertVector3DGeometricallyEquals(rotationRateVectorError, angularErrorRate, 1e-5);
      }
   }

   @Test
   public void testB1()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      Random random = new Random(1738L);
      FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
      desiredCoMPosition.setZ(0.0);
      FrameVector3D desiredCoMAcceleration = new FrameVector3D(desiredCoMPosition);
      desiredCoMAcceleration.setZ(gravityZ);

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2D contactPolygon = new ConvexPolygon2D();
      contactPolygon.addVertex(desiredCoMPosition);
      contactPolygon.addVertex(desiredCoMPosition);
      contactPolygon.addVertex(desiredCoMPosition);
      contactPolygon.addVertex(desiredCoMPosition);
      contactPolygon.update();

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

      int rhoCoefficients = indexHandler.getRhoCoefficientsInSegment(0);
      int comCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);


      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

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

         FrameVector3D desiredContactForce = new FrameVector3D(desiredCoMAcceleration);
         desiredContactForce.subZ(gravityZ);

         FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();
         angularVelocityErrorRateFromContact.cross(desiredContactForce, comPosition);
         desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);
         angularVelocityErrorRateFromContact.scale(-mass);


         DMatrixRMaj rateVector = new DMatrixRMaj(6, 1);
         DMatrixRMaj rateFromContact = new DMatrixRMaj(6, 1);

         DMatrixRMaj velocityErrorRateFromContact = new DMatrixRMaj(3, 1);


         CommonOps_DDRM.multAdd(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateVector);
         CommonOps_DDRM.addEquals(rateVector, inputCalculator.getContinuousCMatrix());

         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateFromContact);
         CommonOps_DDRM.addEquals(rateFromContact, inputCalculator.getContinuousCMatrix());

         DMatrixRMaj scaledGravity = new DMatrixRMaj(3, 1);
         scaledGravity.set(2, 0, 0.5 * time * time * gravityZ);

         DMatrixRMaj expectedBMatrix = new DMatrixRMaj(6, numberOfTrajectoryCoefficients);
         DMatrixRMaj expectedCMatrix = new DMatrixRMaj(6, 1);
         MatrixTools.multAddBlock(inputCalculator.getB1(), MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlane), expectedBMatrix, 3, 0);
         MatrixTools.multAddBlock(inputCalculator.getB1(), scaledGravity, expectedCMatrix, 3, 0);

         MatrixTools.setMatrixBlock(velocityErrorRateFromContact, 0, 0, rateFromContact, 3, 0, 3, 1, 1.0);

         DMatrixRMaj expectedVelocityErrorRateFromContact = new DMatrixRMaj(3, 1);

         angularVelocityErrorRateFromContact.get(expectedVelocityErrorRateFromContact);

         MatrixTestTools.assertMatrixEquals(new DMatrixRMaj(3, 1), inputCalculator.getB0(), 1e-5);
         MatrixTestTools.assertMatrixEquals(new DMatrixRMaj(3, 3 * contactPolygon.getNumberOfVertices()), inputCalculator.getB2(), 1e-5);

         MatrixTestTools.assertMatrixEquals(expectedBMatrix, inputCalculator.getContinuousBMatrix(), 1e-5);
         MatrixTestTools.assertMatrixEquals(expectedCMatrix, inputCalculator.getContinuousCMatrix(), 1e-5);
         MatrixTestTools.assertMatrixEquals(expectedVelocityErrorRateFromContact, velocityErrorRateFromContact, 1e-5);
      }
   }

   @Test
   public void testB2()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

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
         FrameVector3D desiredCoMAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

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
         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rate);

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

   @Test
   public void testConstraintFormulation()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

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

      int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
      DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
      trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

      FrameVector3D initialAngularError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
      FrameVector3D initialAngularVelocityError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

      FrameVector3D angularErrorAtCurrentTick = new FrameVector3D(initialAngularError);
      FrameVector3D angularVelocityErrorAtCurrentTick = new FrameVector3D(initialAngularVelocityError);

      DMatrixRMaj orientationVariables = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);

      DMatrixRMaj Aeq = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), indexHandler.getTotalProblemSize());
      DMatrixRMaj beq = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);

      double time = 0.0;
      for (int tick = 0; tick < indexHandler.getTotalNumberOfOrientationTicks(); tick++)
      {
         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));
         contactPlane.computeContactForce(omega, time);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
         desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);

         DiscreteAngularVelocityOrientationCommand command = getCommand(time,
                                                                        tick,
                                                                        tickDuration,
                                                                        omega,
                                                                        momentOfInertia,
                                                                        desiredCoMPosition,
                                                                        desiredCoMAcceleration,
                                                                        desiredBodyOrientation,
                                                                        desiredBodyAngularVelocity,
                                                                        desiredNetAngularMomentumRate,
                                                                        desiredInternalAngularMomentumRate,
                                                                        angularErrorAtCurrentTick,
                                                                        angularVelocityErrorAtCurrentTick,
                                                                        contactPlane);

         inputCalculator.compute(qpInput, command);

         MatrixTools.setMatrixBlock(Aeq, 6 * tick, 0, qpInput.getTaskJacobian(), 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
         MatrixTools.setMatrixBlock(beq, 6 * tick, 0, qpInput.getTaskObjective(), 0, 0, 6, 1, 1.0);

         FrameVector3DReadOnly expectedAngularErrorRate = computeAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
         FrameVector3DReadOnly expectedAngularVelocityErrorRate = computeAngularVelocityErrorRate(mass,
                                                                                          comPosition,
                                                                                          angularErrorAtCurrentTick,
                                                                                          angularVelocityErrorAtCurrentTick,
                                                                                          command);

         DMatrixRMaj stateVector = new DMatrixRMaj(6, 1);
         angularErrorAtCurrentTick.get(stateVector);
         angularVelocityErrorAtCurrentTick.get(3, stateVector);

         DMatrixRMaj expectedRateVector = new DMatrixRMaj(6, 1);
         expectedAngularErrorRate.get(expectedRateVector);
         expectedAngularVelocityErrorRate.get(3, expectedRateVector);

         DMatrixRMaj rateVector = new DMatrixRMaj(6, 1);

         CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), stateVector, rateVector);
         CommonOps_DDRM.multAdd(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateVector);
         CommonOps_DDRM.addEquals(rateVector, inputCalculator.getContinuousCMatrix());

         MatrixTestTools.assertMatrixEquals("Failed on tick " + tick, expectedRateVector, rateVector, 1e-5);

         FrameVector3D angularErrorRate = new FrameVector3D();
         FrameVector3D angularVelocityErrorRate = new FrameVector3D();
         angularErrorRate.set(rateVector);
         angularVelocityErrorRate.set(3, rateVector);

         angularErrorAtCurrentTick.scaleAdd(tickDuration, angularErrorRate, angularErrorAtCurrentTick);
         angularVelocityErrorAtCurrentTick.scaleAdd(tickDuration, angularVelocityErrorRate, angularVelocityErrorAtCurrentTick);

         angularErrorAtCurrentTick.get(6 * tick, orientationVariables);
         angularVelocityErrorAtCurrentTick.get(6 * tick + 3, orientationVariables);

         time += tickDuration;
      }

      DMatrixRMaj fullVariableMatrix = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
      MatrixTools.setMatrixBlock(fullVariableMatrix, 0, 0, trajectoryCoefficients, 0, 0, numberOfTrajectoryCoefficients, 1, 1.0);
      MatrixTools.setMatrixBlock(fullVariableMatrix, numberOfTrajectoryCoefficients, 0, orientationVariables, 0, 0, 6 * indexHandler.getTotalNumberOfOrientationTicks(), 1, 1.0);

      DMatrixRMaj value = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);
      CommonOps_DDRM.mult(Aeq, fullVariableMatrix, value);

      MatrixTestTools.assertMatrixEquals(value, beq, 1e-5);
   }

   private static DiscreteAngularVelocityOrientationCommand getCommand(double time,
                                                                       int nextTickId,
                                                                       double tickDuration,
                                                                       double omega,
                                                                       Matrix3DReadOnly momentOfInertia,
                                                                       FramePoint3DReadOnly desiredCoMPosition,
                                                                       FrameVector3DReadOnly desiredCoMAcceleration,
                                                                       FrameOrientation3DReadOnly desiredBodyOrientation,
                                                                       FrameVector3DReadOnly desiredBodyAngularVelocity,
                                                                       FrameVector3DReadOnly desiredNetAngularMomentumRate,
                                                                       FrameVector3DReadOnly desiredInternalAngularMomentumRate,
                                                                       FrameVector3DReadOnly currentAxisAngleError,
                                                                       FrameVector3DReadOnly currentAngularVelocityError,
                                                                       MPCContactPlane contactPlane)
   {

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

      if (nextTickId == 0)
      {
         command.setCurrentAxisAngleError(currentAxisAngleError);
         command.setCurrentBodyAngularVelocityErrorInBodyFrame(currentAngularVelocityError);
      }

      return command;
   }

   private static FrameVector3DReadOnly computeAngularVelocityErrorRate(double mass,
                                                                        FramePoint3DReadOnly comPosition,
                                                                        FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                        FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                        DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularVelocityErrorRate = new FrameVector3D();
      FrameVector3D angularVelocityErrorRateFromAngularError = new FrameVector3D();
      FrameVector3D angularVelocityErrorRateFromAngularVelocityError = new FrameVector3D();
      FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();

      FrameVector3D desiredBodyAngularMomentumRate = new FrameVector3D();
      desiredBodyAngularMomentumRate.sub(command.getDesiredNetAngularMomentumRate(), command.getDesiredInternalAngularMomentumRate());

      Matrix3DReadOnly momentOfInertia = command.getMomentOfInertiaInBodyFrame();
      FramePoint3DReadOnly desiredCoMPosition = command.getDesiredCoMPosition();
      FrameVector3DReadOnly desiredCoMAcceleration = command.getDesiredCoMAcceleration();
      FrameOrientation3DReadOnly desiredBodyOrientation = command.getDesiredBodyOrientation();
      Vector3DReadOnly desiredBodyAngularVelocity = command.getDesiredBodyAngularVelocity();
      MPCContactPlane contactPlane = command.getContactPlaneHelper(0);

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
      desiredContactPointForce.addZ(Math.abs(gravityZ));

      angularVelocityErrorRateFromContact.cross(desiredContactPointForce, comPosition);
      angularVelocityErrorRateFromContact.scale(-mass);

      desiredContactPointForce.scale(mass / contactPlane.getNumberOfContactPoints());

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

      return angularVelocityErrorRate;
   }

   private static FrameVector3DReadOnly computeAngularErrorRate(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularErrorRate = new FrameVector3D();
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();
      FrameVector3D angularErrorRateFromAngularVelocityError = new FrameVector3D();

      Vector3DReadOnly desiredBodyAngularVelocity = command.getDesiredBodyAngularVelocity();
      angularErrorRateFromAngularError.cross(desiredBodyAngularVelocity, angularErrorAtCurrentTick);
      angularErrorRateFromAngularError.scale(-1.0);

      angularErrorRateFromAngularVelocityError.set(angularVelocityErrorAtCurrentTick);

      angularErrorRate.add(angularErrorRateFromAngularError, angularErrorRateFromAngularVelocityError);

      return angularErrorRate;
   }

}
