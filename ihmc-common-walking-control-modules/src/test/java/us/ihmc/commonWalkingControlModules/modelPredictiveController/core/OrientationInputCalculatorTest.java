package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteAngularVelocityOrientationCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class OrientationInputCalculatorTest
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

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));
         contactPlane.computeContactForce(omega, time);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         comPosition.get(comPositionVector);

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

         inputCalculator.compute(qpInput, command);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);

         /*
         DMatrixRMaj constructedVelocityErrorRateFromContact = new DMatrixRMaj(inputCalculator.getB0());
         CommonOps_DDRM.multAdd(inputCalculator.getB1(), comPositionVector, constructedVelocityErrorRateFromContact);
         CommonOps_DDRM.multAdd(inputCalculator.getB2(), contactForceVector, constructedVelocityErrorRateFromContact);

         MatrixTestTools.assertMatrixEquals(expectedVelocityErrorRateFromContact, constructedVelocityErrorRateFromContact, 1e-5);
         MatrixTestTools.assertMatrixEquals(expectedVelocityErrorRateFromContact, velocityErrorRateFromContact, 1e-5);

         MatrixTestTools.assertMatrixEquals(expectedRateVector, rateVector, 1e-5);
          */
      }
   }

   @Test
   public void testCoMObjectiveOneSegmentTwoContacts()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();
      leftContactPose.getPosition().setY(0.25);
      rightContactPose.getPosition().setY(-0.25);

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlane.computeBasisVectors(contactPolygon, rightContactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(leftContactPose, contactPolygon);
      contact.addContact(rightContactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoCoefficients = indexHandler.getRhoCoefficientsInSegment(0);
      int comCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment;
      QPInputTypeA qpInput = new QPInputTypeA(rhoCoefficients + comCoefficients + indexHandler.getTotalNumberOfOrientationTicks() * 6);

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         leftContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));
         rightContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) + leftContactPlane.getCoefficientSize());
         leftContactPlane.computeContactForce(omega, time);
         rightContactPlane.computeContactForce(omega, time);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, leftContactPlane, rightContactPlane);
         comPosition.get(comPositionVector);

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
         command.addContactPlaneHelper(leftContactPlane);
         command.addContactPlaneHelper(rightContactPlane);
         //
         inputCalculator.compute(qpInput, command);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);
      }
   }

   @Test
   public void testConstraintFormulationTwoSegmentsTwoContacts()
   {
      double orientationPreviewWindowLength = 0.75;

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();
      leftContactPose.getPosition().setY(0.2);
      rightContactPose.getPosition().setY(-0.2);

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlane.computeBasisVectors(contactPolygon, rightContactPose, mu);

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 0.5);
      contact0.addContact(leftContactPose, contactPolygon);
      contact0.addContact(rightContactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());


      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.getTimeInterval().setInterval(0.5, 1.0);
      contact1.addContact(leftContactPose, contactPolygon);
      contact1.addContact(rightContactPose, contactPolygon);
      contact1.setStartECMPPosition(new FramePoint3D());
      contact1.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact0);
      contactProviders.add(contact1);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      QPInputTypeA qpInput = new QPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);

      int numberOfFirstSegmentTrajectoryCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(0);
      int numberOfSecondSegmentTrajectoryCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(1);
      DMatrixRMaj firstSegmentTrajectoryCoefficients = new DMatrixRMaj(numberOfFirstSegmentTrajectoryCoefficients, 1);
      DMatrixRMaj secondSegmentTrajectoryCoefficients = new DMatrixRMaj(numberOfSecondSegmentTrajectoryCoefficients, 1);
      firstSegmentTrajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfFirstSegmentTrajectoryCoefficients, 10.0));
      secondSegmentTrajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfSecondSegmentTrajectoryCoefficients, 10.0));

      FrameVector3D initialAngularError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
      FrameVector3D initialAngularVelocityError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

      FrameVector3D angularErrorAtCurrentTick = new FrameVector3D(initialAngularError);
      FrameVector3D angularVelocityErrorAtCurrentTick = new FrameVector3D(initialAngularVelocityError);

      DMatrixRMaj fullVariableMatrix = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
      MatrixTools.setMatrixBlock(fullVariableMatrix, indexHandler.getComCoefficientStartIndex(0), 0, firstSegmentTrajectoryCoefficients, 0, 0, numberOfFirstSegmentTrajectoryCoefficients, 1, 1.0);
      MatrixTools.setMatrixBlock(fullVariableMatrix, indexHandler.getComCoefficientStartIndex(1), 0, secondSegmentTrajectoryCoefficients, 0, 0, numberOfSecondSegmentTrajectoryCoefficients, 1, 1.0);


      DMatrixRMaj Aeq = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), indexHandler.getTotalProblemSize());
      DMatrixRMaj beq = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);
      DMatrixRMaj value = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);

      double time = 0.0;
      int tick = 0;
      for (int segmentNumber = 0; segmentNumber < 2; segmentNumber++)
      {
         double tickDuration = indexHandler.getOrientationTickDuration(segmentNumber);
         int endTick = tick + indexHandler.getOrientationTicksInSegment(segmentNumber);

         DMatrixRMaj trajectoryCoefficients;
         if (segmentNumber == 0)
            trajectoryCoefficients = firstSegmentTrajectoryCoefficients;
         else
            trajectoryCoefficients = secondSegmentTrajectoryCoefficients;

         leftContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, LinearMPCIndexHandler.comCoefficientsPerSegment);
         rightContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, LinearMPCIndexHandler.comCoefficientsPerSegment + leftContactPlane.getCoefficientSize());

         for (; tick < endTick; tick++)
         {
            leftContactPlane.computeContactForce(omega, time);
            rightContactPlane.computeContactForce(omega, time);

            FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
            FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
            FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
            FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
            desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
            FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
            FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
            FrameVector3D desiredCoMAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

            FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time,
                                                                                omega,
                                                                                gravityZ,
                                                                                trajectoryCoefficients,
                                                                                leftContactPlane,
                                                                                rightContactPlane);

            DiscreteAngularVelocityOrientationCommand command = getCommand(time,
                                                                           tick,
                                                                           tickDuration,
                                                                           omega,
                                                                           segmentNumber,
                                                                           momentOfInertia,
                                                                           desiredCoMPosition,
                                                                           desiredCoMAcceleration,
                                                                           desiredBodyOrientation,
                                                                           desiredBodyAngularVelocity,
                                                                           desiredNetAngularMomentumRate,
                                                                           desiredInternalAngularMomentumRate,
                                                                           angularErrorAtCurrentTick,
                                                                           angularVelocityErrorAtCurrentTick,
                                                                           leftContactPlane,
                                                                           rightContactPlane);

            inputCalculator.compute(qpInput, command);

            assertTaskIsCorrect(indexHandler, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command, qpInput);
            assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);

            MatrixTools.setMatrixBlock(Aeq, 6 * tick, 0, qpInput.getTaskJacobian(), 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
            MatrixTools.setMatrixBlock(beq, 6 * tick, 0, qpInput.getTaskObjective(), 0, 0, 6, 1, 1.0);

            FrameVector3DReadOnly expectedAngularErrorRate = computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
            FrameVector3DReadOnly expectedAngularVelocityErrorRate = computeExpectedAngularVelocityErrorRate(mass,
                                                                                                             comPosition,
                                                                                                             angularErrorAtCurrentTick,
                                                                                                             angularVelocityErrorAtCurrentTick,
                                                                                                             command);

            angularErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularErrorRate, angularErrorAtCurrentTick);
            angularVelocityErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularVelocityErrorRate, angularVelocityErrorAtCurrentTick);

            int orientationIndex = indexHandler.getOrientationTickStartIndex(tick);
            angularErrorAtCurrentTick.get(orientationIndex, fullVariableMatrix);
            angularVelocityErrorAtCurrentTick.get(orientationIndex + 3, fullVariableMatrix);

            CommonOps_DDRM.mult(Aeq, fullVariableMatrix, value);
            MatrixTestTools.assertMatrixEquals(value, beq, 1e-5);

            time += tickDuration;
         }
      }

      CommonOps_DDRM.mult(Aeq, fullVariableMatrix, value);
      MatrixTestTools.assertMatrixEquals(value, beq, 1e-5);
   }

   @Test
   public void testObjectiveFormulation()
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

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));
         contactPlane.computeContactForce(omega, time);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         comPosition.get(comPositionVector);

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

         DMatrixRMaj gravityMatrix = new DMatrixRMaj(3, 1);
         gravityVector.get(gravityMatrix);


         DMatrixRMaj expectedA = new DMatrixRMaj(6, 6);
         DMatrixRMaj expectedB = new DMatrixRMaj(6, numberOfTrajectoryCoefficients);
         DMatrixRMaj expectedC = new DMatrixRMaj(6, 1);

         DMatrixRMaj skewDesiredAngular = new DMatrixRMaj(3, 3);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredBodyAngularVelocity, skewDesiredAngular);
         CommonOps_DDRM.scale(-1.0, skewDesiredAngular);

         CommonOps_DDRM.insert(skewDesiredAngular, expectedA, 0, 0);
         CommonOps_DDRM.insert(CommonOps_DDRM.identity(3), expectedA, 0, 3);
         CommonOps_DDRM.insert(inputCalculator.getB3(), expectedA, 3, 0);
         CommonOps_DDRM.insert(inputCalculator.getB4(), expectedA, 3, 3);


         MatrixTools.multAddBlock(inputCalculator.getB1(), MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlane), expectedB, 3, 0);
         MatrixTools.multAddBlock(mass, inputCalculator.getB2(), MPCTestHelper.getContactPointAccelerationJacobian(time, omega, contactPlane), expectedB, 3, 0);

         MatrixTools.setMatrixBlock(expectedC, 3, 0, inputCalculator.getB0(), 0, 0, 3, 1, 1.0);
         MatrixTools.multAddBlock(0.5 * time * time, inputCalculator.getB1(), gravityMatrix, expectedC, 3, 0);

         DMatrixRMaj contactForceJacobian = new DMatrixRMaj(3 * contactPlane.getNumberOfContactPoints(), contactPlane.getCoefficientSize());
         MatrixTools.setMatrixBlock(contactForceJacobian,
                                    0,
                                    0,
                                    MPCTestHelper.getContactPointAccelerationJacobian(time, omega, contactPlane),
                                    0,
                                    LinearMPCIndexHandler.comCoefficientsPerSegment,
                                    3 * contactPlane.getNumberOfContactPoints(),
                                    contactPlane.getCoefficientSize(),
                                    mass);
         MatrixTestTools.assertMatrixEquals(contactForceJacobian, inputCalculator.contactForceJacobian, 1e-6);

         MatrixTestTools.assertMatrixEquals(expectedA, inputCalculator.getContinuousAMatrix(), 1e-6);
         MatrixTestTools.assertMatrixEquals(expectedC, inputCalculator.getContinuousCMatrix(), 1e-6);
         MatrixTestTools.assertMatrixEquals(expectedB, inputCalculator.getContinuousBMatrix(), 1e-6);
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
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

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

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients);

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

         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);
      }
   }

   /** This currently does nothing. **/
   @Disabled
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
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

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

      FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
      FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
      FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
      FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
      desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
      FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

      double time = 0.0;
      for (int tick = 0; tick < indexHandler.getTotalNumberOfOrientationTicks(); tick++)
      {

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         FrameVector3DReadOnly comAcceleration = MPCTestHelper.computeCoMAcceleration(time, omega, gravityZ, trajectoryCoefficients, contactPlane);

         DiscreteAngularVelocityOrientationCommand command = getCommand(time,
                                                                        tick,
                                                                        tickDuration,
                                                                        omega,
                                                                        momentOfInertia,
                                                                        comPosition,
                                                                        comAcceleration,
                                                                        desiredBodyOrientation,
                                                                        desiredBodyAngularVelocity,
                                                                        desiredNetAngularMomentumRate,
                                                                        desiredInternalAngularMomentumRate,
                                                                        angularErrorAtCurrentTick,
                                                                        angularVelocityErrorAtCurrentTick,
                                                                        contactPlane);

         inputCalculator.compute(qpInput, command);

         FrameQuaternion expectedOrientationAtEndOfTick = new FrameQuaternion();
         FrameVector3D expectedAngularVelocityAtEndOfTick = new FrameVector3D();

         FrameVector3DReadOnly expectedAngularErrorRate = computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
         FrameVector3DReadOnly expectedAngularVelocityErrorRate = computeExpectedAngularVelocityErrorRate(mass,
                                                                                                          comPosition,
                                                                                                          angularErrorAtCurrentTick,
                                                                                                          angularVelocityErrorAtCurrentTick,
                                                                                                          command);

         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);

         angularErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularErrorRate, angularErrorAtCurrentTick);
         angularVelocityErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularVelocityErrorRate, angularVelocityErrorAtCurrentTick);

         angularErrorAtCurrentTick.get(6 * tick, orientationVariables);
         angularVelocityErrorAtCurrentTick.get(6 * tick + 3, orientationVariables);

         time += tickDuration;
      }

      DMatrixRMaj fullVariableMatrix = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
      MatrixTools.setMatrixBlock(fullVariableMatrix, 0, 0, trajectoryCoefficients, 0, 0, numberOfTrajectoryCoefficients, 1, 1.0);
      MatrixTools.setMatrixBlock(fullVariableMatrix,
                                 numberOfTrajectoryCoefficients,
                                 0,
                                 orientationVariables,
                                 0,
                                 0,
                                 6 * indexHandler.getTotalNumberOfOrientationTicks(),
                                 1,
                                 1.0);
   }

   private static void simulateForward(FrameQuaternionReadOnly currentOrientation,
                                       FrameVector3DReadOnly currentAngularVelocity,
                                       FrameQuaternionBasics integratedOrientationToPack,
                                       FrameVector3DBasics integratedAngularVelocityToPack,
                                       DMatrixRMaj trajectoryCoefficients,
                                       MPCContactPlane contactPlane,
                                       double omega,
                                       double time,
                                       double gravity,
                                       double durationToSimulate,
                                       Matrix3DReadOnly inertiaInBodyFrame,
                                       double mass)
   {
      FrameQuaternion integratedOrientation = new FrameQuaternion(currentOrientation);
      FrameVector3D integratedVelocity = new FrameVector3D(currentAngularVelocity);

      double endTime = time + durationToSimulate;
      double simulateDt = 1e-4;
      for (; time < endTime; time += simulateDt)
      {
         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravity, trajectoryCoefficients, contactPlane);

         DMatrixRMaj rhoAccelerationVector = MPCTestHelper.computeRhoAccelerationVector(time, omega, trajectoryCoefficients, contactPlane);
         FrameVector3D torqueAboutCoM = new FrameVector3D();
         int rhoStart = 0;
         for (int contactIdx = 0; contactIdx < contactPlane.getNumberOfContactPoints(); contactIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactIdx);
            for (int rhoIdx = 0; rhoIdx < contactPoint.getRhoSize(); rhoIdx++)
            {
               FrameVector3D force = new FrameVector3D(contactPoint.getBasisVector(rhoIdx));
               force.scale(mass * rhoAccelerationVector.get(rhoStart + rhoIdx));

               FrameVector3D momentArm = new FrameVector3D();
               momentArm.sub(contactPoint.getBasisVectorOrigin(), comPosition);

               FrameVector3D torque = new FrameVector3D();
               torque.cross(momentArm, force);

               torqueAboutCoM.add(torque);
            }
            rhoStart += contactPoint.getRhoSize();
         }

         Matrix3D inertiaInWorld = new Matrix3D(inertiaInBodyFrame);
         integratedOrientation.transform(inertiaInWorld);

         Vector3D acceleration = new Vector3D(torqueAboutCoM);
         inertiaInWorld.transform(acceleration);

         Vector3D rotationDuringTick = new Vector3D();
         rotationDuringTick.scaleAdd(simulateDt, integratedVelocity, rotationDuringTick);
         rotationDuringTick.scaleAdd(0.5 * simulateDt * simulateDt, acceleration, rotationDuringTick);

         integratedVelocity.scaleAdd(simulateDt, acceleration, integratedVelocity);

         AxisAngle rotation = new AxisAngle();
         rotation.setRotationVector(rotationDuringTick);

         integratedOrientation.append(rotation);
      }

      integratedOrientationToPack.set(integratedOrientation);
      integratedAngularVelocityToPack.set(integratedVelocity);
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
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

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
         angularVelocityErrorRateFromContact.scale(mass);

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
   public void testB1Alt()
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

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));
         contactPlane.computeContactForce(omega, time);

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         comPosition.get(comPositionVector);

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

         FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();

         FrameVector3D desiredContactPointForce = new FrameVector3D(desiredCoMAcceleration);
         desiredContactPointForce.sub(gravityVector);
         desiredContactPointForce.scale(mass / contactPlane.getNumberOfContactPoints());

         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            FrameVector3D torqueFromContact = new FrameVector3D();
            FrameVector3D coriolisForce = new FrameVector3D();

            coriolisForce.cross(desiredContactPointForce, comPosition);
            torqueFromContact.add(coriolisForce);

            angularVelocityErrorRateFromContact.add(torqueFromContact);
         }

         desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);


         DMatrixRMaj rateFromContact = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(inputCalculator.getB1(), comPositionVector, rateFromContact);

         DMatrixRMaj expectedRateFromContact = new DMatrixRMaj(3, 1);
         angularVelocityErrorRateFromContact.get(expectedRateFromContact);

         MatrixTestTools.assertMatrixEquals(expectedRateFromContact, rateFromContact, 1e-6);
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
   public void testB2Alt()
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

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));
         contactPlane.computeContactForce(omega, time);

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         comPosition.get(comPositionVector);

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

         FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();
         DMatrixRMaj contactForceVector = new DMatrixRMaj(contactPlane.getNumberOfContactPoints() * 3, 1);


         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);

            FrameVector3D torqueFromContact = new FrameVector3D();

            FrameVector3D desiredMomentArm = new FrameVector3D();
            desiredMomentArm.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);

            torqueFromContact.cross(desiredMomentArm, contactPoint.getContactAcceleration());
            torqueFromContact.scale(mass);

            angularVelocityErrorRateFromContact.add(torqueFromContact);

            contactPoint.getContactAcceleration().get(3 * contactPointIdx, contactForceVector);
         }
         CommonOps_DDRM.scale(mass, contactForceVector);

         desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

         DMatrixRMaj rateFromContact = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(inputCalculator.getB2(), contactForceVector, rateFromContact);

         DMatrixRMaj expectedRateFromContact = new DMatrixRMaj(3, 1);
         angularVelocityErrorRateFromContact.get(expectedRateFromContact);

         MatrixTestTools.assertMatrixEquals(expectedRateFromContact, rateFromContact, 1e-6);
      }
   }

   @Test
   public void testB0()
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

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));
         contactPlane.computeContactForce(omega, time);

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, trajectoryCoefficients, contactPlane);
         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         comPosition.get(comPositionVector);

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

         FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();
         FrameVector3D angularVelocityErrorRateFromContact2 = new FrameVector3D();

         FrameVector3D desiredContactPointForce = new FrameVector3D(desiredCoMAcceleration);
         desiredContactPointForce.sub(gravityVector);
         desiredContactPointForce.scale(mass / contactPlane.getNumberOfContactPoints());

         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);

            FrameVector3D desiredTorqueFromContact = new FrameVector3D();
            FrameVector3D desiredCoriolisForce = new FrameVector3D();
            FrameVector3D fullTorqueFromPoint = new FrameVector3D();

            FrameVector3D desiredTorqueAboutPoint = new FrameVector3D();

            FrameVector3D desiredMomentArm = new FrameVector3D();
            desiredMomentArm.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);

            desiredTorqueFromContact.cross(desiredMomentArm, desiredContactPointForce);
            desiredCoriolisForce.cross(desiredCoMPosition, desiredContactPointForce);

            fullTorqueFromPoint.sub(desiredCoriolisForce, desiredTorqueFromContact);
            angularVelocityErrorRateFromContact.add(fullTorqueFromPoint);

            FrameVector3D other = new FrameVector3D(contactPoint.getBasisVectorOrigin());
            other.scaleAdd(-2.0, desiredCoMPosition, contactPoint.getBasisVectorOrigin());
            desiredTorqueAboutPoint.cross(desiredContactPointForce, other);
            angularVelocityErrorRateFromContact2.add(desiredTorqueAboutPoint);

            EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(fullTorqueFromPoint, desiredTorqueAboutPoint, 1e-6);
         }

         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(angularVelocityErrorRateFromContact, angularVelocityErrorRateFromContact2, 1e-6);

         desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

         DMatrixRMaj expectedRateFromContact = new DMatrixRMaj(3, 1);
         angularVelocityErrorRateFromContact.get(expectedRateFromContact);

         MatrixTestTools.assertMatrixEquals(expectedRateFromContact, inputCalculator.getB0(), 1e-6);
      }
   }


   @Test
   public void testConstraintFormulationOneSegmentOneContact()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

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

         assertTaskIsCorrect(indexHandler,
                             angularErrorAtCurrentTick,
                             angularVelocityErrorAtCurrentTick,
                             trajectoryCoefficients,
                             inputCalculator,
                             command,
                             qpInput);
         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);

         MatrixTools.setMatrixBlock(Aeq, 6 * tick, 0, qpInput.getTaskJacobian(), 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
         MatrixTools.setMatrixBlock(beq, 6 * tick, 0, qpInput.getTaskObjective(), 0, 0, 6, 1, 1.0);

         FrameVector3DReadOnly expectedAngularErrorRate = computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
         FrameVector3DReadOnly expectedAngularVelocityErrorRate = computeExpectedAngularVelocityErrorRate(mass,
                                                                                                          comPosition,
                                                                                                          angularErrorAtCurrentTick,
                                                                                                          angularVelocityErrorAtCurrentTick,
                                                                                                          command);

         angularErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularErrorRate, angularErrorAtCurrentTick);
         angularVelocityErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularVelocityErrorRate, angularVelocityErrorAtCurrentTick);

         angularErrorAtCurrentTick.get(6 * tick, orientationVariables);
         angularVelocityErrorAtCurrentTick.get(6 * tick + 3, orientationVariables);

         time += tickDuration;
      }

      DMatrixRMaj fullVariableMatrix = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
      MatrixTools.setMatrixBlock(fullVariableMatrix, 0, 0, trajectoryCoefficients, 0, 0, numberOfTrajectoryCoefficients, 1, 1.0);
      MatrixTools.setMatrixBlock(fullVariableMatrix,
                                 numberOfTrajectoryCoefficients,
                                 0,
                                 orientationVariables,
                                 0,
                                 0,
                                 6 * indexHandler.getTotalNumberOfOrientationTicks(),
                                 1,
                                 1.0);

      DMatrixRMaj value = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);
      CommonOps_DDRM.mult(Aeq, fullVariableMatrix, value);

      MatrixTestTools.assertMatrixEquals(value, beq, 1e-5);
   }

   @Test
   public void testConstraintFormulationOneSegmentTwoContacts()
   {
      double orientationPreviewWindowLength = 0.75;

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();
      leftContactPose.getPosition().setY(0.2);
      rightContactPose.getPosition().setY(-0.2);

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlane.computeBasisVectors(contactPolygon, rightContactPose, mu);

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 0.5);
      contact0.addContact(leftContactPose, contactPolygon);
      contact0.addContact(rightContactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact0);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      QPInputTypeA qpInput = new QPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);

      int numberOfFirstSegmentTrajectoryCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(0);
      DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfFirstSegmentTrajectoryCoefficients, 1);
      DMatrixRMaj reactionForceCoefficients = new DMatrixRMaj(indexHandler.getRhoCoefficientsInSegment(0), 1);
      trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfFirstSegmentTrajectoryCoefficients, 10.0));
      MatrixTools.setMatrixBlock(reactionForceCoefficients, 0, 0, trajectoryCoefficients, LinearMPCIndexHandler.comCoefficientsPerSegment, 0, indexHandler.getRhoCoefficientsInSegment(0), 1, 1.0);

      FrameVector3D initialAngularError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
      FrameVector3D initialAngularVelocityError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

      FrameVector3D angularErrorAtCurrentTick = new FrameVector3D(initialAngularError);
      FrameVector3D angularVelocityErrorAtCurrentTick = new FrameVector3D(initialAngularVelocityError);

      DMatrixRMaj orientationVariables = new DMatrixRMaj(6 * indexHandler.getOrientationTicksInSegment(0), 1);

      DMatrixRMaj Aeq = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), indexHandler.getTotalProblemSize());
      DMatrixRMaj beq = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);

      double time = 0.0;
      int tick = 0;
      int segmentNumber = 0;

      double tickDuration = indexHandler.getOrientationTickDuration(segmentNumber);
      int endTick = tick + indexHandler.getOrientationTicksInSegment(segmentNumber);

      leftContactPlane.computeContactForceCoefficientMatrix(reactionForceCoefficients, 0);
      rightContactPlane.computeContactForceCoefficientMatrix(reactionForceCoefficients, leftContactPlane.getCoefficientSize());

      for (; tick < endTick; tick++)
      {
         leftContactPlane.computeContactForce(omega, time);
         rightContactPlane.computeContactForce(omega, time);

         DMatrixRMaj expectedReactionForceVector = new DMatrixRMaj(3 * (leftContactPlane.getNumberOfContactPoints() + rightContactPlane.getNumberOfContactPoints()), 1);
         for (int i = 0; i < leftContactPlane.getNumberOfContactPoints(); i++)
            leftContactPlane.getContactPointHelper(i).getContactAcceleration().get(3 * i, expectedReactionForceVector);
         for (int i = 0; i < rightContactPlane.getNumberOfContactPoints(); i++)
            rightContactPlane.getContactPointHelper(i).getContactAcceleration().get(3 * (i + leftContactPlane.getNumberOfContactPoints()), expectedReactionForceVector);
         CommonOps_DDRM.scale(mass, expectedReactionForceVector);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
         desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time,
                                                                             omega,
                                                                             gravityZ,
                                                                             trajectoryCoefficients,
                                                                             leftContactPlane,
                                                                             rightContactPlane);

         DiscreteAngularVelocityOrientationCommand command = getCommand(time,
                                                                        tick,
                                                                        tickDuration,
                                                                        omega,
                                                                        segmentNumber,
                                                                        momentOfInertia,
                                                                        desiredCoMPosition,
                                                                        desiredCoMAcceleration,
                                                                        desiredBodyOrientation,
                                                                        desiredBodyAngularVelocity,
                                                                        desiredNetAngularMomentumRate,
                                                                        desiredInternalAngularMomentumRate,
                                                                        angularErrorAtCurrentTick,
                                                                        angularVelocityErrorAtCurrentTick,
                                                                        leftContactPlane,
                                                                        rightContactPlane);

         inputCalculator.compute(qpInput, command);

         DMatrixRMaj reactionForceVector = new DMatrixRMaj(3 * (leftContactPlane.getNumberOfContactPoints() + rightContactPlane.getNumberOfContactPoints()), 1);
         CommonOps_DDRM.mult(inputCalculator.contactForceJacobian, reactionForceCoefficients, reactionForceVector);
         MatrixTestTools.assertMatrixEquals(expectedReactionForceVector, reactionForceVector, 1e-6);

         MatrixTools.setMatrixBlock(Aeq, 6 * tick, 0, qpInput.getTaskJacobian(), 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
         MatrixTools.setMatrixBlock(beq, 6 * tick, 0, qpInput.getTaskObjective(), 0, 0, 6, 1, 1.0);

         assertTaskIsCorrect(indexHandler,
                             angularErrorAtCurrentTick,
                             angularVelocityErrorAtCurrentTick,
                             trajectoryCoefficients,
                             inputCalculator,
                             command,
                             qpInput);
         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);

         FrameVector3DReadOnly expectedAngularErrorRate = computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
         FrameVector3DReadOnly expectedAngularVelocityErrorRate = computeExpectedAngularVelocityErrorRate(mass,
                                                                                                          comPosition,
                                                                                                          angularErrorAtCurrentTick,
                                                                                                          angularVelocityErrorAtCurrentTick,
                                                                                                          command);

         angularErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularErrorRate, angularErrorAtCurrentTick);
         angularVelocityErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularVelocityErrorRate, angularVelocityErrorAtCurrentTick);

         angularErrorAtCurrentTick.get(6 * tick, orientationVariables);
         angularVelocityErrorAtCurrentTick.get(6 * tick + 3, orientationVariables);

         time += tickDuration;
      }

      DMatrixRMaj fullVariableMatrix = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
      MatrixTools.setMatrixBlock(fullVariableMatrix, indexHandler.getComCoefficientStartIndex(0), 0, trajectoryCoefficients, 0, 0, numberOfFirstSegmentTrajectoryCoefficients, 1, 1.0);
      MatrixTools.setMatrixBlock(fullVariableMatrix, indexHandler.getOrientationStartIndices(0), 0, orientationVariables, 0, 0, 6 * indexHandler.getOrientationTicksInSegment(0), 1, 1.0);

      DMatrixRMaj value = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);
      CommonOps_DDRM.mult(Aeq, fullVariableMatrix, value);

      MatrixTestTools.assertMatrixEquals(value, beq, 1e-5);
   }

   @Test
   public void testConstraintFormulationTwoSegmentsOneContact()
   {
      double orientationPreviewWindowLength = 0.75;

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D leftContactPose = new FramePose3D();

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 0.5);
      contact0.addContact(leftContactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());


      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.getTimeInterval().setInterval(0.5, 1.0);
      contact1.addContact(leftContactPose, contactPolygon);
      contact1.setStartECMPPosition(new FramePoint3D());
      contact1.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact0);
      contactProviders.add(contact1);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationInputCalculator inputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      QPInputTypeA qpInput = new QPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);

      int numberOfFirstSegmentTrajectoryCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(0);
      int numberOfSecondSegmentTrajectoryCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(1);
      DMatrixRMaj firstSegmentTrajectoryCoefficients = new DMatrixRMaj(numberOfFirstSegmentTrajectoryCoefficients, 1);
      DMatrixRMaj secondSegmentTrajectoryCoefficients = new DMatrixRMaj(numberOfSecondSegmentTrajectoryCoefficients, 1);
      firstSegmentTrajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfFirstSegmentTrajectoryCoefficients, 10.0));
      secondSegmentTrajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfSecondSegmentTrajectoryCoefficients, 10.0));

      DMatrixRMaj fullVariableMatrix = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);
      CommonOps_DDRM.insert(firstSegmentTrajectoryCoefficients, fullVariableMatrix, indexHandler.getComCoefficientStartIndex(0), 0);
      CommonOps_DDRM.insert(secondSegmentTrajectoryCoefficients, fullVariableMatrix, indexHandler.getComCoefficientStartIndex(1), 0);

      FrameVector3D initialAngularError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
      FrameVector3D initialAngularVelocityError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

      FrameVector3D angularErrorAtCurrentTick = new FrameVector3D(initialAngularError);
      FrameVector3D angularVelocityErrorAtCurrentTick = new FrameVector3D(initialAngularVelocityError);

      DMatrixRMaj Aeq = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), indexHandler.getTotalProblemSize());
      DMatrixRMaj beq = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);
      DMatrixRMaj value = new DMatrixRMaj(6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);

      double time = 0.0;
      int tick = 0;
      for (int segmentNumber = 0; segmentNumber < 2; segmentNumber++)
      {
         double tickDuration = indexHandler.getOrientationTickDuration(segmentNumber);
         int endTick = tick + indexHandler.getOrientationTicksInSegment(segmentNumber);

         DMatrixRMaj trajectoryCoefficients;
         if (segmentNumber == 0)
         {
            trajectoryCoefficients = firstSegmentTrajectoryCoefficients;
         }
         else
         {
            trajectoryCoefficients = secondSegmentTrajectoryCoefficients;
         }

         leftContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, LinearMPCIndexHandler.comCoefficientsPerSegment);

         for (; tick < endTick; tick++)
         {

            leftContactPlane.computeContactForce(omega, time);

            FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
            FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
            FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
            FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
            desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
            FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
            FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
            FrameVector3D desiredCoMAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

            FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time,
                                                                                omega,
                                                                                gravityZ,
                                                                                trajectoryCoefficients,
                                                                                leftContactPlane);

            DiscreteAngularVelocityOrientationCommand command = getCommand(time,
                                                                           tick,
                                                                           tickDuration,
                                                                           omega,
                                                                           segmentNumber,
                                                                           momentOfInertia,
                                                                           desiredCoMPosition,
                                                                           desiredCoMAcceleration,
                                                                           desiredBodyOrientation,
                                                                           desiredBodyAngularVelocity,
                                                                           desiredNetAngularMomentumRate,
                                                                           desiredInternalAngularMomentumRate,
                                                                           angularErrorAtCurrentTick,
                                                                           angularVelocityErrorAtCurrentTick,
                                                                           leftContactPlane);

            inputCalculator.compute(qpInput, command);

            assertTaskIsCorrect(indexHandler,
                                angularErrorAtCurrentTick,
                                angularVelocityErrorAtCurrentTick,
                                trajectoryCoefficients,
                                inputCalculator,
                                command,
                                qpInput);
            assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);

            MatrixTools.setMatrixBlock(Aeq, 6 * tick, 0, qpInput.getTaskJacobian(), 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
            MatrixTools.setMatrixBlock(beq, 6 * tick, 0, qpInput.getTaskObjective(), 0, 0, 6, 1, 1.0);

            FrameVector3DReadOnly expectedAngularErrorRate = computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
            FrameVector3DReadOnly expectedAngularVelocityErrorRate = computeExpectedAngularVelocityErrorRate(mass,
                                                                                                             comPosition,
                                                                                                             angularErrorAtCurrentTick,
                                                                                                             angularVelocityErrorAtCurrentTick,
                                                                                                             command);

            angularErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularErrorRate, angularErrorAtCurrentTick);
            angularVelocityErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularVelocityErrorRate, angularVelocityErrorAtCurrentTick);

            int tickStartIndex = indexHandler.getOrientationTickStartIndex(tick);
            angularErrorAtCurrentTick.get(tickStartIndex, fullVariableMatrix);
            angularVelocityErrorAtCurrentTick.get(tickStartIndex + 3, fullVariableMatrix);

            CommonOps_DDRM.mult(Aeq, fullVariableMatrix, value);
            MatrixTestTools.assertMatrixEquals(value, beq, 1e-5);

            time += tickDuration;
         }
      }

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
                                                                       MPCContactPlane... contactPlanes)
   {
      return getCommand(time,
                        nextTickId,
                        tickDuration,
                        omega,
                        0,
                        momentOfInertia,
                        desiredCoMPosition,
                        desiredCoMAcceleration,
                        desiredBodyOrientation,
                        desiredBodyAngularVelocity,
                        desiredNetAngularMomentumRate,
                        desiredInternalAngularMomentumRate,
                        currentAxisAngleError,
                        currentAngularVelocityError,
                        contactPlanes);
   }

   private static DiscreteAngularVelocityOrientationCommand getCommand(double time,
                                                                       int nextTickId,
                                                                       double tickDuration,
                                                                       double omega,
                                                                       int segmentNumber,
                                                                       Matrix3DReadOnly momentOfInertia,
                                                                       FramePoint3DReadOnly desiredCoMPosition,
                                                                       FrameVector3DReadOnly desiredCoMAcceleration,
                                                                       FrameOrientation3DReadOnly desiredBodyOrientation,
                                                                       FrameVector3DReadOnly desiredBodyAngularVelocity,
                                                                       FrameVector3DReadOnly desiredNetAngularMomentumRate,
                                                                       FrameVector3DReadOnly desiredInternalAngularMomentumRate,
                                                                       FrameVector3DReadOnly currentAxisAngleError,
                                                                       FrameVector3DReadOnly currentAngularVelocityError,
                                                                       MPCContactPlane... contactPlanes)
   {

      DiscreteAngularVelocityOrientationCommand command = new DiscreteAngularVelocityOrientationCommand();
      command.setMomentOfInertiaInBodyFrame(momentOfInertia);
      command.setDesiredCoMAcceleration(desiredCoMAcceleration);
      command.setDesiredCoMPosition(desiredCoMPosition);
      command.setDesiredBodyOrientation(desiredBodyOrientation);
      command.setDesiredBodyAngularVelocityInBodyFrame(desiredBodyAngularVelocity);
      command.setTimeOfConstraint(time);
      command.setSegmentNumber(segmentNumber);
      command.setEndingDiscreteTickId(nextTickId);
      command.setDurationOfHold(tickDuration);
      command.setOmega(omega);
      command.setDesiredNetAngularMomentumRate(desiredNetAngularMomentumRate);
      command.setDesiredInternalAngularMomentumRate(desiredInternalAngularMomentumRate);
      for (MPCContactPlane contactPlane : contactPlanes)
         command.addContactPlaneHelper(contactPlane);

      if (nextTickId == 0)
      {
         command.setCurrentAxisAngleError(currentAxisAngleError);
         command.setCurrentBodyAngularVelocityErrorInBodyFrame(currentAngularVelocityError);
      }

      return command;
   }

   private static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRate(double mass,
                                                                                FramePoint3DReadOnly comPosition,
                                                                                FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularVelocityErrorRate = new FrameVector3D();
      FrameVector3DReadOnly angularVelocityErrorRateFromAngularError = computeExpectedAngularVelocityErrorRateFromAngularError(angularErrorAtCurrentTick, command);
      FrameVector3DReadOnly angularVelocityErrorRateFromAngularVelocityError = computeExpectedAngularVelocityErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick,
                                                                                                                                               command);
      FrameVector3DReadOnly angularVelocityErrorRateFromContact = computeExpectedAngularVelocityErrorRateFromContact(mass, comPosition, command);

      angularVelocityErrorRate.set(angularVelocityErrorRateFromAngularError);
      angularVelocityErrorRate.add(angularVelocityErrorRateFromAngularVelocityError);
      angularVelocityErrorRate.add(angularVelocityErrorRateFromContact);


      return angularVelocityErrorRate;
   }

   private static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                                DiscreteAngularVelocityOrientationCommand command)
   {
      FrameOrientation3DReadOnly desiredBodyOrientation = command.getDesiredBodyOrientation();
      FrameVector3D angularVelocityErrorRateFromAngularError = new FrameVector3D();

      FrameVector3D desiredBodyAngularMomentumRate = new FrameVector3D();
      desiredBodyAngularMomentumRate.sub(command.getDesiredNetAngularMomentumRate(), command.getDesiredInternalAngularMomentumRate());
      Matrix3DReadOnly momentOfInertia = command.getMomentOfInertiaInBodyFrame();

      FrameVector3D tempVector = new FrameVector3D(desiredBodyAngularMomentumRate);
      desiredBodyOrientation.inverseTransform(tempVector);
      angularVelocityErrorRateFromAngularError.cross(tempVector, angularErrorAtCurrentTick);
      momentOfInertia.inverseTransform(angularVelocityErrorRateFromAngularError);

      return angularVelocityErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                                        DiscreteAngularVelocityOrientationCommand command)
   {
      Vector3DReadOnly desiredBodyAngularVelocity = command.getDesiredBodyAngularVelocity();
      Matrix3DReadOnly momentOfInertia = command.getMomentOfInertiaInBodyFrame();

      FrameVector3D angularVelocityErrorRateFromAngularVelocityError = new FrameVector3D();

      FrameVector3D tempVector = new FrameVector3D();
      FrameVector3D tempVector2 = new FrameVector3D();
      Matrix3D tempMatrix = new Matrix3D();
      momentOfInertia.inverseTransform(desiredBodyAngularVelocity, tempVector);
      MatrixMissingTools.toSkewSymmetricMatrix(tempVector, tempMatrix);

      tempMatrix.transform(angularVelocityErrorAtCurrentTick, angularVelocityErrorRateFromAngularVelocityError);
      momentOfInertia.transform(angularVelocityErrorAtCurrentTick, tempVector);
      tempVector2.cross(desiredBodyAngularVelocity, tempVector);
      angularVelocityErrorRateFromAngularVelocityError.sub(tempVector2);
      momentOfInertia.inverseTransform(angularVelocityErrorRateFromAngularVelocityError);

      return angularVelocityErrorRateFromAngularVelocityError;
   }

   private static FrameVector3DReadOnly computeExpectedAngularErrorRateFromContact()
   {
      return new FrameVector3D();
   }

   private static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRateFromContact(double mass,
                                                                                           FramePoint3DReadOnly comPosition,
                                                                                           DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();

      Matrix3DReadOnly momentOfInertia = command.getMomentOfInertiaInBodyFrame();
      FramePoint3DReadOnly desiredCoMPosition = command.getDesiredCoMPosition();
      FrameVector3DReadOnly desiredCoMAcceleration = command.getDesiredCoMAcceleration();
      FrameOrientation3DReadOnly desiredBodyOrientation = command.getDesiredBodyOrientation();

      List<MPCContactPoint> allContactPoints = new ArrayList<>();

      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         for (int j = 0; j < command.getContactPlaneHelper(i).getNumberOfContactPoints(); j++)
            allContactPoints.add(command.getContactPlaneHelper(i).getContactPointHelper(j));
      }

      FrameVector3D desiredContactPointForce = new FrameVector3D(desiredCoMAcceleration);
      desiredContactPointForce.addZ(Math.abs(gravityZ));
      desiredContactPointForce.scale(mass / allContactPoints.size());

      for (int contactIdx = 0; contactIdx < allContactPoints.size(); contactIdx++)
      {
         MPCContactPoint contactPoint = allContactPoints.get(contactIdx);

         FrameVector3D desiredMomentArm = new FrameVector3D();
         desiredMomentArm.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);

         FrameVector3D contactForceError = new FrameVector3D(contactPoint.getContactAcceleration());
         contactForceError.scale(mass);
         contactForceError.sub(desiredContactPointForce);

         FrameVector3D comError = new FrameVector3D();
         comError.sub(comPosition, desiredCoMPosition);

         FrameVector3D torqueFromContact = new FrameVector3D();
         torqueFromContact.cross(desiredMomentArm, contactForceError);

         FrameVector3D coriolisForce = new FrameVector3D();
         coriolisForce.cross(desiredContactPointForce, comError);
         torqueFromContact.add(coriolisForce);

         angularVelocityErrorRateFromContact.add(torqueFromContact);
      }

      desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
      momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

      return angularVelocityErrorRateFromContact;
   }



   private static FrameVector3DReadOnly computeExpectedAngularErrorRate(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                        FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                        DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularErrorRate = new FrameVector3D();
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D(computeExpectedAngularErrorRateFromAngularError(angularErrorAtCurrentTick, command));
      FrameVector3D angularErrorRateFromAngularVelocityError = new FrameVector3D(computeExpectedAngularErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick));

      angularErrorRate.add(angularErrorRateFromAngularError, angularErrorRateFromAngularVelocityError);

      return angularErrorRate;
   }

   private static FrameVector3DReadOnly computeExpectedAngularErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                        DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();

      Vector3DReadOnly desiredBodyAngularVelocity = command.getDesiredBodyAngularVelocity();
      angularErrorRateFromAngularError.cross(desiredBodyAngularVelocity, angularErrorAtCurrentTick);
      angularErrorRateFromAngularError.scale(-1.0);

      return angularErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeExpectedAngularErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick)
   {
      return new FrameVector3D(angularVelocityErrorAtCurrentTick);
   }

   private static void assertRateFromAngularErrorIsCorrect(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                           OrientationInputCalculator inputCalculator,
                                                           DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3DReadOnly expectedAngularVelocityErrorRateFromAngularError = computeExpectedAngularVelocityErrorRateFromAngularError(
            angularErrorAtCurrentTick,
            command);
      FrameVector3DReadOnly expectedAngularErrorRateFromAngularError = computeExpectedAngularErrorRateFromAngularError(angularErrorAtCurrentTick, command);
      FrameVector3DReadOnly actualAngularErrorRateFromAngularError = computeActualAngularErrorRateFromAngularError(angularErrorAtCurrentTick, inputCalculator);
      FrameVector3DReadOnly actualAngularVelocityErrorRateFromAngularError = computeActualAngularVelocityErrorRateFromAngularError(angularErrorAtCurrentTick,
                                                                                                                                   inputCalculator);

      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate from angular error is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularErrorRateFromAngularError,
                                                                  actualAngularErrorRateFromAngularError,
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate from angular error is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularVelocityErrorRateFromAngularError,
                                                                  actualAngularVelocityErrorRateFromAngularError,
                                                                  1e-6);
   }

   private static void assertRateFromAngularVelocityErrorIsCorrect(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                   OrientationInputCalculator inputCalculator,
                                                                   DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3DReadOnly angularErrorRateFromAngularVelocityError = computeExpectedAngularErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick);

      FrameVector3DReadOnly expectedAngularVelocityErrorRateFromAngularVelocityError = computeExpectedAngularVelocityErrorRateFromAngularVelocityError(
            angularVelocityErrorAtCurrentTick,
            command);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate from angular velocity error is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  angularErrorRateFromAngularVelocityError,
                                                                  computeActualAngularErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator),
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate from angular velocity error is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularVelocityErrorRateFromAngularVelocityError,
                                                                  computeActualAngularVelocityErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator),
                                                                  1e-6);
   }

   private static void assertRateFromContactIsCorrect(FramePoint3DReadOnly comPosition,
                                                      DMatrixRMaj trajectoryCoefficients,
                                                      OrientationInputCalculator inputCalculator,
                                                      DiscreteAngularVelocityOrientationCommand command)
   {

      FrameVector3DReadOnly expectedAngularErrorRateFromContact = computeExpectedAngularErrorRateFromContact();
      FrameVector3DReadOnly expectedAngularVelocityErrorRateFromContact = computeExpectedAngularVelocityErrorRateFromContact(mass, comPosition, command);
      FrameVector3DReadOnly actualAngularErrorRateFromContact = computeActualAngularErrorRateFromContact(trajectoryCoefficients, inputCalculator);
      FrameVector3DReadOnly actualAngularVelocityErrorRateFromContact = computeActualAngularVelocityErrorRateFromContact(trajectoryCoefficients,
                                                                                                                         inputCalculator);

      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate from contact is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularErrorRateFromContact,
                                                                  actualAngularErrorRateFromContact,
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate from contact is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularVelocityErrorRateFromContact,
                                                                  actualAngularVelocityErrorRateFromContact,
                                                                  1e-6);
   }

   private static void assertRateIsCorrect(FramePoint3DReadOnly comPosition,
                                           FrameVector3DReadOnly angularErrorAtCurrentTick,
                                           FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                           DMatrixRMaj trajectoryCoefficients,
                                           OrientationInputCalculator inputCalculator,
                                           DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3DReadOnly expectedAngularErrorRate = computeExpectedAngularErrorRate(angularErrorAtCurrentTick,
                                                                                       angularVelocityErrorAtCurrentTick,
                                                                                       command);
      FrameVector3DReadOnly actualAngularErrorRate = computeActualAngularErrorRate(angularErrorAtCurrentTick,
                                                                                   angularVelocityErrorAtCurrentTick,
                                                                                   trajectoryCoefficients,
                                                                                   inputCalculator);
      FrameVector3DReadOnly expectedAngularVelocityErrorRate = computeExpectedAngularVelocityErrorRate(mass,
                                                                                                       comPosition,
                                                                                                       angularErrorAtCurrentTick,
                                                                                                       angularVelocityErrorAtCurrentTick,
                                                                                                       command);
      FrameVector3DReadOnly actualAngularVelocityErrorRate = computeActualAngularVelocityErrorRate(angularErrorAtCurrentTick,
                                                                                                   angularVelocityErrorAtCurrentTick,
                                                                                                   trajectoryCoefficients,
                                                                                                   inputCalculator);

      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularErrorRate,
                                                                  actualAngularErrorRate,
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularVelocityErrorRate,
                                                                  actualAngularVelocityErrorRate,
                                                                  1e-6);
   }

   private static void assertAllRatesAreCorrect(FramePoint3DReadOnly comPosition,
                                           FrameVector3DReadOnly angularErrorAtCurrentTick,
                                           FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                           DMatrixRMaj trajectoryCoefficients,
                                           OrientationInputCalculator inputCalculator,
                                           DiscreteAngularVelocityOrientationCommand command)
   {
      assertRateFromAngularErrorIsCorrect(angularErrorAtCurrentTick, inputCalculator, command);
      assertRateFromAngularVelocityErrorIsCorrect(angularVelocityErrorAtCurrentTick, inputCalculator, command);
      assertRateFromContactIsCorrect(comPosition, trajectoryCoefficients, inputCalculator, command);
      assertRateIsCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);
   }

   private static void assertTaskIsCorrect(SE3MPCIndexHandler indexHandler,
                                           FrameVector3DReadOnly angularErrorAtCurrentTick,
                                           FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                           DMatrixRMaj trajectoryCoefficients,
                                           OrientationInputCalculator inputCalculator,
                                           DiscreteAngularVelocityOrientationCommand command,
                                           QPInputTypeA qpInputToTest)
   {
      int tick = command.getEndDiscreteTickId();
      int segmentNumber = command.getSegmentNumber();
      DMatrixRMaj jacobianExpected = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
      DMatrixRMaj objectiveExpected = new DMatrixRMaj(6, 1);

      DMatrixRMaj stateVector = new DMatrixRMaj(6, 1);
      angularErrorAtCurrentTick.get(stateVector);
      angularVelocityErrorAtCurrentTick.get(3, stateVector);

      if (tick > 0)
      {
         MatrixTools.addMatrixBlock(jacobianExpected, 0, indexHandler.getOrientationTickStartIndex(tick), CommonOps_DDRM.identity(6), 0, 0, 6, 6, 1.0);
         MatrixTools.addMatrixBlock(jacobianExpected, 0, indexHandler.getOrientationTickStartIndex(tick - 1), inputCalculator.getDiscreteAMatrix(), 0, 0, 6, 6, -1.0);
         MatrixTools.addMatrixBlock(jacobianExpected, 0, indexHandler.getComCoefficientStartIndex(segmentNumber), inputCalculator.getDiscreteBMatrix(), 0, 0, 6, trajectoryCoefficients.getNumRows(), -1.0);

         objectiveExpected.set(inputCalculator.getDiscreteCMatrix());
      }
      else
      {
         MatrixTools.setMatrixBlock(jacobianExpected, 0, indexHandler.getComCoefficientStartIndex(segmentNumber), inputCalculator.getDiscreteBMatrix(), 0, 0, 6, trajectoryCoefficients.getNumRows(), -1.0);
         MatrixTools.addMatrixBlock(jacobianExpected, 0, indexHandler.getOrientationTickStartIndex(tick), CommonOps_DDRM.identity(6), 0, 0, 6, 6, 1.0);

         objectiveExpected.set(inputCalculator.getDiscreteCMatrix());
         CommonOps_DDRM.multAdd(inputCalculator.getDiscreteAMatrix(), stateVector, objectiveExpected);
      }

      String failureMessage = "Failed at tick " + tick;
      MatrixTestTools.assertMatrixEquals(failureMessage, jacobianExpected, qpInputToTest.getTaskJacobian(), 1e-6);
      MatrixTestTools.assertMatrixEquals(failureMessage, objectiveExpected, qpInputToTest.getTaskObjective(), 1e-6);
   }

   private static FrameVector3DReadOnly computeActualAngularErrorRate(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                      FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                      DMatrixRMaj trajectoryCoefficients,
                                                                      OrientationInputCalculator inputCalculator)
   {
      FrameVector3D actualAngularErrorRate = new FrameVector3D();
      actualAngularErrorRate.set(computeActualRateVector(angularErrorAtCurrentTick,
                                                         angularVelocityErrorAtCurrentTick,
                                                         trajectoryCoefficients,
                                                         inputCalculator));

      return actualAngularErrorRate;
   }

   private static FrameVector3DReadOnly computeActualAngularVelocityErrorRate(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                              FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                              DMatrixRMaj trajectoryCoefficients,
                                                                              OrientationInputCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRate = new FrameVector3D();
      actualAngularVelocityErrorRate.set(3, computeActualRateVector(angularErrorAtCurrentTick,
                                                                    angularVelocityErrorAtCurrentTick,
                                                                    trajectoryCoefficients,
                                                                    inputCalculator));

      return actualAngularVelocityErrorRate;
   }

   private static FrameVector3DReadOnly computeActualAngularErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                      OrientationInputCalculator inputCalculator)
   {
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();
      angularErrorRateFromAngularError.set(computeActualRateVectorFromAngularError(angularErrorAtCurrentTick, inputCalculator));

      return angularErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeActualAngularVelocityErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                              OrientationInputCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRateFromAngularError = new FrameVector3D();
      actualAngularVelocityErrorRateFromAngularError.set(3, computeActualRateVectorFromAngularError(angularErrorAtCurrentTick, inputCalculator));

      return actualAngularVelocityErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeActualAngularErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                              OrientationInputCalculator inputCalculator)
   {
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();
      angularErrorRateFromAngularError.set(computeActualRateVectorFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator));

      return angularErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeActualAngularVelocityErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                                      OrientationInputCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRateFromAngularError = new FrameVector3D();
      actualAngularVelocityErrorRateFromAngularError.set(3, computeActualRateVectorFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator));

      return actualAngularVelocityErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeActualAngularErrorRateFromContact(DMatrixRMaj trajectoryCoefficients,
                                                                                 OrientationInputCalculator inputCalculator)
   {
      FrameVector3D angularErrorRateFromContact = new FrameVector3D();
      angularErrorRateFromContact.set(computeActualRateVectorFromContact(trajectoryCoefficients, inputCalculator));

      return angularErrorRateFromContact;
   }

   private static FrameVector3DReadOnly computeActualAngularVelocityErrorRateFromContact(DMatrixRMaj trajectoryCoefficients,
                                                                                         OrientationInputCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRateFromContact = new FrameVector3D();
      actualAngularVelocityErrorRateFromContact.set(3, computeActualRateVectorFromContact(trajectoryCoefficients, inputCalculator));

      return actualAngularVelocityErrorRateFromContact;
   }

   private static DMatrixRMaj computeActualRateVector(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                      FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                      DMatrixRMaj trajectoryCoefficients,
                                                      OrientationInputCalculator inputCalculator)
   {
      DMatrixRMaj stateVector = new DMatrixRMaj(6, 1);
      DMatrixRMaj rateVector = new DMatrixRMaj(6, 1);

      angularErrorAtCurrentTick.get(stateVector);
      angularVelocityErrorAtCurrentTick.get(3, stateVector);

      CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), stateVector, rateVector);
      CommonOps_DDRM.multAdd(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateVector);
      CommonOps_DDRM.addEquals(rateVector, inputCalculator.getContinuousCMatrix());

      return rateVector;
   }


   private static DMatrixRMaj computeActualRateVectorFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                      OrientationInputCalculator inputCalculator)
   {
      DMatrixRMaj angleErrorStateVector = new DMatrixRMaj(6, 1);
      DMatrixRMaj rateFromAngleError = new DMatrixRMaj(6, 1);

      angularErrorAtCurrentTick.get(angleErrorStateVector);

      CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), angleErrorStateVector, rateFromAngleError);

      return rateFromAngleError;
   }

   private static DMatrixRMaj computeActualRateVectorFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                      OrientationInputCalculator inputCalculator)
   {
      DMatrixRMaj velocityErrorStateVector = new DMatrixRMaj(6, 1);
      DMatrixRMaj rateFromAngularVelocityError = new DMatrixRMaj(6, 1);

      angularVelocityErrorAtCurrentTick.get(3, velocityErrorStateVector);

      CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), velocityErrorStateVector, rateFromAngularVelocityError);

      return rateFromAngularVelocityError;
   }


   private static DMatrixRMaj computeActualRateVectorFromContact(DMatrixRMaj trajectoryCoefficients,
                                                                 OrientationInputCalculator inputCalculator)
   {
      DMatrixRMaj rateFromContact = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateFromContact);
      CommonOps_DDRM.addEquals(rateFromContact, inputCalculator.getContinuousCMatrix());

      return rateFromContact;
   }

}
