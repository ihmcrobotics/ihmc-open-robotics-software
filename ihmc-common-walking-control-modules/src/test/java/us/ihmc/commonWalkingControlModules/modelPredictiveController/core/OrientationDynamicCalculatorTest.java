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

public class OrientationDynamicCalculatorTest
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

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

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

         inputCalculator.compute(command);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);
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
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoCoefficients = indexHandler.getRhoCoefficientsInSegment(0);
      int comCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment;

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
         inputCalculator.compute(command);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);
      }
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
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

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
         inputCalculator.compute(command);

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
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoCoefficients = 0;
      int comCoefficients = 6;

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
         inputCalculator.compute(command);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);
      }
   }


   private FrameVector3DReadOnly getNetCoMTorque(FramePoint3DReadOnly comPosition, MPCContactPlane... contactPlanes)
   {
      FrameVector3D netCoMTorque = new FrameVector3D();
      for (MPCContactPlane contactPlane : contactPlanes)
      {
         for (int leftContact = 0; leftContact < contactPlane.getNumberOfContactPoints(); leftContact++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(leftContact);
            FrameVector3D leverArm = new FrameVector3D();
            leverArm.sub(contactPoint.getBasisVectorOrigin(), comPosition);

            FrameVector3D contactTorque = new FrameVector3D();
            contactTorque.cross(leverArm, contactPoint.getContactAcceleration());

            netCoMTorque.scaleAdd(mass, contactTorque, netCoMTorque);
         }
      }

      return netCoMTorque;
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
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoCoefficients = indexHandler.getRhoCoefficientsInSegment(0);
      int comCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment;

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
         inputCalculator.compute(command);

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
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

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
         inputCalculator.compute(command);

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

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

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

         inputCalculator.compute(command);

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

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

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
         inputCalculator.compute(command);

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
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravityZ);

      indexHandler.initialize(contactProviders, orientationPreviewWindowLength);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

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
         inputCalculator.compute(command);

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
                                                           OrientationDynamicsCalculator inputCalculator,
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
                                                                   OrientationDynamicsCalculator inputCalculator,
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
                                                      OrientationDynamicsCalculator inputCalculator,
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
                                           OrientationDynamicsCalculator inputCalculator,
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
                                           OrientationDynamicsCalculator inputCalculator,
                                           DiscreteAngularVelocityOrientationCommand command)
   {
      assertRateFromAngularErrorIsCorrect(angularErrorAtCurrentTick, inputCalculator, command);
      assertRateFromAngularVelocityErrorIsCorrect(angularVelocityErrorAtCurrentTick, inputCalculator, command);
      assertRateFromContactIsCorrect(comPosition, trajectoryCoefficients, inputCalculator, command);
      assertRateIsCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);
   }

   private static FrameVector3DReadOnly computeActualAngularErrorRate(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                      FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                      DMatrixRMaj trajectoryCoefficients,
                                                                      OrientationDynamicsCalculator inputCalculator)
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
                                                                              OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRate = new FrameVector3D();
      actualAngularVelocityErrorRate.set(3, computeActualRateVector(angularErrorAtCurrentTick,
                                                                    angularVelocityErrorAtCurrentTick,
                                                                    trajectoryCoefficients,
                                                                    inputCalculator));

      return actualAngularVelocityErrorRate;
   }

   private static FrameVector3DReadOnly computeActualAngularErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                      OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();
      angularErrorRateFromAngularError.set(computeActualRateVectorFromAngularError(angularErrorAtCurrentTick, inputCalculator));

      return angularErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeActualAngularVelocityErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                              OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRateFromAngularError = new FrameVector3D();
      actualAngularVelocityErrorRateFromAngularError.set(3, computeActualRateVectorFromAngularError(angularErrorAtCurrentTick, inputCalculator));

      return actualAngularVelocityErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeActualAngularErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                              OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();
      angularErrorRateFromAngularError.set(computeActualRateVectorFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator));

      return angularErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeActualAngularVelocityErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                                      OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRateFromAngularError = new FrameVector3D();
      actualAngularVelocityErrorRateFromAngularError.set(3, computeActualRateVectorFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator));

      return actualAngularVelocityErrorRateFromAngularError;
   }

   private static FrameVector3DReadOnly computeActualAngularErrorRateFromContact(DMatrixRMaj trajectoryCoefficients,
                                                                                 OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D angularErrorRateFromContact = new FrameVector3D();
      angularErrorRateFromContact.set(computeActualRateVectorFromContact(trajectoryCoefficients, inputCalculator));

      return angularErrorRateFromContact;
   }

   private static FrameVector3DReadOnly computeActualAngularVelocityErrorRateFromContact(DMatrixRMaj trajectoryCoefficients,
                                                                                         OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRateFromContact = new FrameVector3D();
      actualAngularVelocityErrorRateFromContact.set(3, computeActualRateVectorFromContact(trajectoryCoefficients, inputCalculator));

      return actualAngularVelocityErrorRateFromContact;
   }

   private static DMatrixRMaj computeActualRateVector(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                      FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                      DMatrixRMaj trajectoryCoefficients,
                                                      OrientationDynamicsCalculator inputCalculator)
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
                                                                      OrientationDynamicsCalculator inputCalculator)
   {
      DMatrixRMaj angleErrorStateVector = new DMatrixRMaj(6, 1);
      DMatrixRMaj rateFromAngleError = new DMatrixRMaj(6, 1);

      angularErrorAtCurrentTick.get(angleErrorStateVector);

      CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), angleErrorStateVector, rateFromAngleError);

      return rateFromAngleError;
   }

   private static DMatrixRMaj computeActualRateVectorFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                              OrientationDynamicsCalculator inputCalculator)
   {
      DMatrixRMaj velocityErrorStateVector = new DMatrixRMaj(6, 1);
      DMatrixRMaj rateFromAngularVelocityError = new DMatrixRMaj(6, 1);

      angularVelocityErrorAtCurrentTick.get(3, velocityErrorStateVector);

      CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), velocityErrorStateVector, rateFromAngularVelocityError);

      return rateFromAngularVelocityError;
   }


   private static DMatrixRMaj computeActualRateVectorFromContact(DMatrixRMaj trajectoryCoefficients,
                                                                 OrientationDynamicsCalculator inputCalculator)
   {
      DMatrixRMaj rateFromContact = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateFromContact);
      CommonOps_DDRM.addEquals(rateFromContact, inputCalculator.getContinuousCMatrix());

      return rateFromContact;
   }

}
