package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.PreviewWindowSegment;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class OrientationDynamicCalculatorTest
{
   private static final double gravityZ = OrientationDynamicsHelper.gravityZ;
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

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         contactPlane.computeContactForce(omega, time);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients), contactPlane);
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

         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(contactPlane),
                                 time,
                                 tickDuration,
                                 omega);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         OrientationDynamicsHelper.assertAllRatesAreCorrect(mass, comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator,
                                                            momentOfInertia,
                                                            desiredCoMPosition,
                                                            desiredCoMAcceleration,
                                                            desiredBodyOrientation,
                                                            desiredBodyAngularVelocity,
                                                            desiredNetAngularMomentumRate,
                                                            desiredInternalAngularMomentumRate,
                                                            toList(contactPlane));
      }
   }

   @Test
   public void testCoMObjectiveOneSegmentTwoContacts()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
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

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoCoefficients = indexHandler.getRhoCoefficientsInSegment(0);
      int comCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         leftContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         rightContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) + leftContactPlane.getCoefficientSize() - SE3MPCIndexHandler.variablesPerOrientationTick);
         leftContactPlane.computeContactForce(omega, time);
         rightContactPlane.computeContactForce(omega, time);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients), leftContactPlane, rightContactPlane);
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

         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(leftContactPlane, rightContactPlane),
                                 time,
                                 tickDuration,
                                 omega);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         OrientationDynamicsHelper.assertAllRatesAreCorrect(mass,
                                                            comPosition,
                                                            angularErrorAtCurrentTick,
                                                            angularVelocityErrorAtCurrentTick,
                                                            trajectoryCoefficients,
                                                            inputCalculator,
                                                            momentOfInertia,
                                                            desiredCoMPosition,
                                                            desiredCoMAcceleration,
                                                            desiredBodyOrientation,
                                                            desiredBodyAngularVelocity,
                                                            desiredNetAngularMomentumRate,
                                                            desiredInternalAngularMomentumRate,
                                                            toList(leftContactPlane, rightContactPlane));
      }
   }



   @Test
   public void testObjectiveFormulation()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         contactPlane.computeContactForce(omega, time);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients), contactPlane);
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

         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(contactPlane),
                                 time,
                                 tickDuration,
                                 omega);

         DMatrixRMaj gravityMatrix = new DMatrixRMaj(3, 1);
         gravityVector.get(gravityMatrix);


         DMatrixRMaj expectedA = new DMatrixRMaj(6, 6);
         NativeMatrix expectedB = new NativeMatrix(6, numberOfTrajectoryCoefficients);
         DMatrixRMaj expectedC = new DMatrixRMaj(6, 1);

         DMatrixRMaj skewDesiredAngular = new DMatrixRMaj(3, 3);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredBodyAngularVelocity, skewDesiredAngular);
         CommonOps_DDRM.scale(-1.0, skewDesiredAngular);

         CommonOps_DDRM.insert(skewDesiredAngular, expectedA, 0, 0);
         CommonOps_DDRM.insert(CommonOps_DDRM.identity(3), expectedA, 0, 3);
         CommonOps_DDRM.insert(inputCalculator.getB3(), expectedA, 3, 0);
         CommonOps_DDRM.insert(inputCalculator.getB4(), expectedA, 3, 3);


         expectedB.multAddBlock(new NativeMatrix(inputCalculator.getB1()), MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlane), 3, 0);
         MatrixTools.addMatrixBlock(expectedB, 3, 6, inputCalculator.getB2(), 0, 0, 3, rhoCoefficients, 1.0 );

         MatrixTools.setMatrixBlock(expectedC, 3, 0, inputCalculator.getB0(), 0, 0, 3, 1, 1.0);
         MatrixTools.multAddBlock(0.5 * time * time, inputCalculator.getB1(), gravityMatrix, expectedC, 3, 0);

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

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoCoefficients = 0;
      int comCoefficients = 6;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients));

         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
         desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

         //
         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 new ArrayList<>(),
                                 time,
                                 tickDuration,
                                 omega);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         OrientationDynamicsHelper.assertAllRatesAreCorrect(mass,
                                                            comPosition,
                                                            angularErrorAtCurrentTick,
                                                            angularVelocityErrorAtCurrentTick,
                                                            trajectoryCoefficients,
                                                            inputCalculator,
                                                            momentOfInertia,
                                                            desiredCoMPosition,
                                                            desiredCoMAcceleration,
                                                            desiredBodyOrientation,
                                                            desiredBodyAngularVelocity,
                                                            desiredNetAngularMomentumRate,
                                                            desiredInternalAngularMomentumRate,
                                                            new ArrayList<>());
      }
   }

   @Disabled
   @Test
   public void testCoMObjectiveOneSegmentTwoContactsAgainstGroundTruth()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
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

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoCoefficients = indexHandler.getRhoCoefficientsInSegment(0);
      int comCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));
         // FIGUREO UT HWO TO ENSURE IT'S HIGH ENOUGH

         leftContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0));
         rightContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) + leftContactPlane.getCoefficientSize());
         leftContactPlane.computeContactForce(omega, time);
         rightContactPlane.computeContactForce(omega, time);

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients), leftContactPlane, rightContactPlane);
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

         //
         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(leftContactPlane, rightContactPlane),
                                 time,
                                 tickDuration,
                                 omega);

         FrameVector3D angularErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D angularVelocityErrorAtCurrentTick = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         angularErrorAtCurrentTick.normalize();
         angularErrorAtCurrentTick.scale(0.1);
         angularVelocityErrorAtCurrentTick.normalize();
         angularVelocityErrorAtCurrentTick.scale(0.1);

//         assertAllRatesAreCorrect(comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);

         FrameQuaternion actualOrientation = new FrameQuaternion(desiredBodyOrientation);
         AxisAngle rotationErrorVector = new AxisAngle();
         rotationErrorVector.setRotationVector(angularErrorAtCurrentTick);
         actualOrientation.append(rotationErrorVector);

         FrameVector3D actualVelocity = new FrameVector3D(desiredBodyAngularVelocity);
         actualVelocity.add(angularVelocityErrorAtCurrentTick);

         FrameVector3DReadOnly netCoMTorque = getNetCoMTorque(comPosition, leftContactPlane, rightContactPlane);
         FrameVector3D rotatedNetTorque = new FrameVector3D(netCoMTorque);
         desiredBodyOrientation.inverseTransform(rotatedNetTorque);

         FrameVector3D desiredAngularAcceleration = new FrameVector3D();
         FrameVector3D desiredCoriolis = new FrameVector3D();
         FrameVector3D tempVector = new FrameVector3D();
         momentOfInertia.transform(desiredBodyAngularVelocity, tempVector);
         desiredCoriolis.cross(desiredBodyAngularVelocity, tempVector);
         FrameVector3D rotatedInternalMomentum = new FrameVector3D();
         desiredBodyOrientation.inverseTransform(desiredInternalAngularMomentumRate, rotatedInternalMomentum);

         desiredAngularAcceleration.sub(rotatedNetTorque, desiredCoriolis);
         desiredAngularAcceleration.sub(rotatedInternalMomentum);
         momentOfInertia.inverseTransform(desiredAngularAcceleration);

         FrameVector3D actualAngularAcceleration = new FrameVector3D();
         actualOrientation.inverseTransform(netCoMTorque, rotatedNetTorque);
         FrameVector3D actualCoriolis = new FrameVector3D();
         momentOfInertia.transform(actualVelocity, tempVector);
         actualCoriolis.cross(actualVelocity, tempVector);
         actualOrientation.inverseTransform(desiredInternalAngularMomentumRate, rotatedInternalMomentum);

         actualAngularAcceleration.sub(rotatedNetTorque, actualCoriolis);
         actualAngularAcceleration.sub(rotatedInternalMomentum);
         momentOfInertia.inverseTransform(actualAngularAcceleration);

         FrameVector3D angularAccelerationDeviation = new FrameVector3D();
         angularAccelerationDeviation.sub(actualAngularAcceleration, desiredAngularAcceleration);

         FrameVector3DReadOnly angularVelocityErrorRate = OrientationDynamicsHelper.computeActualAngularVelocityErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator);

         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Failed at tick " + nextTickId, angularAccelerationDeviation, angularVelocityErrorRate, 1e-4);
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

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
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

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoCoefficients = indexHandler.getRhoCoefficientsInSegment(0);
      int comCoefficients = LinearMPCIndexHandler.comCoefficientsPerSegment;

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         contactPlane.computeContactForce(omega, time);

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients), contactPlane);

         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentumRate = new FrameVector3D(desiredBodyAngularMomentumRate);
         desiredInternalAngularMomentumRate.scale(-1.0);
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
         desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         //
         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(contactPlane),
                                 time,
                                 tickDuration,
                                 omega);

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

         NativeMatrix scaledGravity = new NativeMatrix(3, 1);
         scaledGravity.set(2, 0, 0.5 * time * time * gravityZ);

         NativeMatrix expectedBMatrix = new NativeMatrix(6, numberOfTrajectoryCoefficients);
         NativeMatrix expectedCMatrix = new NativeMatrix(6, 1);
         expectedBMatrix.multAddBlock(new NativeMatrix(inputCalculator.getB1()), MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlane), 3, 0);
         expectedCMatrix.multAddBlock(new NativeMatrix(inputCalculator.getB1()), scaledGravity, 3, 0);

         MatrixTools.setMatrixBlock(velocityErrorRateFromContact, 0, 0, rateFromContact, 3, 0, 3, 1, 1.0);

         DMatrixRMaj expectedVelocityErrorRateFromContact = new DMatrixRMaj(3, 1);

         angularVelocityErrorRateFromContact.get(expectedVelocityErrorRateFromContact);

         MatrixTestTools.assertMatrixEquals(new DMatrixRMaj(3, 1), inputCalculator.getB0(), 1e-5);
         MatrixTestTools.assertMatrixEquals(new DMatrixRMaj(3,  LinearMPCIndexHandler.coefficientsPerRho * 4 * contactPolygon.getNumberOfVertices() ), inputCalculator.getB2(), 1e-5);

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

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         contactPlane.computeContactForce(omega, time);

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients), contactPlane);
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

         //
         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(contactPlane),
                                 time,
                                 tickDuration,
                                 omega);

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

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         DMatrixRMaj contactCoefficients = new DMatrixRMaj(rhoCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));
         MatrixTools.setMatrixBlock(contactCoefficients, 0, 0, trajectoryCoefficients, LinearMPCIndexHandler.comCoefficientsPerSegment, 0, rhoCoefficients, 1, 1.0);

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         contactPlane.computeContactForce(omega, time);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
         desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(contactPlane),
                                 time,
                                 tickDuration,
                                 omega);

         FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();

         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);
            FrameVector3D relativeContactPointLocation = new FrameVector3D();
            relativeContactPointLocation.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);

            FrameVector3D torqueFromContact = new FrameVector3D();
            torqueFromContact.cross(relativeContactPointLocation, contactPoint.getContactAcceleration());
            torqueFromContact.scale(mass);

            angularVelocityErrorRateFromContact.add(torqueFromContact);
         }

         desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

         DMatrixRMaj rate = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rate);

         DMatrixRMaj expectedAngularRateErrorRateFromContact = new DMatrixRMaj(3, 1);
         angularVelocityErrorRateFromContact.get(expectedAngularRateErrorRateFromContact);

         DMatrixRMaj angularRateErrorRateFromContact = new DMatrixRMaj(3, 1);
         MatrixTools.setMatrixBlock(angularRateErrorRateFromContact, 0, 0, rate, 3, 0, 3, 1, 1.0);

         DMatrixRMaj easyExpectedAngularAcceleration = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(inputCalculator.getB2(), contactCoefficients, easyExpectedAngularAcceleration);
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

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         DMatrixRMaj contactCoefficients = new DMatrixRMaj(rhoCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));
         MatrixTools.setMatrixBlock(contactCoefficients, 0, 0, trajectoryCoefficients, LinearMPCIndexHandler.comCoefficientsPerSegment, 0, rhoCoefficients, 1, 1.0);

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         contactPlane.computeContactForce(omega, time);

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients), contactPlane);
         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         comPosition.get(comPositionVector);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
         desiredNetAngularMomentumRate.add(desiredBodyAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         //
         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(contactPlane),
                                 time,
                                 tickDuration,
                                 omega);

         FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();

         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);

            FrameVector3D torqueFromContact = new FrameVector3D();

            FrameVector3D desiredMomentArm = new FrameVector3D();
            desiredMomentArm.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);

            torqueFromContact.cross(desiredMomentArm, contactPoint.getContactAcceleration());
            torqueFromContact.scale(mass);

            angularVelocityErrorRateFromContact.add(torqueFromContact);
         }

         desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

         DMatrixRMaj rateFromContact = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(inputCalculator.getB2(), contactCoefficients, rateFromContact);

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

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         contactPlane.computeContactForce(omega, time);

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients), contactPlane);
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

         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(contactPlane),
                                 time,
                                 tickDuration,
                                 omega);

         FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();

         FrameVector3D desiredContactPointForce = new FrameVector3D(desiredCoMAcceleration);
         desiredContactPointForce.sub(gravityVector);
         desiredContactPointForce.scale(mass);

         angularVelocityErrorRateFromContact.cross(desiredCoMPosition, desiredContactPointForce);
         angularVelocityErrorRateFromContact.sub(desiredNetAngularMomentumRate);

         desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
         momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

         DMatrixRMaj expectedRateFromContact = new DMatrixRMaj(3, 1);
         angularVelocityErrorRateFromContact.get(expectedRateFromContact);

         MatrixTestTools.assertMatrixEquals(expectedRateFromContact, inputCalculator.getB0(), 1e-6);
      }
   }


   @Test
   public void testContactJacobians()
   {
      double orientationPreviewWindowLength = 0.75;
      double tickDuration = 0.1;

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

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

      PreviewWindowSegment segment = new PreviewWindowSegment();
      segment.set(contact);

      contactProviders.add(segment);

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      OrientationDynamicsCalculator inputCalculator = new OrientationDynamicsCalculator(mass, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 8 * rhoSize;
      int comCoefficients = 6;

      Random random = new Random(1738L);

      for (double time = 0.0; time < orientationPreviewWindowLength; time += 0.01)
      {
         int numberOfTrajectoryCoefficients = rhoCoefficients + comCoefficients;
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         leftContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) - SE3MPCIndexHandler.variablesPerOrientationTick);
         rightContactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients, indexHandler.getRhoCoefficientStartIndex(0) + leftContactPlane.getCoefficientSize() - SE3MPCIndexHandler.variablesPerOrientationTick);
         leftContactPlane.computeContactForce(omega, time);
         rightContactPlane.computeContactForce(omega, time);

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time, omega, gravityZ, new NativeMatrix(trajectoryCoefficients), leftContactPlane, rightContactPlane);
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

         //
         inputCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);
         inputCalculator.compute(desiredCoMPosition,
                                 desiredCoMAcceleration,
                                 desiredBodyOrientation,
                                 desiredBodyAngularVelocity,
                                 desiredNetAngularMomentumRate,
                                 desiredInternalAngularMomentumRate,
                                 toList(leftContactPlane, rightContactPlane),
                                 time,
                                 tickDuration,
                                 omega);


         FrameVector3DReadOnly expectedNetTorque = getNetCoMTorque(desiredCoMPosition, leftContactPlane, rightContactPlane);

         DMatrixRMaj contactCoefficients = new DMatrixRMaj(rhoCoefficients, 1);
         MatrixTools.setMatrixBlock(contactCoefficients, 0, 0, trajectoryCoefficients, LinearMPCIndexHandler.comCoefficientsPerSegment, 0, rhoCoefficients, 1, 1.0);

         DMatrixRMaj torque = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(inputCalculator.contactOriginTorqueJacobian, contactCoefficients, torque);
         FrameVector3D netTorque = new FrameVector3D();
         netTorque.set(torque);

         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(expectedNetTorque, netTorque, 1e-5);
      }
   }

   private static <T> List<T> toList(T... items)
   {
      return Arrays.asList(items);
   }
}
