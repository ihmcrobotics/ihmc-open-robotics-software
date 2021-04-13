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
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class OrientationInputCalculatorTest
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
            OrientationDynamicsHelper.assertAllRatesAreCorrect(mass, comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator.dynamicsCalculator, command);

            MatrixTools.setMatrixBlock(Aeq, 6 * tick, 0, qpInput.getTaskJacobian(), 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
            MatrixTools.setMatrixBlock(beq, 6 * tick, 0, qpInput.getTaskObjective(), 0, 0, 6, 1, 1.0);

            FrameVector3DReadOnly expectedAngularErrorRate = OrientationDynamicsHelper.computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
            FrameVector3DReadOnly expectedAngularVelocityErrorRate = OrientationDynamicsHelper.computeExpectedAngularVelocityErrorRate(mass,
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

   /** This currently does nothing. **/
   @Disabled
   @Test
   public void testCoMObjectiveOneSegmentAgainstExactSolutionSimple()
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

      double time = 0.0;
      int tick = 0;
      int segmentNumber = 0;

      double tickDuration = indexHandler.getOrientationTickDuration(segmentNumber);
      int endTick = tick + indexHandler.getOrientationTicksInSegment(segmentNumber);

      leftContactPlane.computeContactForceCoefficientMatrix(reactionForceCoefficients, 0);
      rightContactPlane.computeContactForceCoefficientMatrix(reactionForceCoefficients, leftContactPlane.getCoefficientSize());


      FrameQuaternion desiredBodyOrientation = new FrameQuaternion();
      FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
      FrameVector3D desiredInternalAngularMomentumRate = new FrameVector3D();
      FrameVector3D desiredBodyAngularMomentumRate = new FrameVector3D();
      FrameVector3D desiredBodyAngularVelocity = new FrameVector3D();
      FramePoint3D desiredCoMPosition = new FramePoint3D();
      FrameVector3D desiredCoMAcceleration = new FrameVector3D();

      AxisAngle axisAngleError = new AxisAngle();
      axisAngleError.setRotationVector(initialAngularError);

      FrameQuaternionBasics initialActualOrientation = new FrameQuaternion(desiredBodyOrientation);
      initialActualOrientation.append(axisAngleError);
      FrameVector3D initialActualBodyAngularVelocity = new FrameVector3D();
      initialActualBodyAngularVelocity.add(desiredBodyAngularVelocity, initialAngularVelocityError);

      FrameQuaternion actualOrientationAtCurrentTick = new FrameQuaternion(initialActualOrientation);
      FrameVector3D actualBodyVelocityAtCurrentTick = new FrameVector3D(initialActualBodyAngularVelocity);

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

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time,
                                                                             omega,
                                                                             gravityZ,
                                                                             trajectoryCoefficients,
                                                                             leftContactPlane,
                                                                             rightContactPlane);
         FrameVector3DReadOnly netCoMTorque = getNetCoMTorque(comPosition, leftContactPlane, rightContactPlane);



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

         assertTaskIsCorrect(indexHandler,
                             angularErrorAtCurrentTick,
                             angularVelocityErrorAtCurrentTick,
                             trajectoryCoefficients,
                             inputCalculator,
                             command,
                             qpInput);
         OrientationDynamicsHelper.assertAllRatesAreCorrect(mass, comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator.dynamicsCalculator, command);

         FrameVector3DReadOnly expectedAngularErrorRate = OrientationDynamicsHelper.computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
         FrameVector3DReadOnly expectedAngularVelocityErrorRate = OrientationDynamicsHelper.computeExpectedAngularVelocityErrorRate(mass,
                                                                                                                                    comPosition,
                                                                                                                                    angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick,
                                                                                                                                    command);

         axisAngleError.setRotationVector(angularErrorAtCurrentTick);

         FrameQuaternion currentOrientation = new FrameQuaternion(desiredBodyOrientation);
         currentOrientation.append(axisAngleError);
         FrameVector3D currentBodyAngularVelocity = new FrameVector3D();
         currentBodyAngularVelocity.add(desiredBodyAngularVelocity, angularVelocityErrorAtCurrentTick);

         double epsilon = 1e-1;
         EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(actualOrientationAtCurrentTick, currentOrientation, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(actualBodyVelocityAtCurrentTick, currentBodyAngularVelocity, epsilon);

         FrameVector3D tempCoriolis = new FrameVector3D(actualBodyVelocityAtCurrentTick);
         momentOfInertia.transform(tempCoriolis);
         FrameVector3D coriolis = new FrameVector3D();
         coriolis.cross(actualBodyVelocityAtCurrentTick, tempCoriolis);

         FrameVector3D torqueInBody = new FrameVector3D();
         actualOrientationAtCurrentTick.inverseTransform(netCoMTorque, torqueInBody);

         FrameVector3D currentAngularVelocityRate = new FrameVector3D();
         currentAngularVelocityRate.sub(coriolis);
         currentAngularVelocityRate.add(torqueInBody);
         momentOfInertia.inverseTransform(currentAngularVelocityRate);

         FrameVector3D rotationVector = new FrameVector3D(actualBodyVelocityAtCurrentTick);
         rotationVector.scale(tickDuration);
         axisAngleError.setRotationVector(rotationVector);
         actualOrientationAtCurrentTick.append(axisAngleError);
         actualBodyVelocityAtCurrentTick.scaleAdd(tickDuration, currentAngularVelocityRate, actualBodyVelocityAtCurrentTick);

         angularErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularErrorRate, angularErrorAtCurrentTick);
         angularVelocityErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularVelocityErrorRate, angularVelocityErrorAtCurrentTick);

         time += tickDuration;
      }
   }

   /** This currently does nothing. **/
   @Disabled
   @Test
   public void testCoMObjectiveOneSegmentAgainstExactSolution()
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

         FramePoint3DReadOnly comPosition = MPCTestHelper.computeCoMPosition(time,
                                                                             omega,
                                                                             gravityZ,
                                                                             trajectoryCoefficients,
                                                                             leftContactPlane,
                                                                             rightContactPlane);
         FrameVector3DReadOnly netCoMTorque = getNetCoMTorque(comPosition, leftContactPlane, rightContactPlane);


         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D(netCoMTorque);
         FrameVector3D desiredInternalAngularMomentumRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentumRate = new FrameVector3D();
         desiredBodyAngularMomentumRate.sub(desiredNetAngularMomentumRate, desiredInternalAngularMomentumRate);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMAcceleration = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());


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

         assertTaskIsCorrect(indexHandler,
                             angularErrorAtCurrentTick,
                             angularVelocityErrorAtCurrentTick,
                             trajectoryCoefficients,
                             inputCalculator,
                             command,
                             qpInput);
         OrientationDynamicsHelper.assertAllRatesAreCorrect(mass, comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator.dynamicsCalculator, command);

         FrameVector3DReadOnly expectedAngularErrorRate = OrientationDynamicsHelper.computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
         FrameVector3DReadOnly expectedAngularVelocityErrorRate = OrientationDynamicsHelper.computeExpectedAngularVelocityErrorRate(mass,
                                                                                                                                    comPosition,
                                                                                                                                    angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick,
                                                                                                                                    command);

         FrameVector3D currentAngularVelocity = new FrameVector3D(desiredBodyAngularVelocity);
         currentAngularVelocity.add(angularVelocityErrorAtCurrentTick);

         AxisAngle axisAngleError = new AxisAngle();
         axisAngleError.setRotationVector(angularErrorAtCurrentTick);

         FrameQuaternionBasics currentOrientation = new FrameQuaternion(desiredBodyOrientation);
         currentOrientation.append(axisAngleError);

         FrameVector3D tempCoriolis = new FrameVector3D(currentAngularVelocity);
         momentOfInertia.transform(tempCoriolis);
         FrameVector3D coriolis = new FrameVector3D();
         coriolis.cross(currentAngularVelocity, tempCoriolis);

         FrameVector3D currentAngularVelocityRateA = new FrameVector3D();
         currentOrientation.inverseTransform(desiredBodyAngularMomentumRate, currentAngularVelocityRateA);
         currentAngularVelocityRateA.sub(coriolis);
         momentOfInertia.inverseTransform(currentAngularVelocityRateA);


         FrameVector3D torqueInBody = new FrameVector3D();
         currentOrientation.inverseTransform(netCoMTorque, torqueInBody);

         FrameVector3D currentAngularVelocityRateB = new FrameVector3D();
         currentOrientation.inverseTransform(desiredInternalAngularMomentumRate, currentAngularVelocityRateB);
         currentAngularVelocityRateB.scale(-1.0);
         currentAngularVelocityRateB.sub(coriolis);
         currentAngularVelocityRateB.add(torqueInBody);
         momentOfInertia.inverseTransform(currentAngularVelocityRateB);

         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(currentAngularVelocityRateA, currentAngularVelocityRateB, 1e-5);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(currentAngularVelocityRateA, expectedAngularVelocityErrorRate, 1e-5);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(currentAngularVelocityRateB, expectedAngularVelocityErrorRate, 1e-5);

         angularErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularErrorRate, angularErrorAtCurrentTick);
         angularVelocityErrorAtCurrentTick.scaleAdd(tickDuration, expectedAngularVelocityErrorRate, angularVelocityErrorAtCurrentTick);

         time += tickDuration;
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
         OrientationDynamicsHelper.assertAllRatesAreCorrect(mass, comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator.dynamicsCalculator, command);

         MatrixTools.setMatrixBlock(Aeq, 6 * tick, 0, qpInput.getTaskJacobian(), 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
         MatrixTools.setMatrixBlock(beq, 6 * tick, 0, qpInput.getTaskObjective(), 0, 0, 6, 1, 1.0);

         FrameVector3DReadOnly expectedAngularErrorRate = OrientationDynamicsHelper.computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
         FrameVector3DReadOnly expectedAngularVelocityErrorRate = OrientationDynamicsHelper.computeExpectedAngularVelocityErrorRate(mass,
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
         CommonOps_DDRM.mult(inputCalculator.dynamicsCalculator.contactForceJacobian, reactionForceCoefficients, reactionForceVector);
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
         OrientationDynamicsHelper.assertAllRatesAreCorrect(mass, comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator.dynamicsCalculator, command);

         FrameVector3DReadOnly expectedAngularErrorRate = OrientationDynamicsHelper.computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
         FrameVector3DReadOnly expectedAngularVelocityErrorRate = OrientationDynamicsHelper.computeExpectedAngularVelocityErrorRate(mass,
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
            OrientationDynamicsHelper.assertAllRatesAreCorrect(mass, comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator.dynamicsCalculator, command);

            MatrixTools.setMatrixBlock(Aeq, 6 * tick, 0, qpInput.getTaskJacobian(), 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
            MatrixTools.setMatrixBlock(beq, 6 * tick, 0, qpInput.getTaskObjective(), 0, 0, 6, 1, 1.0);

            FrameVector3DReadOnly expectedAngularErrorRate = OrientationDynamicsHelper.computeExpectedAngularErrorRate(angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, command);
            FrameVector3DReadOnly expectedAngularVelocityErrorRate = OrientationDynamicsHelper.computeExpectedAngularVelocityErrorRate(mass,
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
}
