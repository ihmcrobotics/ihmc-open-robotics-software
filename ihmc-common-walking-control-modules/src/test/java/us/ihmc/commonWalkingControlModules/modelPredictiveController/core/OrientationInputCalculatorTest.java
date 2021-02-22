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
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class OrientationInputCalculatorTest
{
   @Test
   public void testCoMObjectiveOneSegment()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
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

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

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

         DMatrixRMaj comPositionVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj comVelocityVector = new DMatrixRMaj(3, 1);

         CommonOps_DDRM.mult(MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlaneHelper), trajectoryCoefficients, comPositionVector);
         CommonOps_DDRM.mult(MPCTestHelper.getCoMVelocityJacobian(time, omega, contactPlaneHelper), trajectoryCoefficients, comVelocityVector);

         FramePoint3D comPosition = new FramePoint3D();
         FrameVector3D comVelocity = new FrameVector3D();
         comPosition.set(comPositionVector);
         comVelocity.set(comVelocityVector);

         FrameVector3D netTorqueFromContact = new FrameVector3D();
         contactPlaneHelper.computeContactForce(omega, time);

         for (int i = 0; i < contactPlaneHelper.getNumberOfContactPoints(); i++)
         {
            MPCContactPoint contactPoint = contactPlaneHelper.getContactPointHelper(i);
            FrameVector3D contactPointTorque = new FrameVector3D();
            FrameVector3D contactForce = new FrameVector3D();
            contactForce.set(contactPoint.getContactAcceleration());
            contactForce.scale(mass);

            contactPointTorque.cross(contactPoint.getBasisVectorOrigin(), contactForce);
            netTorqueFromContact.add(contactPointTorque);
         }

         FrameVector3D expectedTorqueAboutBodyInWorldAtCurrentInstant = new FrameVector3D();
         expectedTorqueAboutBodyInWorldAtCurrentInstant.cross(gravityVector, comPosition);
         expectedTorqueAboutBodyInWorldAtCurrentInstant.scale(-mass);
         expectedTorqueAboutBodyInWorldAtCurrentInstant.add(netTorqueFromContact);

         FrameVector3D angularMomentumAboutBodyInWorldAtCurrentInstant = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D axisAngleErrorAtCurrentInstant = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D expectedAngularMomentumAboutBodyInWorldAtNextTick = new FrameVector3D();
         FrameVector3D axisAngleErrorAtNextTick = new FrameVector3D();

         expectedAngularMomentumAboutBodyInWorldAtNextTick.scaleAdd(tickDuration, expectedTorqueAboutBodyInWorldAtCurrentInstant, angularMomentumAboutBodyInWorldAtCurrentInstant);

         DMatrixRMaj fullSolutionVector = new DMatrixRMaj(numberOfTrajectoryCoefficients + 6 * indexHandler.getTotalNumberOfOrientationTicks(), 1);

         int nextTickId = RandomNumbers.nextInt(random, 1, indexHandler.getTotalNumberOfOrientationTicks() - 1);

         MatrixTools.setMatrixBlock(fullSolutionVector, 0, 0, trajectoryCoefficients, 0, 0, trajectoryCoefficients.getNumRows(), 1, 1.0);

         FrameQuaternion desiredBodyOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredNetAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredBodyAngularMomentum = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredInternalAngularMomentum = new FrameVector3D();
         desiredInternalAngularMomentum.sub(desiredNetAngularMomentum, desiredBodyAngularMomentum);
         FrameVector3D desiredBodyAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D desiredCoMVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D desiredCoMPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         Matrix3D skewDesiredCoMPosition = new Matrix3D();
         Matrix3D skewDesiredCoMVelocity = new Matrix3D();

         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, skewDesiredCoMPosition);
         MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, skewDesiredCoMVelocity);

         FrameVector3D axisAngleRateAtCurrentInstant = new FrameVector3D();
         axisAngleRateAtCurrentInstant.cross(desiredBodyAngularVelocity, axisAngleErrorAtCurrentInstant);

         FrameVector3D tempVector = new FrameVector3D();
         desiredBodyOrientation.inverseTransform(desiredBodyAngularMomentum);
         tempVector.cross(desiredBodyAngularMomentum, axisAngleErrorAtCurrentInstant);
         momentOfInertia.inverseTransform(tempVector);
         axisAngleRateAtCurrentInstant.add(tempVector);

         desiredBodyOrientation.inverseTransform(angularMomentumAboutBodyInWorldAtCurrentInstant, tempVector);
         momentOfInertia.inverseTransform(tempVector);
         axisAngleRateAtCurrentInstant.add(tempVector);

         FrameVector3D tempVector2 = new FrameVector3D();
         FrameVector3D tempVector3 = new FrameVector3D();
         tempVector.cross(desiredCoMPosition, comVelocity);
         tempVector2.cross(desiredCoMVelocity, comPosition);
         tempVector3.cross(desiredCoMPosition, desiredCoMVelocity);
         tempVector.sub(tempVector2);
         tempVector3.sub(tempVector3);
         tempVector.scale(-mass);
         desiredBodyOrientation.inverseTransform(tempVector);
         momentOfInertia.inverseTransform(tempVector);
         axisAngleRateAtCurrentInstant.add(tempVector);

         momentOfInertia.inverseTransform(desiredBodyAngularMomentum, tempVector);
         tempVector.scale(-1.0);
         axisAngleRateAtCurrentInstant.add(tempVector);


         axisAngleErrorAtNextTick.scaleAdd(tickDuration, axisAngleRateAtCurrentInstant, axisAngleErrorAtCurrentInstant);

         axisAngleErrorAtCurrentInstant.get(indexHandler.getOrientationTickStartIndex(nextTickId - 1), fullSolutionVector );
         angularMomentumAboutBodyInWorldAtCurrentInstant.get(indexHandler.getOrientationTickStartIndex(nextTickId - 1) + 3, fullSolutionVector );
         axisAngleErrorAtNextTick.get(indexHandler.getOrientationTickStartIndex(nextTickId), fullSolutionVector);
         expectedAngularMomentumAboutBodyInWorldAtNextTick.get(indexHandler.getOrientationTickStartIndex(nextTickId) + 3, fullSolutionVector);

         DiscreteOrientationCommand command = new DiscreteOrientationCommand();
         command.setDesiredCoMVelocity(desiredCoMVelocity);
         command.setDesiredCoMPosition(desiredCoMPosition);
         command.setMomentOfInertiaInBodyFrame(momentOfInertia);
         command.setDesiredBodyOrientation(desiredBodyOrientation);
         command.setTimeOfConstraint(time);
         command.setSegmentNumber(0);
         command.setEndingDiscreteTickId(nextTickId);
         command.setDurationOfHold(tickDuration);
         command.setOmega(omega);
         command.setDesiredNetAngularMomentum(new FrameVector3D());

         inputCalculator.compute(qpInput, command);



         FrameVector3D a0 = new FrameVector3D();
         a0.cross(desiredCoMVelocity, desiredCoMPosition);
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

         // TODO a4 Matrix
         DMatrixRMaj jacobian = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());
         MatrixTools.multAddBlock(a1Matrix, MPCTestHelper.getCoMPositionJacobian(time, omega, contactPlaneHelper), jacobian, 0, 0);
         MatrixTools.multAddBlock(a2Matrix, MPCTestHelper.getCoMVelocityJacobian(time, omega, contactPlaneHelper), jacobian, 0, 0);
         MatrixTools.addMatrixBlock(jacobian, 0, indexHandler.getOrientationTickStartIndex(nextTickId - 1) + 3, a3Matrix, 0, 0, 3, 3, 1.0);
         MatrixTools.addMatrixBlock(jacobian, 0, indexHandler.getOrientationTickStartIndex(nextTickId - 1) , a4Matrix, 0, 0, 3, 3, 1.0);

         CommonOps_DDRM.scale(-tickDuration, jacobian);
         MatrixTools.addMatrixBlock(jacobian, 0, indexHandler.getOrientationTickStartIndex(nextTickId) , CommonOps_DDRM.identity(3), 0, 0, 3, 3, 1.0);

         MatrixTestTools.assertMatrixEquals(jacobian, qpInput.getTaskJacobian(), 1e-6);

         DMatrixRMaj producedValue = new DMatrixRMaj(6, 1);

         CommonOps_DDRM.mult(qpInput.getTaskJacobian(), fullSolutionVector, producedValue);

         MatrixTestTools.assertMatrixEquals(qpInput.getTaskObjective(), producedValue, 1e-6);
      }
   }
}
