package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.ProcessModelTest.TestBiasProvider;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.ProcessModelTest.TestIMU;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class MeasurementModelTest
{
   private static final double gravityZ = 9.81;
   private static final FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
   private static final double estimatorDt = 0.01;

   @Test
   public void testUpdate()
   {
      int iters = 100;
      Random random = new Random(1738L);

      YoRegistry test = new YoRegistry(getClass().getSimpleName());
      ProcessModelTest.TestIMU baseIMU = new ProcessModelTest.TestIMU("base");
      ProcessModelTest.TestIMU footIMU = new ProcessModelTest.TestIMU("foot");

      StateVariables baseState = new StateVariables("base", baseIMU.getMeasurementFrame(), test);
      StateVariables footState = new StateVariables("foot", footIMU.getMeasurementFrame(), test);
      SensedVariables baseSensing = new SensedVariables("baseSensed", baseIMU, baseIMU, new TestBiasProvider(), test);
      SensedVariables footSensing = new SensedVariables("footSensed", baseIMU, footIMU, new TestBiasProvider(), test);
      MeasurementVariables footMeasure = new MeasurementVariables("footMeasure", test);

      MeasurementModel measurementModel = new MeasurementModel("foot0", baseState, footState, footSensing, footMeasure,  gravity, () -> Double.MAX_VALUE, test);
      DMatrixRMaj jacobian = new DMatrixRMaj(OdometryIndexHelper.errorSizePerLink, OdometryIndexHelper.errorSizePerLink * 2);

      for (int iter = 0; iter < iters; iter++)
      {
         jacobian.zero();

         OdomTestTools.setRandomSensed(random, footSensing);
         footIMU.linearAcceleration.set(footSensing.accelMeasurement);
         footIMU.angularVelocity.set(footSensing.gyroMeasurement);
         footIMU.pose.set(EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextOrientation3D(random));
         footIMU.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 10.0));
         footIMU.sensorFrame.update();

         OdomTestTools.setRandomSensed(random, baseSensing);
         baseIMU.linearAcceleration.set(baseSensing.accelMeasurement);
         baseIMU.angularVelocity.set(baseSensing.gyroMeasurement);
         baseIMU.pose.set(EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextOrientation3D(random));
         baseIMU.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 10.0));
         baseIMU.sensorFrame.update();

         OdomTestTools.setRandomState(random, baseSensing, baseState);
         OdomTestTools.setRandomState(random, footSensing, footState);

         measurementModel.update(new DMatrixRMaj(3, 3));

         DMatrixRMaj baseTranslation = new DMatrixRMaj(3, 1);
         DMatrixRMaj footTranslation = new DMatrixRMaj(3, 1);
         DMatrixRMaj footRelativePosition = new DMatrixRMaj(3, 1);
         DMatrixRMaj baseLinearVelocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj footLinearVelocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj baseRotationMatrix = new DMatrixRMaj(3, 3);
         DMatrixRMaj footRotationMatrix = new DMatrixRMaj(3, 3);
         DMatrixRMaj baseAcceleration = new DMatrixRMaj(3, 1);
         DMatrixRMaj baseAccelerationBias = new DMatrixRMaj(3, 1);
         DMatrixRMaj footAccelerationBias = new DMatrixRMaj(3, 1);
         DMatrixRMaj baseGyro = new DMatrixRMaj(3, 1);
         DMatrixRMaj baseGyroBias = new DMatrixRMaj(3, 1);
         DMatrixRMaj baseGyroUnbiased = new DMatrixRMaj(3, 1);
         DMatrixRMaj footVelocity = new DMatrixRMaj(3, 1);

         DMatrixRMaj footOffset = new DMatrixRMaj(3, 1);
         DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);

         baseState.translation.get(baseTranslation);
         footState.translation.get(footTranslation);
         footSensing.positionMeasurement.get(footRelativePosition);
         baseState.linearVelocity.get(baseLinearVelocity);
         footState.linearVelocity.get(footLinearVelocity);
         OdometryTools.toRotationMatrix(baseState.orientation, baseRotationMatrix);
         OdometryTools.toRotationMatrix(footState.orientation, footRotationMatrix);
         gravity.get(gravityVector);
         baseSensing.accelMeasurement.get(baseAcceleration);
         baseState.accelBias.get(baseAccelerationBias);
         footState.accelBias.get(footAccelerationBias);
         baseSensing.gyroMeasurement.get(baseGyro);
         baseState.gyroBias.get(baseGyroBias);

         DMatrixRMaj expectedRelativeError = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedPredictedLinearVelocityError = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedPredictedRotationError = new DMatrixRMaj(4, 1);
         DMatrixRMaj expectedPredictedContactVelocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedPredictedContactAccel = new DMatrixRMaj(3, 1);

         // First term
         CommonOps_DDRM.subtract(footTranslation, baseTranslation, footOffset);
         CommonOps_DDRM.multTransA(baseRotationMatrix, footOffset, expectedRelativeError);

         // Second term
         DMatrixRMaj rotation = multiplyLeftInverse(footSensing.orientationMeasurement, OdomTestTools.fromVector(multiplyLeftInverse(baseState.orientation, footState.orientation)));
         expectedPredictedRotationError.set(OdometryTools.logMap(rotation));

         // Third term
         CommonOps_DDRM.subtract(baseLinearVelocity, footLinearVelocity, footVelocity);
         if (OdometryKalmanFilter.includeBias)
            CommonOps_DDRM.subtract(baseGyro, baseGyroBias, baseGyroUnbiased);
         else
            baseGyroUnbiased.set(baseGyro);
         OdometryTools.toSkewSymmetricMatrix(baseGyroUnbiased.get(0), baseGyroUnbiased.get(1), baseGyroUnbiased.get(2), skewMatrix);
         footSensing.linearVelocity.get(expectedPredictedLinearVelocityError);
         CommonOps_DDRM.multAdd(-1.0, skewMatrix, footRelativePosition, expectedPredictedLinearVelocityError);
         CommonOps_DDRM.multAddTransA(-1.0, baseRotationMatrix, footVelocity, expectedPredictedLinearVelocityError);

         // Fourth term
         expectedPredictedContactVelocity.set(footLinearVelocity);


         // Fifth term
         CommonOps_DDRM.multTransA(footRotationMatrix, gravityVector, expectedPredictedContactAccel);
         if (OdometryKalmanFilter.includeBias)
            CommonOps_DDRM.addEquals(expectedPredictedContactAccel, footAccelerationBias);

         DMatrixRMaj predictedRelativeError = new DMatrixRMaj(3, 1);
         DMatrixRMaj predictedRotationError = new DMatrixRMaj(3, 1);
         DMatrixRMaj predictedLinearVelocityError = new DMatrixRMaj(3, 1);
         DMatrixRMaj predictedContactVelocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj predictedContactAccel = new DMatrixRMaj(3, 1);

         footMeasure.relativePosition.get(predictedRelativeError);
         footMeasure.relativeOrientationError.get(predictedRotationError);
         footMeasure.relativeLinearVelocityError.get(predictedLinearVelocityError);
         footMeasure.contactVelocity.get(predictedContactVelocity);
         footMeasure.accelMeasure.get(predictedContactAccel);

         EjmlUnitTests.assertEquals(expectedRelativeError, predictedRelativeError, 1e-6);
         EjmlUnitTests.assertEquals(expectedPredictedRotationError, predictedRotationError, 1e-6);
         EjmlUnitTests.assertEquals(expectedPredictedLinearVelocityError, predictedLinearVelocityError, 1e-6);
         EjmlUnitTests.assertEquals(expectedPredictedContactVelocity, predictedContactVelocity, 1e-6);
         EjmlUnitTests.assertEquals(expectedPredictedContactAccel, predictedContactAccel, 1e-6);
      }
   }


   @Test
   public void testComputeMeasurementJacobian()
   {
      int iters = 100;
      Random random = new Random(1738L);

      YoRegistry test = new YoRegistry(getClass().getSimpleName());
      TestIMU baseIMU = new TestIMU("base");
      TestIMU footIMU = new TestIMU("foot");

      StateVariables baseState = new StateVariables("base", baseIMU.getMeasurementFrame(), test);
      StateVariables footState = new StateVariables("foot", footIMU.getMeasurementFrame(), test);
      SensedVariables baseSensing = new SensedVariables("baseSensed", baseIMU, baseIMU, new TestBiasProvider(), test);
      SensedVariables footSensing = new SensedVariables("footSensed", baseIMU, footIMU, new TestBiasProvider(), test);
      MeasurementVariables measurement = new MeasurementVariables("footMeasure", test);

      MeasurementModel measurementModel = new MeasurementModel("foot0", baseState, footState, footSensing, measurement,  gravity, () -> Double.MAX_VALUE, test);
      DMatrixRMaj jacobian = new DMatrixRMaj(OdometryIndexHelper.errorSizePerLink, OdometryIndexHelper.errorSizePerLink * 2);

      for (int iter = 0; iter < iters; iter++)
      {
         jacobian.zero();

         measurementModel.isInContact.set(true);

         OdomTestTools.setRandomSensed(random, footSensing);
         footIMU.linearAcceleration.set(footSensing.accelMeasurement);
         footIMU.angularVelocity.set(footSensing.gyroMeasurement);
         footIMU.pose.set(EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextOrientation3D(random));
         footIMU.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 10.0));
         footIMU.sensorFrame.update();

         OdomTestTools.setRandomSensed(random, baseSensing);
         baseIMU.linearAcceleration.set(baseSensing.accelMeasurement);
         baseIMU.angularVelocity.set(baseSensing.gyroMeasurement);
         baseIMU.pose.set(EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextOrientation3D(random));
         baseIMU.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 10.0));
         baseIMU.sensorFrame.update();

         OdomTestTools.setRandomState(random, baseSensing, baseState);
         OdomTestTools.setRandomState(random, footSensing, footState);

         measurementModel.computeBaseMeasurementJacobian( 0, jacobian);
         measurementModel.computeFootMeasurementJacobian( 0, OdometryIndexHelper.errorSizePerLink, jacobian);

         DMatrixRMaj jacobianExpected = new DMatrixRMaj(OdometryIndexHelper.errorSizePerLink, 2 * OdometryIndexHelper.errorSizePerLink);
         DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);
         DMatrixRMaj baseRotationMatrix = new DMatrixRMaj(3, 3);
         DMatrixRMaj baseRotationMatrixTranspose = new DMatrixRMaj(3, 3);
         DMatrixRMaj footRotationMatrix = new DMatrixRMaj(3, 3);
         DMatrixRMaj footRotationMatrixTranspose = new DMatrixRMaj(3, 3);
         OdometryTools.toRotationMatrix(baseState.orientation, baseRotationMatrix);
         OdometryTools.toRotationMatrix(footState.orientation, footRotationMatrix);
         CommonOps_DDRM.transpose(baseRotationMatrix, baseRotationMatrixTranspose);
         CommonOps_DDRM.transpose(footRotationMatrix, footRotationMatrixTranspose);

         // Row 1, base
         MatrixTools.setMatrixBlock(jacobianExpected, 0, 0, baseRotationMatrixTranspose, 0, 0, 3, 3,  -1.0);
         DMatrixRMaj footPosition = new DMatrixRMaj(3, 1);
         DMatrixRMaj basePosition = new DMatrixRMaj(3, 1);
         DMatrixRMaj footOffset = new DMatrixRMaj(3, 1);
         DMatrixRMaj footOffsetInBaseFrame = new DMatrixRMaj(3, 1);
         footState.translation.get(footPosition);
         baseState.translation.get(basePosition);
         CommonOps_DDRM.subtract(footPosition, basePosition, footOffset);
         CommonOps_DDRM.multTransA(baseRotationMatrix, footOffset, footOffsetInBaseFrame);
         OdometryTools.toSkewSymmetricMatrix(footOffsetInBaseFrame.get(0), footOffsetInBaseFrame.get(1), footOffsetInBaseFrame.get(2), skewMatrix);
         CommonOps_DDRM.insert(skewMatrix, jacobianExpected, 0, 6);

         // Row 1, foot
         CommonOps_DDRM.insert(baseRotationMatrixTranspose, jacobianExpected, 0, 15);

         // Row 2, base
         DMatrixRMaj left = OdometryTools.lOperator(multiplyLeftInverse(footState.orientation, baseState.orientation));
         DMatrixRMaj right = OdometryTools.rOperator(footSensing.orientationMeasurement);
         DMatrixRMaj block = new DMatrixRMaj(4, 4);
         CommonOps_DDRM.mult(left, right, block);
         MatrixTools.setMatrixBlock(jacobianExpected, 3, 6, block, 1, 1, 3, 3, -1.0);

         // Row 2, foot
         block = OdometryTools.lOperator(multiply(footSensing.orientationMeasurement, OdomTestTools.fromVector(multiplyLeftInverse(baseState.orientation, footState.orientation))));
         MatrixTools.setMatrixBlock(jacobianExpected, 3, 21, block, 1, 1, 3, 3, 1.0);

         // Row 3, base
         MatrixTools.setMatrixBlock(jacobianExpected, 6, 3, baseRotationMatrixTranspose, 0, 0, 3, 3, -1.0);
         DMatrixRMaj footVelocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj baseVelocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj velocityError = new DMatrixRMaj(3, 1);
         DMatrixRMaj velocityErrorInBaseFrame = new DMatrixRMaj(3, 1);
         footState.linearVelocity.get(footVelocity);
         baseState.linearVelocity.get(baseVelocity);
         CommonOps_DDRM.subtract(footVelocity, baseVelocity, velocityError);
         CommonOps_DDRM.multTransA(baseRotationMatrix, velocityError, velocityErrorInBaseFrame);
         OdometryTools.toSkewSymmetricMatrix(velocityErrorInBaseFrame.get(0), velocityErrorInBaseFrame.get(1), velocityErrorInBaseFrame.get(2), skewMatrix);
         CommonOps_DDRM.insert(skewMatrix, jacobianExpected, 6, 6);
         if (OdometryKalmanFilter.includeBias)
         {
            OdometryTools.toSkewSymmetricMatrix(footSensing.positionMeasurement, skewMatrix);
            MatrixTools.setMatrixBlock(jacobianExpected, 6, 12, skewMatrix, 0, 0, 3, 3, -1.0);
         }

         // Row 3, foot
         CommonOps_DDRM.insert(baseRotationMatrixTranspose, jacobianExpected, 6, 18);

         // Row 4, foot
         if (OdometryKalmanFilter.includeBias)
         {
            for (int i = 0; i < 3; i++)
               jacobianExpected.set(9 + i, 18 + i, 1.0);
         }

         // Row 5, foot
         if (OdometryKalmanFilter.includeBias)
         {
            DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
            DMatrixRMaj rotatedGravityVector = new DMatrixRMaj(3, 1);
            gravity.get(gravityVector);
            CommonOps_DDRM.multTransA(footRotationMatrix, gravityVector, rotatedGravityVector);
            OdometryTools.toSkewSymmetricMatrix(rotatedGravityVector.get(0), rotatedGravityVector.get(1), rotatedGravityVector.get(2), skewMatrix);
            CommonOps_DDRM.insert(skewMatrix, jacobianExpected, 12, 21);

            for (int i = 0; i < 3; i++)
               jacobianExpected.set(12 + i, 24 + i, 1.0);
         }

         EjmlUnitTests.assertEquals(jacobianExpected, jacobian, 1e-5);
      }
   }

   static DMatrixRMaj multiply(QuaternionReadOnly a, QuaternionReadOnly b)
   {
      DMatrixRMaj vector = new DMatrixRMaj(4, 1);
      CommonOps_DDRM.mult(OdometryTools.lOperator(a), OdomTestTools.toVector(b), vector);

      return vector;
   }

   static DMatrixRMaj multiplyLeftInverse(QuaternionReadOnly a, QuaternionReadOnly b)
   {
      DMatrixRMaj vector = new DMatrixRMaj(4, 1);
      CommonOps_DDRM.mult(OdometryTools.lOperator(a.getS(), -a.getX(), -a.getY(), -a.getZ()), OdomTestTools.toVector(b), vector);

      return vector;
   }

   static class TestBiasProvider implements IMUBiasProvider
   {

      @Override
      public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
      {
         return new FrameVector3D(imu.getMeasurementFrame());
      }

      @Override
      public FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu)
      {
         return new FrameVector3D();
      }

      @Override
      public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
      {
         return new FrameVector3D(imu.getMeasurementFrame());
      }

      @Override
      public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
      {
         return new FrameVector3D();
      }
   }

   static class TestIMU implements IMUSensorReadOnly
   {
      private String name;
      public final MovingReferenceFrame sensorFrame;
      public final Pose3D pose = new Pose3D();
      public final Vector3D angularVelocity = new Vector3D();
      public final Vector3D linearVelocity = new Vector3D();
      public final Vector3D linearAcceleration = new Vector3D();

      public TestIMU(String name)
      {
         this.name = name;

         sensorFrame = new MovingReferenceFrame(name, ReferenceFrame.getWorldFrame())
         {
            @Override
            protected void updateTwistRelativeToParent(Twist twist)
            {
               twist.setToZero();
               pose.transform(linearVelocity, twist.getLinearPart());
            }

            @Override
            protected void updateTransformToParent(RigidBodyTransform rigidBodyTransform)
            {
               rigidBodyTransform.set(pose);
            }
         };
      }

      @Override
      public String getSensorName()
      {
         return name;
      }

      @Override
      public MovingReferenceFrame getMeasurementFrame()
      {
         return sensorFrame;
      }

      @Override
      public RigidBodyBasics getMeasurementLink()
      {
         return null;
      }

      @Override
      public QuaternionReadOnly getOrientationMeasurement()
      {
         return pose.getOrientation();
      }

      @Override
      public Vector3DReadOnly getAngularVelocityMeasurement()
      {
         return angularVelocity;
      }

      @Override
      public Vector3DReadOnly getLinearAccelerationMeasurement()
      {
         return linearAcceleration;
      }

      @Override
      public void getOrientationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
      {

      }

      @Override
      public void getAngularVelocityNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
      {

      }

      @Override
      public void getAngularVelocityBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
      {

      }

      @Override
      public void getLinearAccelerationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
      {

      }

      @Override
      public void getLinearAccelerationBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
      {

      }
   }
}
