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
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

public class ProcessModelTest
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
      TestIMU baseIMU = new TestIMU("base");
      TestIMU footIMU = new TestIMU("foot");

      StateVariables processState = new StateVariables("process", footIMU.getMeasurementFrame(), test);
      StateVariables predictedState = new StateVariables("predicted", footIMU.getMeasurementFrame(), test);
      SensedVariables sensedVariables = new SensedVariables("foot", baseIMU, footIMU, new TestBiasProvider(), test);

      ProcessModel processModel = new ProcessModel(processState, predictedState, () -> true, gravity, estimatorDt);
      DMatrixRMaj jacobian = new DMatrixRMaj(OdometryIndexHelper.errorSizePerLink, OdometryIndexHelper.errorSizePerLink);

      for (int iter = 0; iter < iters; iter++)
      {
         OdomTestTools.setRandomSensed(random, sensedVariables);
         footIMU.linearAcceleration.set(sensedVariables.accelMeasurement);
         footIMU.angularVelocity.set(sensedVariables.gyroMeasurement);
         footIMU.pose.set(EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextOrientation3D(random));
         footIMU.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 10.0));
         footIMU.sensorFrame.update();
         baseIMU.pose.set(EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextOrientation3D(random));
         baseIMU.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 10.0));
         baseIMU.sensorFrame.update();

         OdomTestTools.setRandomState(random, sensedVariables, processState);
         OdomTestTools.setRandomState(random, sensedVariables, predictedState);

         processModel.update();

         DMatrixRMaj translation = new DMatrixRMaj(3, 1);
         DMatrixRMaj linearVelocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
         DMatrixRMaj acceleration = new DMatrixRMaj(3, 1);
         DMatrixRMaj accelerationBias = new DMatrixRMaj(3, 1);
         DMatrixRMaj accelerationUnbiased = new DMatrixRMaj(3, 1);
         DMatrixRMaj accelerationUnbiasedInWorld = new DMatrixRMaj(3, 1);
         DMatrixRMaj gyro = new DMatrixRMaj(3, 1);
         DMatrixRMaj gyroBias = new DMatrixRMaj(3, 1);
         DMatrixRMaj gyroUnbiased = new DMatrixRMaj(3, 1);

         processState.translation.get(translation);
         processState.linearVelocity.get(linearVelocity);
         OdometryTools.toRotationMatrix(processState.orientation, rotationMatrix);
         gravity.get(gravityVector);
         sensedVariables.accelMeasurement.get(acceleration);
         processState.accelBias.get(accelerationBias);
         sensedVariables.gyroMeasurement.get(gyro);
         processState.gyroBias.get(gyroBias);

         DMatrixRMaj expectedPredictedTranslation = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedPredictedLinearVelocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedPredictedRotation = new DMatrixRMaj(4, 1);
         DMatrixRMaj expectedPredictedAccelBias = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedPredictedGyroBias = new DMatrixRMaj(3, 1);

         // First term
         CommonOps_DDRM.add(estimatorDt, linearVelocity, translation, expectedPredictedTranslation);

         // Second term
         if (OdometryKalmanFilter.includeBias)
            CommonOps_DDRM.subtract(acceleration, accelerationBias, accelerationUnbiased);
         else
            accelerationUnbiased.set(acceleration);
         CommonOps_DDRM.mult(rotationMatrix, accelerationUnbiased, accelerationUnbiasedInWorld);
         CommonOps_DDRM.subtractEquals(accelerationUnbiasedInWorld, gravityVector);
         CommonOps_DDRM.add(estimatorDt, accelerationUnbiasedInWorld, linearVelocity, expectedPredictedLinearVelocity);

         // Third term
         if (OdometryKalmanFilter.includeBias)
            CommonOps_DDRM.subtract(gyro, gyroBias, gyroUnbiased);
         else
            gyroUnbiased.set(gyro);
         CommonOps_DDRM.scale(estimatorDt, gyroUnbiased);
         CommonOps_DDRM.mult(OdometryTools.lOperator(processState.orientation), OdometryTools.exponentialMap(gyroUnbiased), expectedPredictedRotation);

         // Fourth term
         expectedPredictedAccelBias.set(accelerationBias);

         // Fifth term
         expectedPredictedGyroBias.set(gyroBias);

         DMatrixRMaj predictedTranslation = new DMatrixRMaj(3, 1);
         DMatrixRMaj predictedLinearVelocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj predictedRotation = new DMatrixRMaj(4, 1);
         DMatrixRMaj predictedAccelBias = new DMatrixRMaj(3, 1);
         DMatrixRMaj predictedGyroBias = new DMatrixRMaj(3, 1);

         predictedState.translation.get(predictedTranslation);
         predictedState.linearVelocity.get(predictedLinearVelocity);
         predictedState.orientation.get(predictedRotation);
         predictedState.accelBias.get(predictedAccelBias);
         predictedState.gyroBias.get(predictedGyroBias);

         EjmlUnitTests.assertEquals(expectedPredictedTranslation, predictedTranslation, 1e-6);
         EjmlUnitTests.assertEquals(expectedPredictedLinearVelocity, predictedLinearVelocity, 1e-6);
         EjmlUnitTests.assertEquals(expectedPredictedRotation, predictedRotation, 1e-6);
         EjmlUnitTests.assertEquals(expectedPredictedAccelBias, predictedAccelBias, 1e-6);
         EjmlUnitTests.assertEquals(expectedPredictedGyroBias, predictedGyroBias, 1e-6);
      }
   }
   @Test
   public void testComputeProcessJacobian()
   {
      int iters = 100;
      Random random = new Random(1738L);

      YoRegistry test = new YoRegistry(getClass().getSimpleName());
      TestIMU baseIMU = new TestIMU("base");
      TestIMU footIMU = new TestIMU("foot");

      StateVariables processState = new StateVariables("process", footIMU.getMeasurementFrame(), test);
      StateVariables predictedState = new StateVariables("predicted", footIMU.getMeasurementFrame(), test);
      SensedVariables sensedVariables = new SensedVariables("foot", baseIMU, footIMU, new TestBiasProvider(), test);

      ProcessModel processModel = new ProcessModel(processState, predictedState, () -> true, gravity, estimatorDt);
      DMatrixRMaj jacobian = new DMatrixRMaj(OdometryIndexHelper.errorSizePerLink, OdometryIndexHelper.errorSizePerLink);

      for (int iter = 0; iter < iters; iter++)
      {
         jacobian.zero();
         OdomTestTools.setRandomSensed(random, sensedVariables);
         footIMU.linearAcceleration.set(sensedVariables.accelMeasurement);
         footIMU.angularVelocity.set(sensedVariables.gyroMeasurement);
         footIMU.pose.set(EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextOrientation3D(random));
         footIMU.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 10.0));
         footIMU.sensorFrame.update();
         baseIMU.pose.set(EuclidCoreRandomTools.nextPoint3D(random, 10.0), EuclidCoreRandomTools.nextOrientation3D(random));
         baseIMU.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 10.0));
         baseIMU.sensorFrame.update();

         OdomTestTools.setRandomState(random, sensedVariables, processState);
         OdomTestTools.setRandomState(random, sensedVariables, predictedState);

         processModel.computeProcessJacobian( 0, jacobian);

         DMatrixRMaj jacobianExpected = new DMatrixRMaj(OdometryIndexHelper.errorSizePerLink, OdometryIndexHelper.errorSizePerLink);
         // Row 1
         CommonOps_DDRM.setIdentity(jacobianExpected);
         for (int i = 0; i < 3; i++)
            jacobianExpected.set(i, i + 3, estimatorDt);

         // Row 2
         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
         DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);
         OdometryTools.toRotationMatrix(predictedState.orientation, rotationMatrix);
         DMatrixRMaj acceleration = new DMatrixRMaj(3, 1);
         DMatrixRMaj accelerationBias = new DMatrixRMaj(3, 1);
         DMatrixRMaj accelerationUnbiased = new DMatrixRMaj(3, 1);
         sensedVariables.accelMeasurement.get(acceleration);
         predictedState.accelBias.get(accelerationBias);
         if (OdometryKalmanFilter.includeBias)
            CommonOps_DDRM.subtract(acceleration, accelerationBias, accelerationUnbiased);
         else
            accelerationUnbiased.set(acceleration);
         CommonOps_DDRM.scale(estimatorDt, accelerationUnbiased);
         OdometryTools.toSkewSymmetricMatrix(accelerationUnbiased.get(0), accelerationUnbiased.get(1), accelerationUnbiased.get(2), skewMatrix);
         MatrixTools.multAddBlock(-1.0, rotationMatrix, skewMatrix, jacobianExpected, 3, 6);
         if (OdometryKalmanFilter.includeBias)
            MatrixTools.setMatrixBlock(jacobianExpected, 3, 9, rotationMatrix, 0, 0, 3, 3, -estimatorDt);

         // Row 3
         DMatrixRMaj velocity = new DMatrixRMaj(3, 1);
         DMatrixRMaj velocityBias = new DMatrixRMaj(3, 1);
         DMatrixRMaj velocityUnbiased = new DMatrixRMaj(3, 1);
         sensedVariables.gyroMeasurement.get(velocity);
         predictedState.gyroBias.get(velocityBias);
         if (OdometryKalmanFilter.includeBias)
            CommonOps_DDRM.subtract(velocity, velocityBias, velocityUnbiased);
         else
            velocityUnbiased.set(velocity);
         CommonOps_DDRM.scale(estimatorDt, velocityUnbiased);
         OdometryTools.toSkewSymmetricMatrix(velocityUnbiased.get(0), velocityUnbiased.get(1), velocityUnbiased.get(2), skewMatrix);
         MatrixTools.addMatrixBlock(jacobianExpected, 6, 6, skewMatrix, 0, 0, 3, 3, -1.0);
         if (OdometryKalmanFilter.includeBias)
         {
            for (int i = 0; i < 3; i++)
               jacobianExpected.set(i + 6, i + 12, -estimatorDt);
         }

         // Row 4 and 5 are already set
         EjmlUnitTests.assertEquals(jacobianExpected, jacobian, 1e-5);
      }
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
