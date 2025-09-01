package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

class SensedVariables
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // State providers
   private final IMUSensorReadOnly baseIMU;
   private final IMUSensorReadOnly imu;
   private final IMUBiasProvider imuBiasProvider;

   // State recorders
   public final YoFrameVector3D gyroMeasurementInWorld;
   public final YoFrameVector3D gyroMeasurement;
   public final YoFrameVector3D accelMeasurementInWorld;
   public final YoFrameVector3D accelMeasurement;

   public final YoFramePoint3D positionMeasurement;
   public final YoFrameQuaternion orientationMeasurement;
   public final YoFrameVector3D linearVelocity;

   // Temp variables
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final Twist twist = new Twist();

   public SensedVariables(String prefix, IMUSensorReadOnly baseIMU, IMUSensorReadOnly imu, IMUBiasProvider imuBiasProvider, YoRegistry registry)
   {
      this.baseIMU = baseIMU;
      this.imu = imu;
      this.imuBiasProvider = imuBiasProvider;

      gyroMeasurementInWorld = new YoFrameVector3D(prefix + "GyroMeasurementInWorld", worldFrame, registry);
      gyroMeasurement = new YoFrameVector3D(prefix + "GyroMeasurement", imu.getMeasurementFrame(), registry);
      accelMeasurementInWorld = new YoFrameVector3D(prefix + "AccelMeasurementInWorld", worldFrame, registry);
      accelMeasurement = new YoFrameVector3D(prefix + "AccelMeasurement", imu.getMeasurementFrame(), registry);

      positionMeasurement = new YoFramePoint3D(prefix + "PositionMeasurement", baseIMU.getMeasurementFrame(), registry);
      orientationMeasurement = new YoFrameQuaternion(prefix + "OrientationMeasurement", baseIMU.getMeasurementFrame(), registry);

      linearVelocity = new YoFrameVector3D(prefix + "LinearVelocity", worldFrame, registry);
   }

   public void update()
   {
      // Update gyro measure
      FrameVector3DReadOnly gyroBiasInput = imuBiasProvider.getAngularVelocityBiasInIMUFrame(imu);
      Vector3DReadOnly gyroRawInput = imu.getAngularVelocityMeasurement();

      angularVelocity.setReferenceFrame(imu.getMeasurementFrame());
      angularVelocity.sub(gyroRawInput, gyroBiasInput);

      gyroMeasurementInWorld.setMatchingFrame(angularVelocity);
      gyroMeasurement.setMatchingFrame(angularVelocity);

      // Update the accelerometer measure
      FrameVector3DReadOnly accelBiasInput = imuBiasProvider.getLinearAccelerationBiasInIMUFrame(imu);
      Vector3DReadOnly accelRawInput = imu.getLinearAccelerationMeasurement();

      linearAcceleration.setReferenceFrame(imu.getMeasurementFrame());
      linearAcceleration.sub(accelRawInput, accelBiasInput);

      accelMeasurementInWorld.setMatchingFrame(linearAcceleration);
      accelMeasurement.setMatchingFrame(linearAcceleration);

      positionMeasurement.setFromReferenceFrame(imu.getMeasurementFrame());
      orientationMeasurement.setFromReferenceFrame(imu.getMeasurementFrame());

      FramePoint3D imuInWorld = new FramePoint3D(imu.getMeasurementFrame());
      FramePoint3D baseInWorld = new FramePoint3D(baseIMU.getMeasurementFrame());
      imuInWorld.changeFrame(worldFrame);
      baseInWorld.changeFrame(worldFrame);

      imu.getMeasurementFrame().getTwistRelativeToOther(baseIMU.getMeasurementFrame(), twist);
      twist.changeFrame(worldFrame);
      linearVelocity.set(twist.getLinearPart());
   }
}
