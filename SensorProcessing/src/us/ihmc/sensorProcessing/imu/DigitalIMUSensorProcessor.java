package us.ihmc.sensorProcessing.imu;


import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.sensors.ProcessedIMUSensorsWriteOnlyInterface;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;

public class DigitalIMUSensorProcessor implements IMUSensorProcessor
{
   private final String name = getClass().getSimpleName();
   private final RawIMUSensorsInterface rawIMUSensors;
   private final ProcessedIMUSensorsWriteOnlyInterface processedSensors;

   private final Matrix3d rotationMatrixBeforeOffset = new Matrix3d();
   private final Matrix3d rotationMatrix = new Matrix3d();
   private final Vector3d angularVelocity = new Vector3d();
   private final Matrix3d orientationOffset; // rotates vectors from robot body to IMU body
   private final Matrix3d orientationOffsetTranspose = new Matrix3d();
   private final Vector3d accelerationOffset;
   private final int imuIndex;
   private final double localGravityZ;

   public DigitalIMUSensorProcessor(RawIMUSensorsInterface rawIMUSensors, ProcessedIMUSensorsWriteOnlyInterface processedIMUSensors, int imuIndex, double localGravityPositiveZ)
   {
      this.rawIMUSensors = rawIMUSensors;
      this.processedSensors = processedIMUSensors;
      this.imuIndex = imuIndex;
      this.localGravityZ = localGravityPositiveZ;
      
      IMUCalibrationProperties imuCalibrationProperties = new IMUCalibrationProperties(imuIndex);
      this.orientationOffset = imuCalibrationProperties.getOrientationOffset();
      this.orientationOffsetTranspose.transpose(this.orientationOffset);
      this.accelerationOffset = imuCalibrationProperties.getAccelerationOffset();
      imuCalibrationProperties.save();
   }

   public void initialize()
   {
      update();
   }

   public void update()
   {
      processAngularVelocity();

      processOrientation();

      processAcceleration();
   }

   private void processAngularVelocity()
   {
      // use orientationOffset^T instead of rotationMatrix, because the angular velocity is measured in IMU body!
      // omega^B = R^B_IMUBody * omega^IMUBody
      rawIMUSensors.getAngularVelocity(angularVelocity, imuIndex);
      orientationOffsetTranspose.transform(angularVelocity);
      processedSensors.setAngularVelocityInBody(angularVelocity, imuIndex);
   }

   private void processOrientation()
   {
      rawIMUSensors.getOrientation(rotationMatrixBeforeOffset, imuIndex);

      rotationMatrix.mul(rotationMatrixBeforeOffset, orientationOffset);
      processedSensors.setRotation(rotationMatrix, imuIndex);
   }

   private void processAcceleration()
   {
      // Compute acceleration in world frame, subtracting off gravity:
      Vector3d acceleration = new Vector3d();
      rawIMUSensors.getAcceleration(acceleration, imuIndex);
      acceleration.sub(accelerationOffset);

      // use rotationMatrixBeforeOffset instead of rotationMatrix, because the accelerations are measured in IMU body!
      // a(M') = R(M'S') * a(S')
      rotationMatrixBeforeOffset.transform(acceleration);
      acceleration.setZ(acceleration.getZ() - localGravityZ);
      FrameVector frameAcceleration = new FrameVector(ReferenceFrame.getWorldFrame(), acceleration);

      processedSensors.setAcceleration(frameAcceleration, imuIndex);
   }
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return name;
   }
}
