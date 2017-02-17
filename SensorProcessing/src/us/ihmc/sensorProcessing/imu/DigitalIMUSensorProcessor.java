package us.ihmc.sensorProcessing.imu;


import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
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

   private final RotationMatrix rotationMatrixBeforeOffset = new RotationMatrix();
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final Vector3D angularVelocity = new Vector3D();
   private final RotationMatrix orientationOffset; // rotates vectors from robot body to IMU body
   private final RotationMatrix orientationOffsetTranspose = new RotationMatrix();
   private final Vector3D accelerationOffset;
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
      this.orientationOffsetTranspose.setAndTranspose(this.orientationOffset);
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

      rotationMatrix.set(rotationMatrixBeforeOffset);
      rotationMatrix.multiply(orientationOffset);
      processedSensors.setRotation(rotationMatrix, imuIndex);
   }

   private void processAcceleration()
   {
      // Compute acceleration in world frame, subtracting off gravity:
      Vector3D acceleration = new Vector3D();
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
