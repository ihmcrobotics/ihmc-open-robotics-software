package us.ihmc.robotics.sensors;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameVector;

public interface ProcessedIMUSensorsWriteOnlyInterface
{
   public abstract void setAcceleration(FrameVector accelerationInWorld, int imuIndex);

   public abstract void setRotation(RotationMatrix rotationMatrix, int imuIndex);

   public abstract void setAngularVelocityInBody(Vector3D angularVelocityInBody, int imuIndex);

   public abstract void setAngularAccelerationInBody(Vector3D angularAccelerationInBody, int imuIndex);
}