package us.ihmc.robotics.sensors;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameVector;

public interface ProcessedIMUSensorsWriteOnlyInterface
{
   public abstract void setAcceleration(FrameVector accelerationInWorld, int imuIndex);

   public abstract void setRotation(Matrix3d rotationMatrix, int imuIndex);

   public abstract void setAngularVelocityInBody(Vector3d angularVelocityInBody, int imuIndex);

   public abstract void setAngularAccelerationInBody(Vector3d angularAccelerationInBody, int imuIndex);
}