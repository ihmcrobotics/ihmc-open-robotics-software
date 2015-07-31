package us.ihmc.sensorProcessing.sensors;

import javax.vecmath.Quat4d;

import us.ihmc.robotics.geometry.FrameVector;

public interface ProcessedIMUSensorsReadOnlyInterface
{
   public abstract FrameVector getAcceleration(int imuIndex);
   
   public abstract double getYaw(int imuIndex);

   public abstract double getPitch(int imuIndex);

   public abstract double getRoll(int imuIndex);
   
   public abstract Quat4d getQuaternion(int imuIndex);

   public abstract FrameVector getAngularVelocity(int imuIndex);
   
   public abstract FrameVector getAngularAcceleration(int imuIndex);
}