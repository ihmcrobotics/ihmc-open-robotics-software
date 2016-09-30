package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.*;

public interface FloatingInverseDynamicsJoint extends InverseDynamicsJoint
{
   public abstract FloatingInverseDynamicsJointReferenceFrame getFrameAfterJoint();

   public abstract void setRotation(double yaw, double pitch, double roll);

   public abstract void setRotation(Quat4d jointRotation);

   public abstract void setRotation(double x, double y, double z, double w);

   public abstract void setRotation(Matrix3d jointRotation);

   public abstract void setPosition(Tuple3d qTrans);

   public abstract void setPosition(double x, double y, double z);

   public abstract void setPositionAndRotation(RigidBodyTransform transform);

   public abstract void getTranslation(Tuple3d translationToPack);

   public abstract void getTranslation(Tuple3f translationToPack);

   public abstract void getAngularVelocity(Vector3d angularVelocityToPack);

   public abstract void getLinearVelocity(Vector3d linearVelocityToPack);

   public abstract void setJointTwist(Twist jointTwist);

   public abstract void setAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void setDesiredAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void setWrench(Wrench jointWrench);

   public abstract void getRotation(Quat4d rotationToPack);

   public abstract void getRotation(Quat4f rotationToPack);

   public abstract void getRotation(Matrix3d rotationToPack);

   public abstract void getRotation(double[] yawPitchRoll);

   public abstract Tuple3d getTranslationForReading();

   public abstract Quat4d getRotationForReading();

   public abstract Vector3d getLinearVelocityForReading();

   public abstract Vector3d getAngularVelocityForReading();

   public abstract void getLinearAcceleration(Vector3d linearAccelerationToPack);

   public abstract void getWrench(Wrench wrenchToPack);

   public void getJointTransform3D(RigidBodyTransform rigidBodyTransformToPack);
}
