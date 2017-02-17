package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface FloatingInverseDynamicsJoint extends InverseDynamicsJoint
{
   public abstract FloatingInverseDynamicsJointReferenceFrame getFrameAfterJoint();

   public abstract void setRotation(double yaw, double pitch, double roll);

   public abstract void setRotation(QuaternionReadOnly jointRotation);

   public abstract void setRotation(double x, double y, double z, double w);

   public abstract void setRotation(RotationMatrixReadOnly jointRotation);

   public abstract void setPosition(Tuple3DReadOnly qTrans);

   public abstract void setPosition(double x, double y, double z);

   public abstract void setPositionAndRotation(RigidBodyTransform transform);

   public abstract void getTranslation(Tuple3DBasics translationToPack);

   public abstract void getAngularVelocity(Vector3DBasics angularVelocityToPack);

   public abstract void getLinearVelocity(Vector3DBasics linearVelocityToPack);

   public abstract void setJointTwist(Twist jointTwist);

   public abstract void setAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void setDesiredAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void setWrench(Wrench jointWrench);

   public abstract void getRotation(QuaternionBasics rotationToPack);

   public abstract void getRotation(RotationMatrix rotationToPack);

   public abstract void getRotation(double[] yawPitchRoll);

   public abstract Tuple3DReadOnly getTranslationForReading();

   public abstract QuaternionReadOnly getRotationForReading();

   public abstract Vector3DReadOnly getLinearVelocityForReading();

   public abstract Vector3DReadOnly getAngularVelocityForReading();

   public abstract void getLinearAcceleration(Vector3DBasics linearAccelerationToPack);

   public abstract void getWrench(Wrench wrenchToPack);

   public void getJointTransform3D(RigidBodyTransform rigidBodyTransformToPack);
}
