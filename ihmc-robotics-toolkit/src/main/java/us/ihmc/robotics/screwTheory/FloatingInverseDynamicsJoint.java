package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public interface FloatingInverseDynamicsJoint extends JointBasics
{
   default Pose3DBasics getJointPose()
   {
      return null;
   }

   default TwistReadOnly getJointTwist()
   {
      return null;
   }

   default SpatialAccelerationReadOnly getJointAcceleration()
   {
      return null;
   }

   default WrenchReadOnly getJointWrench()
   {
      return null;
   }

   public abstract void setJointOrientation(QuaternionReadOnly jointRotation);

   public abstract void setJointOrientation(RotationMatrixReadOnly jointRotation);

   public abstract void setJointPosition(Tuple3DReadOnly qTrans);

   public abstract void setJointConfiguration(RigidBodyTransform transform);

   default void setJointConfiguration(Pose3DReadOnly pose)
   {

   }

   public abstract void setJointTwist(Twist jointTwist);

   public abstract void setJointAcceleration(SpatialAcceleration jointAcceleration);

   public abstract void setDesiredAcceleration(SpatialAcceleration jointAcceleration);

   public abstract void setWrench(Wrench jointWrench);

   public void getJointTransform3D(RigidBodyTransform rigidBodyTransformToPack);
}
