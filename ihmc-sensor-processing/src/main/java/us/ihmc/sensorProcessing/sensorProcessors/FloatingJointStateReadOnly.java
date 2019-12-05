package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.function.Supplier;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

public interface FloatingJointStateReadOnly
{
   Pose3DReadOnly getPose();

   TwistReadOnly getTwist();

   SpatialAccelerationReadOnly getAcceleration();

   public static FloatingJointStateReadOnly fromFloatingJoint(FloatingJointReadOnly joint)
   {
      return fromSuppliers(joint.getName(), joint::getJointPose, joint::getJointTwist, joint::getJointAcceleration);
   }

   public static FloatingJointStateReadOnly fromSuppliers(String jointName, Supplier<Pose3DReadOnly> pose, Supplier<TwistReadOnly> twist,
                                                          Supplier<SpatialAccelerationReadOnly> acceleration)
   {
      return new FloatingJointStateReadOnly()
      {
         @Override
         public Pose3DReadOnly getPose()
         {
            return pose.get();
         }

         @Override
         public TwistReadOnly getTwist()
         {
            return twist.get();
         }

         @Override
         public SpatialAccelerationReadOnly getAcceleration()
         {
            return acceleration.get();
         }
      };
   }
}
