package us.ihmc.communication.crdt;

import controller_msgs.RigidBodyTransformMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * Represents a RigidBodyTransform that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTStatusRigidBodyTransform extends CRDTStatusMutableField<RigidBodyTransform>
{
   public CRDTStatusRigidBodyTransform(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      super(sideThatCanModify, crdtInfo, RigidBodyTransform::new);
   }

   public RigidBodyTransformReadOnly getValueReadOnly()
   {
      return getValueInternal();
   }

   /** Prefer this method in the case you need to call it every tick, as it no-ops in the case the value is the same. */
   public void setValue(RigidBodyTransformReadOnly value, double epsilon)
   {
      // rotation and translation must be checked separately to handle pose-transform comparison case
      // translation must use epsilonEquals because it can be vector vs. point
      if (!(getValueInternal().getRotation().geometricallyEquals(value.getRotation(), epsilon)
         && getValueInternal().getTranslation().epsilonEquals(value.getTranslation(), epsilon)))
      {
         accessValue().set(value);
      }
   }

   public void toMessage(Pose3D poseMessage)
   {
      poseMessage.set(getValueReadOnly());
   }

   public void toMessage(RigidBodyTransformMessage rigidBodyTransformMessage)
   {
      MessageTools.toMessage(getValueInternal(), rigidBodyTransformMessage);
   }

   public void fromMessage(RigidBodyTransformMessage rigidBodyTransformMessage)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         MessageTools.toEuclid(rigidBodyTransformMessage, getValueInternal());
      }
   }

   public void fromMessage(Pose3D poseMessage)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         getValueInternal().set(poseMessage);
      }
   }
}
