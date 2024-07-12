package us.ihmc.communication.crdt;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
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
public class CRDTUnidirectionalRigidBodyTransform extends CRDTUnidirectionalMutableField<RigidBodyTransform>
{
   public CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable)
   {
      super(sideThatCanModify, requestConfirmFreezable, RigidBodyTransform::new);
   }

   public RigidBodyTransformReadOnly getValueReadOnly()
   {
      return getValueInternal();
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
      if (isNotFrozen())
      {
         MessageTools.toEuclid(rigidBodyTransformMessage, getValueInternal());
      }
   }

   public void fromMessage(Pose3D poseMessage)
   {
      if (isNotFrozen())
      {
         getValueInternal().set(poseMessage);
      }
   }
}
