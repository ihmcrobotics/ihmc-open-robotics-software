package us.ihmc.communication.crdt;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * Represents a RigidBodyTransform that can be modified by both the
 * robot and the operator. The internal writeable instance is kept protected
 * from unchecked modifications.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalRigidBodyTransform extends CRDTBidirectionalMutableField<RigidBodyTransform>
{
   public CRDTBidirectionalRigidBodyTransform(RequestConfirmFreezable requestConfirmFreezable)
   {
      super(requestConfirmFreezable, new RigidBodyTransform());
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
      if (!isFrozen()) // Ignore updates if we are frozen
      {
         MessageTools.toEuclid(rigidBodyTransformMessage, getValueInternal());
      }
   }

   public void fromMessage(Pose3D poseMessage)
   {
      if (!isFrozen()) // Ignore updates if we are frozen
      {
         getValueInternal().set(poseMessage);
      }
   }
}
