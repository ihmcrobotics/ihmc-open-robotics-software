package us.ihmc.communication.crdt;

import controller_msgs.RigidBodyTransformMessage;
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
   public CRDTBidirectionalRigidBodyTransform(LatestTimestampModifiable latestTimestampModifiable)
   {
      super(latestTimestampModifiable, new RigidBodyTransform());
   }

   public RigidBodyTransformReadOnly getValueReadOnly()
   {
      return getValueInternal();
   }

   /**
    * Used to initialize Pose 3D gizmos. This implementation should be reconsidered.
    */
   public RigidBodyTransform getValueUnsafe()
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
         getValueAndModify().set(value);
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
      if (isModificationIncoming())
      {
         MessageTools.toEuclid(rigidBodyTransformMessage, getValueInternal());
      }
   }

   public void fromMessage(Pose3D poseMessage)
   {
      if (isModificationIncoming())
      {
         getValueInternal().set(poseMessage);
      }
   }
}
