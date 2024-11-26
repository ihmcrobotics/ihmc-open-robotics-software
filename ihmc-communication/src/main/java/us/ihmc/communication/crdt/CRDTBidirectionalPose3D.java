package us.ihmc.communication.crdt;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;

/**
 * Represents a Pose3D that that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalPose3D extends CRDTBidirectionalMutableField<Pose3D>
{
   public CRDTBidirectionalPose3D(RequestConfirmFreezable requestConfirmFreezable)
   {
      super(requestConfirmFreezable, new Pose3D());
   }

   public Pose3DReadOnly getValueReadOnly()
   {
      return getValueInternal();
   }

   public void toMessage(Pose3D message)
   {
      message.set(getValueReadOnly());
   }

   public void fromMessage(Pose3D message)
   {
      if (!isFrozen())
      {
         getValueInternal().set(message);
      }
   }
}