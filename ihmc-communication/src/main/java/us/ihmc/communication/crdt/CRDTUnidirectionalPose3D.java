package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;

/**
 * Represents a Pose3D that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTUnidirectionalPose3D extends CRDTUnidirectionalMutableField<Pose3D>
{
   public CRDTUnidirectionalPose3D(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable)
   {
      super(sideThatCanModify, requestConfirmFreezable, Pose3D::new);
   }

   public Pose3DReadOnly getValueReadOnly()
   {
      return getValueInternal();
   }

   public void toMessage(Pose3D poseMessage)
   {
      poseMessage.set(getValueReadOnly());
   }

   public void fromMessage(Pose3D poseMessage)
   {
      if (isNotFrozen())
      {
         getValueInternal().set(poseMessage);
      }
   }
}