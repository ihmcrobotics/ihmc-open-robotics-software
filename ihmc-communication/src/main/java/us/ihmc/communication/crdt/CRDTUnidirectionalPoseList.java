package us.ihmc.communication.crdt;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.idl.IDLSequence;

/**
 * Represents a list of  that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTUnidirectionalPoseList extends CRDTUnidirectionalMutableField<RecyclingArrayList<Pose3D>>
{
   public CRDTUnidirectionalPoseList(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable)
   {
      super(sideThatCanModify, requestConfirmFreezable, () -> new RecyclingArrayList<>(Pose3D::new));
   }

   public Pose3DReadOnly getValueReadOnly(int index)
   {
      return getValueInternal().get(index);
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   public void toMessage(IDLSequence.Object<Pose3D> trajectoryMessage)
   {
      trajectoryMessage.clear();

      for (Pose3D pose3D : getValueInternal())
      {
         trajectoryMessage.add().set(pose3D);
      }
   }

   public void fromMessage(IDLSequence.Object<Pose3D> trajectoryMessage)
   {
      if (isNotFrozen())
      {
         getValueInternal().clear();

         for (Pose3D pose3D : trajectoryMessage)
         {
            getValueInternal().add().set(pose3D);
         }
      }
   }
}
