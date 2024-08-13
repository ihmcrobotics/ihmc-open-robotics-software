package us.ihmc.communication.crdt;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Represents a list of footsteps that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTStatusFootstepList extends CRDTStatusMutableField<RecyclingArrayList<FootstepPlanActionFootstepDefinitionMessage>>
{
   public CRDTStatusFootstepList(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      super(sideThatCanModify, crdtInfo, () -> new RecyclingArrayList<>(FootstepPlanActionFootstepDefinitionMessage::new));
   }

   public Pose3DReadOnly getPoseReadOnly(int index)
   {
      return getValueInternal().get(index).getSolePose();
   }

   public RobotSide getSide(int index)
   {
      return RobotSide.fromByte(getValueInternal().get(index).getRobotSide());
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   public void toMessage(IDLSequence.Object<FootstepPlanActionFootstepDefinitionMessage> message)
   {
      message.clear();

      for (FootstepPlanActionFootstepDefinitionMessage footstep : getValueInternal())
      {
         message.add().set(footstep);
      }
   }

   public void fromMessage(IDLSequence.Object<FootstepPlanActionFootstepDefinitionMessage> message)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         getValueInternal().clear();

         for (FootstepPlanActionFootstepDefinitionMessage footstep : message)
         {
            getValueInternal().add().set(footstep);
         }
      }
   }
}
