package us.ihmc.communication.crdt;

import behavior_msgs.WalkActionFootstepDefinitionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
// IDLSequence.Object is replaced with direct usage from jros2
// import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Represents a list of footsteps that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTStatusFootstepList extends CRDTStatusMutableField<RecyclingArrayList<WalkActionFootstepDefinitionMessage>>
{
   private final Pose3D tempPose = new Pose3D();

   public CRDTStatusFootstepList(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      super(sideThatCanModify, crdtInfo, () -> new RecyclingArrayList<>(WalkActionFootstepDefinitionMessage::new));
   }

   public Pose3DReadOnly getPoseReadOnly(int index)
   {
      // TODO jros2
//      MessageTools.fromMessage(getValueInternal().get(index).getSolePose(), tempPose);
      return tempPose;
   }

   public RobotSide getSide(int index)
   {
      return RobotSide.fromByte(getValueInternal().get(index).getRobotSide());
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   public void toMessage(us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<WalkActionFootstepDefinitionMessage> message)
   {
      message.clear();

      for (WalkActionFootstepDefinitionMessage footstep : getValueInternal())
      {
         message.add().set(footstep);
      }
   }

   public void fromMessage(us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<WalkActionFootstepDefinitionMessage> message)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         getValueInternal().clear();

         for (WalkActionFootstepDefinitionMessage footstep : message)
         {
            getValueInternal().add().set(footstep);
         }
      }
   }
}
