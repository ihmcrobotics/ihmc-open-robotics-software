package us.ihmc.communication.crdt;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * Represents a RigidBodyTransform that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTUnidirectionalRigidBodyTransform
{
   private final ROS2ActorDesignation sideThatCanModify;
   private final CRDTInfo crdtInfo;

   private final RigidBodyTransform value = new RigidBodyTransform();

   public CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      this.sideThatCanModify = sideThatCanModify;
      this.crdtInfo = crdtInfo;
   }

   public RigidBodyTransformReadOnly getValueReadOnly()
   {
      return value;
   }

   public RigidBodyTransform getValue()
   {
      checkActorCanModify();
      return value;
   }

   private void checkActorCanModify()
   {
      if (sideThatCanModify != crdtInfo.getActorDesignation())
         throw new RuntimeException("%s is not allowed to modify this value.".formatted(crdtInfo.getActorDesignation()));
   }

   public void fromMessage(RigidBodyTransformMessage rigidBodyTransformMessage)
   {
      if (sideThatCanModify != crdtInfo.getActorDesignation()) // Ignore updates if we are the only side that can modify
      {
         MessageTools.toEuclid(rigidBodyTransformMessage, value);
      }
   }

   public void toMessage(RigidBodyTransformMessage rigidBodyTransformMessage)
   {
      MessageTools.toMessage(value, rigidBodyTransformMessage);
   }
}
