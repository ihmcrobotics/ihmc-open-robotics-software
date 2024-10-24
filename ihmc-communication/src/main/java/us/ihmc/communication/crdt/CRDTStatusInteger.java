package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTStatusInteger extends CRDTStatusField
{
   private int value;

   public CRDTStatusInteger(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, int initialValue)
   {
      super(sideThatCanModify, crdtInfo);

      value = initialValue;
   }

   public int getValue()
   {
      return value;
   }

   public void setValue(int value)
   {
      checkActorCanModifyAndMarkHasStatus();

      this.value = value;
   }

   public int toMessage()
   {
      return value;
   }

   public void fromMessage(int value)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         this.value = value;
      }
   }
}
