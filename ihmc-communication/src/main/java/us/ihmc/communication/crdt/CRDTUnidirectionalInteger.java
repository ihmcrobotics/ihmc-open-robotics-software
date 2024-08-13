package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalInteger extends CRDTUnidirectionalField
{
   private int value;

   public CRDTUnidirectionalInteger(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable, int initialValue)
   {
      super(sideThatCanModify, requestConfirmFreezable);

      value = initialValue;
   }

   public int getValue()
   {
      return value;
   }

   public void setValue(int value)
   {
      if (this.value != value)
      {
         checkActorCanModifyAndFreeze();

         this.value = value;
      }
   }

   public int toMessage()
   {
      return value;
   }

   public void fromMessage(int value)
   {
      if (isNotFrozen())
      {
         this.value = value;
      }
   }
}
