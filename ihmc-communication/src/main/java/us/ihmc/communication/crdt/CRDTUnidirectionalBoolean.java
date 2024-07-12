package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalBoolean extends CRDTUnidirectionalField
{
   private boolean value;

   public CRDTUnidirectionalBoolean(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable, boolean initialValue)
   {
      super(sideThatCanModify, requestConfirmFreezable);

      value = initialValue;
   }

   public boolean getValue()
   {
      return value;
   }

   public void setValue(boolean value)
   {
      if (this.value != value)
      {
         checkActorCanModifyAndFreeze();

         this.value = value;
      }
   }

   public boolean toMessage()
   {
      return value;
   }

   public void fromMessage(boolean value)
   {
      if (isNotFrozen())
      {
         this.value = value;
      }
   }
}
