package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalLong extends CRDTUnidirectionalField
{
   private long value;

   public CRDTUnidirectionalLong(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable, long initialValue)
   {
      super(sideThatCanModify, requestConfirmFreezable);

      value = initialValue;
   }

   public long getValue()
   {
      return value;
   }

   public void setValue(long value)
   {
      if (this.value != value)
      {
         checkActorCanModifyAndFreeze();

         this.value = value;
      }
   }

   public long toMessage()
   {
      return value;
   }

   public void fromMessage(long value)
   {
      if (isNotFrozen())
      {
         this.value = value;
      }
   }
}
