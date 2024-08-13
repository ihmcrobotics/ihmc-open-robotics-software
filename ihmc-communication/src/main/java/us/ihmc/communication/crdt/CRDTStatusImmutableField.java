package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTStatusImmutableField<T> extends CRDTStatusField
{
   private T value;

   public CRDTStatusImmutableField(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, T initialValue)
   {
      super(sideThatCanModify, crdtInfo);

      value = initialValue;
   }

   public T getValue()
   {
      return value;
   }

   public void setValue(T value)
   {
      checkActorCanModifyAndMarkHasStatus();

      this.value = value;
   }

   public T toMessage()
   {
      return value;
   }

   public void fromMessage(T value)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         this.value = value;
      }
   }
}
