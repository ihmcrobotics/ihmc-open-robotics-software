package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalImmutableField<T> extends CRDTUnidirectionalField
{
   private T value;

   public CRDTUnidirectionalImmutableField(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable, T initialValue)
   {
      super(sideThatCanModify, requestConfirmFreezable);

      value = initialValue;
   }

   public T getValue()
   {
      return value;
   }

   public void setValue(T value)
   {
      boolean nullValuePresent = this.value == null || value == null;

      boolean equals;
      if (nullValuePresent)
         equals = this.value == value;
      else
         equals = this.value.equals(value);

      if (!equals) // Don't want to do anything in the case nothing changed
      {
         checkActorCanModifyAndFreeze();

         this.value = value;
      }
   }

   public T toMessage()
   {
      return value;
   }

   public void fromMessage(T value)
   {
      if (isNotFrozen())
      {
         this.value = value;
      }
   }
}
