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
      if (this.value != value)
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
