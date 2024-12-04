package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.Objects;

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
      if (!Objects.equals(this.value, value)) // Don't want to do anything in the case nothing changed
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
