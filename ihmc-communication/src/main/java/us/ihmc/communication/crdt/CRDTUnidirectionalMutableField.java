package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.function.Supplier;

/**
 * Represents a mutable field that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 *
 * This is abstract because there is a need to provide a read-only access to this
 * mutable value, which will vary by type.
 */
public abstract class CRDTUnidirectionalMutableField<T> extends CRDTUnidirectionalField
{
   private final T value;

   public CRDTUnidirectionalMutableField(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable, Supplier<T> valueSupplier)
   {
      super(sideThatCanModify, requestConfirmFreezable);

      value = valueSupplier.get();
   }

   /**
    * Accessing this freezes the node and assumes a modification will be made.
    * Only use this if modifying the value.
    */
   public T accessValue()
   {
      checkActorCanModifyAndFreeze();
      return value;
   }

   protected T getValueInternal()
   {
      return value;
   }
}
