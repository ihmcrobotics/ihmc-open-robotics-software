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
public abstract class CRDTStatusMutableField<T> extends CRDTStatusField
{
   private final T value;

   public CRDTStatusMutableField(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, Supplier<T> valueSupplier)
   {
      super(sideThatCanModify, crdtInfo);

      value = valueSupplier.get();
   }

   public T accessValue()
   {
      checkActorCanModifyAndMarkHasStatus();
      return value;
   }

   protected T getValueInternal()
   {
      return value;
   }
}
