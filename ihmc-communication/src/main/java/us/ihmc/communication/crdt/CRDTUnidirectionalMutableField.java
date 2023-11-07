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
public abstract class CRDTUnidirectionalMutableField<T>
{
   private final ROS2ActorDesignation sideThatCanModify;
   private final CRDTInfo crdtInfo;

   private final T value;

   public CRDTUnidirectionalMutableField(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, Supplier<T> valueSupplier)
   {
      this.sideThatCanModify = sideThatCanModify;
      this.crdtInfo = crdtInfo;
      value = valueSupplier.get();
   }

   public T getValue()
   {
      checkActorCanModify();
      return value;
   }

   protected T getValueInternal()
   {
      return value;
   }

   protected void checkActorCanModify()
   {
      if (!canActorModify())
         throw new RuntimeException("%s is not allowed to modify this value.".formatted(crdtInfo.getActorDesignation()));
   }

   protected boolean canActorModify()
   {
      return sideThatCanModify == crdtInfo.getActorDesignation();
   }
}
