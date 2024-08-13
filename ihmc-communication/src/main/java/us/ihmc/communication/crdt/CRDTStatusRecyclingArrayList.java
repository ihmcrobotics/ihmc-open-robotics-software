package us.ihmc.communication.crdt;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Represents a list that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTStatusRecyclingArrayList<T> extends CRDTStatusMutableField<RecyclingArrayList<T>>
{
   public CRDTStatusRecyclingArrayList(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, Supplier<RecyclingArrayList<T>> valueSupplier)
   {
      super(sideThatCanModify, crdtInfo, valueSupplier);
   }

   public T getValueReadOnly(int index)
   {
      return getValueInternal().get(index);
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   /**
    * Used only for preallocating using {@link us.ihmc.robotics.lists.RecyclingArrayListTools#getUnsafe}.
    */
   public RecyclingArrayList<T> getValueUnsafe()
   {
      return getValueInternal();
   }

   public void fromMessage(Consumer<RecyclingArrayList<T>> valueConsumer)
   {
      if (isModificationDisallowed())
      {
         valueConsumer.accept(getValueInternal());
      }
   }
}