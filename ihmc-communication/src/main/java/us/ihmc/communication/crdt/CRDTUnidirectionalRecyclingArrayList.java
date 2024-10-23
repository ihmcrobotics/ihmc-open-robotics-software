package us.ihmc.communication.crdt;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.RecyclingArrayListTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Represents a list that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTUnidirectionalRecyclingArrayList<T> extends CRDTUnidirectionalMutableField<RecyclingArrayList<T>>
{
   public CRDTUnidirectionalRecyclingArrayList(ROS2ActorDesignation sideThatCanModify,
                                               RequestConfirmFreezable requestConfirmFreezable,
                                               Supplier<RecyclingArrayList<T>> valueSupplier)
   {
      super(sideThatCanModify, requestConfirmFreezable, valueSupplier);
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
    * Used only for preallocating using {@link RecyclingArrayListTools#getUnsafe}.
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