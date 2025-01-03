package us.ihmc.communication.crdt;

import us.ihmc.commons.lists.RecyclingArrayList;

import java.util.function.Consumer;

/**
 * Represents a recycling array list that can be modified by both the
 * robot and the operator. The internal writeable instance is kept protected
 * from unchecked modifications.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalRecyclingArrayList<T> extends CRDTBidirectionalMutableField<RecyclingArrayList<T>>
{
   public CRDTBidirectionalRecyclingArrayList(RequestConfirmFreezable requestConfirmFreezable, RecyclingArrayList<T> initialValue)
   {
      super(requestConfirmFreezable, initialValue);
   }

   public T getValueReadOnly(int index)
   {
      return getValueInternal().get(index);
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   /** Use to prevent unecessary freezes. */
   public void setValue(int index, T value)
   {
      if (!getValueReadOnly(index).equals(value))
         getValueAndFreeze().set(index, value);
   }

   public int getLength()
   {
      return getValueInternal().size();
   }

   /**
    * Used only for preallocating using {@link us.ihmc.commons.lists.RecyclingArrayListTools#getUnsafe}.
    */
   public RecyclingArrayList<T> getValueUnsafe()
   {
      return getValueInternal();
   }

   public void fromMessage(Consumer<RecyclingArrayList<T>> valueConsumer)
   {
      if (!isFrozen()) // Ignore updates if we are frozen
      {
         valueConsumer.accept(getValueInternal());
      }
   }
}
