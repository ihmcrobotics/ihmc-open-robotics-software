package us.ihmc.communication.crdt;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

public class CRDTBidirectionalSet<T> extends CRDTBidirectionalMutableField<Set<T>>
{
   public CRDTBidirectionalSet(LatestTimestampModifiable latestTimestampModifiable)
   {
      this(latestTimestampModifiable, new HashSet<>());
   }

   public CRDTBidirectionalSet(LatestTimestampModifiable latestTimestampModifiable, Set<T> internalSet)
   {
      super(latestTimestampModifiable, internalSet);
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   public boolean add(T element)
   {
      boolean added = getValueInternal().add(element);
      if (added)
         modify();
      return added;
   }

   public boolean remove(T object)
   {
      boolean removed = getValueInternal().remove(object);
      if (removed)
         modify();
      return removed;
   }

   public boolean addAll(Collection<? extends T> collection)
   {
      boolean changed = getValueInternal().addAll(collection);
      if (changed)
         modify();
      return changed;
   }

   public boolean retainAll(Collection<T> collection)
   {
      boolean changed = getValueInternal().retainAll(collection);
      if (changed)
         modify();
      return changed;   }

   public boolean removeAll(Collection<T> collection)
   {
      boolean changed = getValueInternal().removeAll(collection);
      if (changed)
         modify();
      return changed;
   }

   public void clear()
   {
      getValueAndModify().clear();
   }

   public void fromMessage(Consumer<Set<T>> valueConsumer)
   {
      if (isModificationIncoming())
      {
         valueConsumer.accept(getValueInternal());
      }
   }
}
