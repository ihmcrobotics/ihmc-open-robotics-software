package us.ihmc.communication.crdt;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

/**
 * Represents a set that can be modified by both the robot and the operator.
 * Provides methods for safely modifying the set.
 * <p>
 * Warning: With this type, the data should not be continuously modified
 * tick after tick, as that will mean the value is essentially never
 * synced properly to the other side.
 */
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

   /**
    * Add an element to the set, and mark the set as modified if the element was accepted by the set.
    *
    * @param element Element to add.
    * @return {@code true} if the element was added, {@code false} if the set already contained the element.
    */
   public boolean add(T element)
   {
      boolean added = getValueInternal().add(element);
      if (added)
         modify();
      return added;
   }

   /**
    * Remove an element from the set, and mark the set as modified if it contained the element.
    *
    * @param object Element to remove.
    * @return {@code true} if the element was removed, {@code false} if the set did not contain the element.
    */
   public boolean remove(T object)
   {
      boolean removed = getValueInternal().remove(object);
      if (removed)
         modify();
      return removed;
   }

   /**
    * Add all elements of a collection to the set, and mark the set as modified if it was changed.
    * If the set changes through this method, it is marked as modified.
    *
    * @param collection Collection of elements to add to the set.
    * @return {@code true} if the set was changed through this addition, {@code false} otherwise.
    */
   public boolean addAll(Collection<? extends T> collection)
   {
      boolean changed = getValueInternal().addAll(collection);
      if (changed)
         modify();
      return changed;
   }

   /**
    * Remove all elements from the set that were not contained in the collection.
    * If the set changes through this method, it is marked as modified.
    *
    * @param collection Collection of elements to keep.
    * @return {@code true} if the set was changed through this method, {@code false} otherwise.
    */
   public boolean retainAll(Collection<T> collection)
   {
      boolean changed = getValueInternal().retainAll(collection);
      if (changed)
         modify();
      return changed;
   }

   /**
    * Remove all elements from the set that are contained in the collection.
    * If the set changes through this method, it is marked as modified.
    *
    * @param collection Collection of elements to remove.
    * @return {@code true} if the set was changed through this method, {@code false} otherwise.
    */
   public boolean removeAll(Collection<T> collection)
   {
      boolean changed = getValueInternal().removeAll(collection);
      if (changed)
         modify();
      return changed;
   }

   /**
    * Clear the set, and mark it as modified if elements were removed.
    */
   public void clear()
   {
      if (!getValueInternal().isEmpty())
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
