package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Represents a set that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 *  * from unchecked modifications.
 * @param <T>
 */
public class CRDTStatusSet<T> extends CRDTStatusMutableField<Set<T>>
{
   private final Supplier<Set<T>> newSetSupplier;
   private final Set<T> readOnlySet;

   public CRDTStatusSet(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      this(sideThatCanModify, crdtInfo, HashSet::new);
   }

   public CRDTStatusSet(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, Supplier<Set<T>> newSetSupplier)
   {
      super(sideThatCanModify, crdtInfo, newSetSupplier);
      this.newSetSupplier = newSetSupplier;
      readOnlySet = Collections.unmodifiableSet(getValueInternal());
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   /**
    * Get a read-only instance of the set.
    * @return An unmodifiable instance of the set.
    */
   public Set<T> getReadOnly()
   {
      return readOnlySet;
   }

   /**
    * Get a copy of the set.
    * @return A modifiable copy of the set.
    */
   public Set<T> getCopy()
   {
      Set<T> copy = newSetSupplier.get();
      copy.addAll(getValueInternal());
      return copy;
   }

   public boolean add(T element)
   {
      checkActorCanModify();
      boolean added = getValueInternal().add(element);
      if (added)
         markHasStatus();
      return added;
   }

   public boolean remove(T object)
   {
      checkActorCanModify();
      boolean removed = getValueInternal().remove(object);
      if (removed)
         markHasStatus();
      return removed;
   }

   public boolean addAll(Collection<? extends T> collection)
   {
      checkActorCanModify();
      boolean changed = getValueInternal().addAll(collection);
      if (changed)
         markHasStatus();
      return changed;
   }

   public boolean retainAll(Collection<T> collection)
   {
      checkActorCanModify();
      boolean changed = getValueInternal().retainAll(collection);
      if (changed)
         markHasStatus();
      return changed;   }

   public boolean removeAll(Collection<T> collection)
   {
      checkActorCanModify();
      boolean changed = getValueInternal().removeAll(collection);
      if (changed)
         markHasStatus();
      return changed;
   }

   public void clear()
   {
      checkActorCanModifyAndMarkHasStatus();
      getValueInternal().clear();
   }

   public void fromMessage(Consumer<Set<T>> valueConsumer)
   {
      if (isModificationDisallowed())
      {
         valueConsumer.accept(getValueInternal());
      }
   }
}
