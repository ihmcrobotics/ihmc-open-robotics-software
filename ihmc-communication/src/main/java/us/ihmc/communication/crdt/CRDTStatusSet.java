package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class CRDTStatusSet<T> extends CRDTStatusMutableField<Set<T>>
{
   private final Supplier<Set<T>> newSetSupplier;

   public CRDTStatusSet(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      this(sideThatCanModify, crdtInfo, HashSet::new);
   }

   public CRDTStatusSet(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, Supplier<Set<T>> newSetSupplier)
   {
      super(sideThatCanModify, crdtInfo, newSetSupplier);
      this.newSetSupplier = newSetSupplier;
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

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
//         Set<T> before = getCopy();
         valueConsumer.accept(getValueInternal());
//         if (!getValueInternal().containsAll(before) || !before.containsAll(getValueInternal()))
//            markHasStatus();
      }
   }
}
