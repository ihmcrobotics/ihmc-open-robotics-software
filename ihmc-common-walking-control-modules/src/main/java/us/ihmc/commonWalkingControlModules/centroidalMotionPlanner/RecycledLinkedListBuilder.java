package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import java.lang.reflect.Array;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.lists.GenericTypeBuilder;

/**
 * Helps in creating link lists that can be recycled
 * This is done through the nested class {@code RecycledLinkedListEntry} which has a private constructor
 * In order to insert / remove elements from the linked list the builder must be used
 * @author Apoorv S
 * @param <T>
 */
public class RecycledLinkedListBuilder<T> implements List<T>
{
   public class RecycledLinkedListEntry<T>
   {
      final T element;
      private RecycledLinkedListEntry<T> next;
      private RecycledLinkedListEntry<T> previous;

      // Only allow the builder to add or remove entries
      private RecycledLinkedListEntry(T element)
      {
         this.element = element;
      }

      public RecycledLinkedListEntry<T> getNext()
      {
         return next;
      }

      public RecycledLinkedListEntry<T> getPrevious()
      {
         return previous;
      }
   }

   private RecycledLinkedListEntry<T> firstEntry;
   private RecycledLinkedListEntry<T> lastEntry;
   private final GenericTypeBuilder<T> builder;
   private int size;
   private int freeEntrySize;
   private RecycledLinkedListEntry<T>[] freeEntries;

   private static final int DEFAULT_INITIAL_CAPACITY = 1;

   public RecycledLinkedListBuilder(GenericTypeBuilder<T> builder)
   {
      this(DEFAULT_INITIAL_CAPACITY, builder);
   }

   public RecycledLinkedListBuilder(Class<T> clazz)
   {
      this(DEFAULT_INITIAL_CAPACITY, clazz);
   }

   public RecycledLinkedListBuilder(int initialCapacity, Class<T> clazz)
   {
      this(initialCapacity, GenericTypeBuilder.createBuilderWithEmptyConstructor(clazz));
   }

   public RecycledLinkedListBuilder(int initialCapacity, GenericTypeBuilder<T> builder)
   {
      firstEntry = null;
      lastEntry = null;
      this.builder = builder;
      freeEntries = createEntryArray(initialCapacity);
      populateFreeEntries();
      size = 0;
   }

   @SuppressWarnings("unchecked")
   private RecycledLinkedListBuilder<T>.RecycledLinkedListEntry<T>[] createEntryArray(int initialCapacity)
   {
      return (RecycledLinkedListBuilder<T>.RecycledLinkedListEntry<T>[]) Array.newInstance(RecycledLinkedListEntry.class, initialCapacity);
   }

   public void clear()
   {
      RecycledLinkedListEntry<T> entry = firstEntry;
      while (entry != null)
      {
         RecycledLinkedListEntry<T> nextEntry = entry.getNext();
         entry.next = null;
         entry.previous = null;
         storeFreeEntry(entry);
         entry = nextEntry;
      }
      this.firstEntry = null;
      this.lastEntry = null;
      this.size = 0;
   }

   public RecycledLinkedListEntry<T> getFirstEntry()
   {
      return firstEntry;
   }

   public RecycledLinkedListEntry<T> getOrCreateFirstEntry()
   {
      if (firstEntry != null)
         return firstEntry;
      firstEntry = getFreeEntry();
      size++;
      return lastEntry = firstEntry;
   }

   public RecycledLinkedListEntry<T> getOrCreateLastEntry()
   {
      if (lastEntry != null)
         return lastEntry;
      else
         return getOrCreateFirstEntry();
   }

   public RecycledLinkedListEntry<T> getLastEntry()
   {
      return lastEntry;
   }

   public RecycledLinkedListEntry<T> insertAfter(RecycledLinkedListEntry<T> entry)
   {
      RecycledLinkedListEntry<T> newEntry = getFreeEntry();
      newEntry.previous = entry;
      newEntry.next = entry.next;

      entry.next = newEntry;
      if (newEntry.next != null)
         newEntry.next.previous = newEntry;
      else
         lastEntry = newEntry;
      size++;
      return newEntry;
   }

   public RecycledLinkedListEntry<T> insertBefore(RecycledLinkedListEntry<T> entry)
   {
      RecycledLinkedListEntry<T> newEntry = getFreeEntry();
      newEntry.previous = entry.previous;
      newEntry.next = entry;

      entry.previous = newEntry;
      if (newEntry.previous != null)
         newEntry.previous.next = newEntry;
      else
         firstEntry = newEntry;
      size++;
      return newEntry;
   }

   public RecycledLinkedListEntry<T> insertAtEnd()
   {
      if (lastEntry != null)
         return insertAfter(lastEntry);
      else
         return getOrCreateLastEntry();
   }

   public RecycledLinkedListEntry<T> insertAtBeginning()
   {
      if (firstEntry != null)
         return insertBefore(firstEntry);
      else
         return getOrCreateFirstEntry();
   }

   public void remove(RecycledLinkedListEntry<T> entryToRemove)
   {
      if (entryToRemove != lastEntry)
         entryToRemove.next.previous = entryToRemove.previous;
      else
         lastEntry = entryToRemove.previous;

      if (entryToRemove != firstEntry)
         entryToRemove.previous.next = entryToRemove.next;
      else
         firstEntry = entryToRemove.next;

      entryToRemove.next = null;
      entryToRemove.previous = null;
      size--;
      storeFreeEntry(entryToRemove);
   }

   public int getSize()
   {
      return size;
   }

   private void storeFreeEntry(RecycledLinkedListEntry<T> entryToStore)
   {
      if (freeEntrySize == freeEntries.length)
      {
         RecycledLinkedListEntry<T>[] newFreeEnteriesArray = createEntryArray(freeEntries.length * 2);
         // Store references to the existing free entries
         for (int i = 0; i < freeEntrySize; i++)
            newFreeEnteriesArray[i] = freeEntries[i];
         freeEntries = newFreeEnteriesArray;
      }
      freeEntries[freeEntrySize++] = entryToStore;
   }

   private RecycledLinkedListBuilder<T>.RecycledLinkedListEntry<T> getFreeEntry()
   {
      if (freeEntrySize == 0)
         populateFreeEntries();
      RecycledLinkedListBuilder<T>.RecycledLinkedListEntry<T> freeEntry = freeEntries[--freeEntrySize];
      freeEntries[freeEntrySize] = null;
      return freeEntry;
   }

   private void populateFreeEntries()
   {
      for (int i = 0; i < freeEntries.length; i++)
         if (freeEntries[i] == null)
            freeEntries[i] = new RecycledLinkedListEntry<T>(builder.newInstance());
      freeEntrySize = freeEntries.length;
   }

   @Override
   public boolean add(T arg0)
   {
      throw new RuntimeException("Cannot add external elements to recycling list. Use the the insert methods");
   }

   @Override
   public void add(int arg0, T arg1)
   {
      throw new RuntimeException("Cannot add external elements to recycling list. Use the the insert methods");
   }

   @Override
   public boolean addAll(Collection<? extends T> arg0)
   {
      throw new RuntimeException("Cannot add external elements to recycling list. Use the the insert methods");
   }

   @Override
   public boolean addAll(int arg0, Collection<? extends T> arg1)
   {
      throw new RuntimeException("Cannot add external elements to recycling list. Use the the insert methods");
   }

   @Override
   public boolean contains(Object arg0)
   {

      for (RecycledLinkedListEntry<T> entry = getFirstEntry(); entry != null; entry = entry.getNext())
      {
         if (entry.element == arg0)
            return true;
      }
      return false;
   }

   @Override
   public boolean containsAll(Collection<?> arg0)
   {
      boolean ans = true;
      Iterator<?> iterator = arg0.iterator();
      while (ans)
      {
         Object objectToCheck = iterator.next();
         ans &= contains(objectToCheck);
      }
      return ans;
   }

   @Override
   public T get(int arg0)
   {
      RecycledLinkedListEntry<T> entry = getFirstEntry();
      for (; arg0 > 0; arg0--)
         entry = entry.getNext();
      return entry.element;
   }

   @Override
   public int indexOf(Object arg0)
   {
      throw new RuntimeException("Linked list does not contain indices");
   }

   @Override
   public boolean isEmpty()
   {
      return (size == 0);
   }

   @Override
   public Iterator<T> iterator()
   {
      throw new RuntimeException("Use the entries of the linked list to iterate through the linked list");
   }

   @Override
   public int lastIndexOf(Object arg0)
   {
      throw new RuntimeException("Linked list does not contain indices");
   }

   @Override
   public ListIterator<T> listIterator()
   {
      throw new RuntimeException("Use the entries of the linked list to iterate through the linked list");
   }

   @Override
   public ListIterator<T> listIterator(int arg0)
   {
      throw new RuntimeException("Use the entries of the linked list to iterate through the linked list");
   }

   @Override
   public boolean remove(Object arg0)
   {
      throw new RuntimeException("Cannot remove non linked list objects");
   }

   @Override
   public T remove(int arg0)
   {
      throw new RuntimeException("Can only remove objects through the linked list remove method");
   }

   @Override
   public boolean removeAll(Collection<?> arg0)
   {
      throw new RuntimeException("Can only remove objects through the linked list remove method");
   }

   @Override
   public boolean retainAll(Collection<?> arg0)
   {
      throw new RuntimeException("Can only remove objects through the linked list remove method");
   }

   @Override
   public T set(int arg0, T arg1)
   {
      throw new RuntimeException("Cannot set elements inside the list. Get the element and set using an appropriate method");
   }

   @Override
   public int size()
   {
      return getSize();
   }

   @Override
   public List<T> subList(int arg0, int arg1)
   {
      throw new RuntimeException("Unimplemented method");
   }

   @Override
   public Object[] toArray()
   {
      throw new RuntimeException("Unimplemented method");
   }

   @Override
   public <T> T[] toArray(T[] a)
   {
      throw new RuntimeException("Unimplemented method");
   }
}