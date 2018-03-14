package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import java.lang.reflect.Array;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.lists.GenericTypeBuilder;

/**
 * Helps in creating link lists that can be recycled
 * This is done through the nested class {@code RecycledLinkedListEntry} which has a private constructor
 * In order to insert / remove elements from the linked list the builder must be used
 * @author Apoorv S
 * @param <T>
 */
public class RecycledLinkedListBuilder<T>
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

   private RecycledLinkedListEntry<T> firstElement;
   private RecycledLinkedListEntry<T> lastElement;
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
      firstElement = null;
      lastElement = null;
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

   public RecycledLinkedListEntry<T> getFirstEntry()
   {
      return firstElement;
   }

   public RecycledLinkedListEntry<T> getOrCreateFirstEntry()
   {
      if (firstElement != null)
         return firstElement;
      firstElement = getFreeEntry();
      size++;
      return lastElement = firstElement;
   }

   public RecycledLinkedListEntry<T> getOrCreateLastEntry()
   {
      if(lastElement != null)
         return lastElement;
      else
         return getOrCreateFirstEntry();
   }

   public RecycledLinkedListEntry<T> getLastEntry()
   {
      return lastElement;
   }

   public RecycledLinkedListEntry<T> insertAfter(RecycledLinkedListEntry<T> entry)
   {
      RecycledLinkedListEntry<T> newEntry = getFreeEntry();
      newEntry.previous = entry;
      newEntry.next = entry.next;

      entry.next = newEntry;
      if(newEntry.next != null)
         newEntry.next.previous = newEntry;
      else
         lastElement = newEntry;
      size++;
      return newEntry;
   }

   public RecycledLinkedListEntry<T> insertBefore(RecycledLinkedListEntry<T> entry)
   {
      RecycledLinkedListEntry<T> newEntry = getFreeEntry();
      newEntry.previous = entry.previous;
      newEntry.next = entry;

      entry.previous = newEntry;
      if(newEntry.previous != null)
         newEntry.previous.next = newEntry;
      else
         firstElement = newEntry;
      size++;
      return newEntry;
   }

   public void remove(RecycledLinkedListEntry<T> entryToRemove)
   {
      if(entryToRemove != lastElement)
         entryToRemove.next.previous = entryToRemove.previous;
      else
         lastElement = entryToRemove.previous;
      entryToRemove.previous.next = entryToRemove.next;
      
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
}
