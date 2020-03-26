package us.ihmc.robotics.lists;

import java.lang.reflect.Array;
import java.util.ConcurrentModificationException;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import us.ihmc.commons.lists.RecyclingArrayList;

/**
 * Represents a buffer which capacity can be fixed to certain capacity, the but will grow up to
 * capacity and then will go back to the first element added when attempting to add a new element.
 * <p>
 * This implementation can be useful to keep a history of the N most recent elements of some data
 * where N is the capacity of this buffer.
 * </p>
 * <p>
 * Note that as {@link RecyclingArrayList}, this implementation recycles internal memory.
 * </p>
 * 
 * @author Sylvain Bertrand
 * @param <T>
 */
@SuppressWarnings("unchecked")
public class RingBuffer<T> implements Iterable<T>
{
   private int currentIndex = -1;
   private int capacity;
   private boolean isBufferFull = false;

   private transient int modCount = 0;

   private T[] buffer;
   private final BiConsumer<T, T> copier;
   private final Supplier<T> allocator;
   private final Class<T> elementType;

   /**
    * Create a new empty ring buffer.
    * 
    * @param capacity  the initial capacity of this ring buffer.
    * @param allocator builder used to instantiate this buffer's elements.
    */
   public RingBuffer(int capacity, Supplier<T> allocator)
   {
      this(capacity, allocator, null);
   }

   /**
    * Create a new empty ring buffer.
    * 
    * @param capacity  the initial capacity of this ring buffer.
    * @param allocator builder used to instantiate this buffer's elements.
    * @param copier    coper required to use {@link #add(Object)}.
    */
   public RingBuffer(int capacity, Supplier<T> allocator, BiConsumer<T, T> copier)
   {
      this.allocator = allocator;
      if (capacity <= 0)
         throw new IllegalArgumentException("Cannot instantiate a buffer with a size of zero or less.");
      this.capacity = capacity;
      this.copier = copier;

      T firstElement = allocator.get();
      elementType = (Class<T>) firstElement.getClass();
      buffer = (T[]) new Object[capacity];
      buffer[0] = firstElement;
      for (int i = 1; i < capacity; i++)
         buffer[i] = allocator.get();
   }

   /**
    * Resizes this buffer while preserving order and state of the previously added elements when
    * possible.
    * 
    * @param newCapacity the new capacity for this ring buffer.
    */
   public void changeCapacity(int newCapacity)
   {
      if (capacity == newCapacity)
         return;

      modCount++;

      T[] newBuffer = (T[]) new Object[newCapacity];

      int size = size();

      if (newCapacity > size)
      {
         int index = isBufferFull ? currentIndex + 1 : 0;

         for (int offset = 0; offset < Math.min(capacity, newCapacity); offset++)
         {
            newBuffer[offset] = (T) buffer[index % capacity];
            index++;
         }
         currentIndex = size() - 1;
         isBufferFull = false;

         for (int i = capacity; i < newCapacity; i++)
            newBuffer[i] = allocator.get();
      }
      else
      {
         int index = currentIndex;

         for (int offset = newCapacity - 1; offset >= 0; offset--)
         {
            if (index < 0)
               index += capacity;
            newBuffer[offset] = buffer[index];
            index--;
         }
         currentIndex = newCapacity - 1;
         isBufferFull = true;
      }

      buffer = newBuffer;
      capacity = newCapacity;
   }

   /**
    * Clears this buffer.
    * <p>
    * Note that the internal memory is preserved to avoid garbage generation.
    * </p>
    */
   public void reset()
   {
      modCount++;
      currentIndex = -1;
      isBufferFull = false;
   }

   /**
    * Adds a new element to this buffer, increasing its current size by 1 if not full, otherwise
    * dropping the oldest element that was added.
    * 
    * @return the new element added to this buffer.
    */
   public T add()
   {
      modCount++;
      currentIndex++;

      if (currentIndex >= capacity)
      {
         isBufferFull = true;
         currentIndex = 0;
      }

      return (T) buffer[currentIndex];
   }

   /**
    * Adds a new element to this buffer and sets it to {@code newElementToCopy}.
    * 
    * @param newElementToCopy the value for the new element.
    * @see #add()
    * @throws UnsupportedOperationException if this buffer was not given a copier.
    */
   public void add(T newElementToCopy)
   {
      if (copier == null)
         throw new UnsupportedOperationException("Unable to copy new data to internal element without a copier. Use add() instead.");

      copier.accept(add(), newElementToCopy);
   }

   /**
    * Gets the oldest element that was added to this buffer.
    * 
    * @return the oldest element, or {@code null} if this buffer is empty.
    */
   public T getFirst()
   {
      if (currentIndex == -1)
         return null;

      if (!isBufferFull)
         return (T) buffer[0];

      return (T) buffer[(currentIndex + 1) % capacity];
   }

   /**
    * Gets the newest element that was added to this buffer.
    * 
    * @return the newest element, or {@code null} if this buffer is empty.
    */
   public T getLast()
   {
      if (currentIndex == -1)
         return null;
      else
         return (T) buffer[currentIndex];
   }

   /**
    * Gets the <tt>N</tt><sup>th</sup> oldest element that was added to this buffer.
    * 
    * @param offsetFromFirst offset from the oldest element, i.e. <tt>N</tt> above.
    * @return the <tt>N</tt><sup>th</sup> oldest element.
    * @throws IndexOutOfBoundsException if {@code offsetFromFirst} &notin; [0, <tt>this.size()</tt>[.
    */
   public T getFromFirst(int offsetFromFirst)
   {
      if (offsetFromFirst >= size() || offsetFromFirst < 0)
         throw new IndexOutOfBoundsException(outOfBoundsMessage(offsetFromFirst));

      if (!isBufferFull)
         return (T) buffer[offsetFromFirst];

      int index = currentIndex + 1 + offsetFromFirst;
      return (T) buffer[index % capacity];
   }

   /**
    * Gets the <tt>N</tt><sup>th</sup> newest element that was added to this buffer.
    * 
    * @param offsetFromLast offset from the newest element, i.e. <tt>N</tt> above.
    * @return the <tt>N</tt><sup>th</sup> newest element.
    * @throws IndexOutOfBoundsException if {@code offsetFromLast} &notin; [0, <tt>this.size()</tt>[.
    */
   public T getFromLast(int offsetFromLast)
   {
      if (offsetFromLast >= size())
         throw new IndexOutOfBoundsException(outOfBoundsMessage(offsetFromLast));

      int index = currentIndex - offsetFromLast;
      if (index < 0 && isBufferFull)
         index += capacity;
      return (T) buffer[index];
   }

   /**
    * Gets whether this buffer has reached its capacity or not.
    * 
    * @return {@code true} is this ring buffer is full, {@code false} otherwise.
    */
   public boolean isBufferFull()
   {
      return isBufferFull;
   }

   /**
    * Returns <tt>true</tt> if this list contains no elements.
    *
    * @return <tt>true</tt> if this list contains no elements
    */
   public boolean isEmpty()
   {
      return currentIndex == -1;
   }

   /**
    * Returns the current size of this ring buffer.
    * <p>
    * When this buffer is full, this method returns <tt>capacity</tt>.
    * </p>
    * 
    * @return the number of element in this buffer.
    */
   public int size()
   {
      if (isBufferFull)
         return capacity;
      else
         return currentIndex + 1;
   }

   /**
    * Returns the number of elements this buffer can hold onto.
    * 
    * @return this buffer's capacity.
    */
   public int capacity()
   {
      return capacity;
   }

   /**
    * Creates a array of this buffer's elements ordered from the oldest to the newest.
    * 
    * @return the current buffer's elements from oldest to newest.
    */
   public T[] toArrayFromFirstToLast()
   {
      int size = size();
      T[] array = (T[]) Array.newInstance(elementType, size);

      int index = isBufferFull ? currentIndex + 1 : 0;

      for (int offset = 0; offset < size; offset++)
      {
         array[offset] = (T) buffer[index % capacity];
         index++;
      }
      return array;
   }

   /**
    * Creates a array of this buffer's elements ordered from the newest to the oldest.
    * 
    * @return the current buffer's elements from newest to oldest.
    */
   public T[] toArrayFromLastToFirst()
   {
      int size = size();
      T[] array = (T[]) Array.newInstance(elementType, size);

      int index = currentIndex;

      for (int offset = 0; offset < size; offset++)
      {
         if (index < 0)
            index += capacity;

         array[offset] = (T) buffer[index];
         index--;
      }
      return array;
   }

   /**
    * Returns an iterator over the elements of this ring buffer starting from the oldest element and
    * ending at the newest element.
    */
   @Override
   public Iterator<T> iterator()
   {
      return new RingBufferIterator(false);
   }

   /**
    * Returns an iterator over the elements of this ring buffer starting from the newest element and
    * ending at the oldest element.
    */
   public Iterator<T> reverseIterator()
   {
      return new RingBufferIterator(true);
   }

   private String outOfBoundsMessage(int index)
   {
      return "Index: " + index + ", Size: " + size();
   }

   private class RingBufferIterator implements Iterator<T>
   {
      /**
       * Index of element to be returned by subsequent call to next.
       */
      int cursor = 0;

      /**
       * The modCount value that the iterator believes that the buffer should have. If this expectation is
       * violated, the iterator has detected concurrent modification.
       */
      int expectedModCount = modCount;

      /**
       * Indicates the direction of this iterator:
       * <ul>
       * <li>{@code reverse == false}: iterates from oldest to newest element.
       * <li>{@code reverse == true}: iterates from newest to oldest element.
       * </ul>
       */
      private final boolean reverse;

      private RingBufferIterator(boolean reverse)
      {
         this.reverse = reverse;
      }

      public boolean hasNext()
      {
         return cursor != size();
      }

      public T next()
      {
         checkForComodification();
         try
         {
            int offset = cursor;
            T next = reverse ? getFromLast(offset) : getFromFirst(offset);
            cursor = offset + 1;
            return next;
         }
         catch (IndexOutOfBoundsException e)
         {
            checkForComodification();
            throw new NoSuchElementException();
         }
      }

      public void remove()
      {
         throw new UnsupportedOperationException("Removing elements from the buffer is not supported.");
      }

      final void checkForComodification()
      {
         if (modCount != expectedModCount)
            throw new ConcurrentModificationException();
      }
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof RingBuffer)
      {
         RingBuffer<?> other = (RingBuffer<?>) object;
         if (size() != other.size())
            return false;
         for (int i = 0; i < size(); i++)
         {
            if (!getFromFirst(i).equals(other.getFromFirst(i)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      if (isEmpty())
         return "Empty";

      StringBuilder sb = new StringBuilder();
      int size = size();
      sb.append("Size: ").append(size).append(", [");
      sb.append(getFirst().toString());
      for (int i = 1; i < size; i++)
         sb.append(',').append(' ').append(getFromFirst(i));
      sb.append(']');
      return sb.toString();
   }
}