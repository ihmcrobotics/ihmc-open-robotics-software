package us.ihmc.robotics.lists;

import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Iterator;

/**
 * This is an implementation of ArrayDeque that will reuse objects, making it more allocation efficient.
 *
 * @param <T> the type of object in this deque must extend {@link #Settable}.
 */
public class RecyclingArrayDeque<T extends Settable<T>> extends ArrayDeque<T>
{
   private static final long serialVersionUID = 8118722036566615731L;
   private static final int defaultNumberOfElements = 16;

   private final GenericTypeBuilder<T> typeBuilder;
   private final ArrayDeque<T> unusedObjects;

   public RecyclingArrayDeque(GenericTypeBuilder<T> typeBuilder)
   {
      this(defaultNumberOfElements, typeBuilder);
   }

   public RecyclingArrayDeque(Class<T> objectClass)
   {
      this(defaultNumberOfElements, GenericTypeBuilder.createBuilderWithEmptyConstructor(objectClass));
   }

   public RecyclingArrayDeque(int numElements, Class<T> objectClass)
   {
      this(numElements, GenericTypeBuilder.createBuilderWithEmptyConstructor(objectClass));
   }

   public RecyclingArrayDeque(int numElements, GenericTypeBuilder<T> typeBuilder)
   {
      super(numElements);
      this.typeBuilder = typeBuilder;
      unusedObjects = new ArrayDeque<>(numElements);
      for (int i = 0; i < numElements; i++)
         unusedObjects.add(typeBuilder.newInstance());
   }

   /** {@inheritDoc} */
   @Override
   public int size()
   {
      return super.size();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isEmpty()
   {
      return super.isEmpty();
   }

   /**
    * Add an object at the front of this deque and return it. Because we are recycling objects the object may have data in it.
    * @return the new empty object.
    */
   public T addFirst()
   {
      T newObject = getOrCreateUnusedObject();
      super.addFirst(newObject);
      return newObject;
   }

   /**
    * Add an object at the end of this deque and return it. Because we are recycling objects the object may have data in it.
    * @return the new empty object.
    */
   public T addLast()
   {
      T newObject = getOrCreateUnusedObject();
      super.addLast(newObject);
      return newObject;
   }

   /** {@inheritDoc} */
   @Override
   public boolean add(T newObject)
   {
      return super.add(copyAndReturnLocalObject(newObject));
   }

   /**
    * The deque will be empty after this call returns.
    * The removed elements are saved in a local buffer for recycling purpose to prevent garbage generation.
    */
   @Override
   public void clear()
   {
      while (!super.isEmpty())
         unusedObjects.add(super.pollFirst());
   }

   /** {@inheritDoc} */
   @Override
   public void addFirst(T newObject)
   {
      super.addFirst(copyAndReturnLocalObject(newObject));
   }

   /** {@inheritDoc} */
   @Override
   public void addLast(T newObject)
   {
      super.addLast(copyAndReturnLocalObject(newObject));
   }

   /** {@inheritDoc} */
   @Override
   public void push(T newObject)
   {
      super.push(copyAndReturnLocalObject(newObject));
   }

   /** {@inheritDoc} */
   @Override
   public boolean offerFirst(T newObject)
   {
      return super.offerFirst(copyAndReturnLocalObject(newObject));
   }

   /** {@inheritDoc} */
   @Override
   public boolean offerLast(T newObject)
   {
      return super.offerLast(copyAndReturnLocalObject(newObject));
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T removeFirst()
   {
      T objectToReturn = super.removeFirst();
      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T removeLast()
   {
      T objectToReturn = super.removeLast();
      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T pollFirst()
   {
      T objectToReturn = super.pollFirst();
      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T pollLast()
   {
      T objectToReturn = super.pollLast();
      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T getFirst()
   {
      T objectToReturn = super.getFirst();
//      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T getLast()
   {
      T objectToReturn = super.getLast();
//      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T peekFirst()
   {
      T objectToReturn = super.peekFirst();
//      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T peekLast()
   {
      T objectToReturn = super.peekLast();
//      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T remove()
   {
      T objectToReturn = super.remove();
      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T poll()
   {
      T objectToReturn = super.poll();
      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T element()
   {
      T objectToReturn = super.element();
//      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T peek()
   {
      T objectToReturn = super.peek();
//      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   /**
    * Warning: The returned element will be reused and modified by this deque when adding a new element.
    * {@inheritDoc}
    */
   @Override
   public T pop()
   {
      T objectToReturn = super.pop();
      unusedObjects.add(objectToReturn);
      return objectToReturn;
   }

   private T copyAndReturnLocalObject(T objectToCopy)
   {
      T localObject = getOrCreateUnusedObject();
      localObject.set(objectToCopy);
      return localObject;
   }

   private T getOrCreateUnusedObject()
   {
      if (unusedObjects.isEmpty())
         return typeBuilder.newInstance();
      else
         return unusedObjects.poll();
   }

   @Override
   public String toString()
   {
      Iterator<T> iterator = super.iterator();
      if (!iterator.hasNext())
         return "[]";

      StringBuilder sb = new StringBuilder();
      sb.append('[');
      for (;;)
      {
         T nextObject = iterator.next();
         sb.append(nextObject);
         if (!iterator.hasNext())
            return sb.append(']').toString();
         sb.append(',').append(' ');
      }
   }

   /** Unsupported operation. */
   @Override
   public RecyclingArrayDeque<T> clone()
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean remove(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean contains(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public Iterator<T> iterator()
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public Object[] toArray()
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public <T> T[] toArray(T[] a)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean containsAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean addAll(Collection<? extends T> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean removeAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean retainAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean removeFirstOccurrence(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean removeLastOccurrence(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public boolean offer(T e)
   {
      throw new UnsupportedOperationException();
   }

   /** Unsupported operation. */
   @Override
   public Iterator<T> descendingIterator()
   {
      throw new UnsupportedOperationException();
   }
}
