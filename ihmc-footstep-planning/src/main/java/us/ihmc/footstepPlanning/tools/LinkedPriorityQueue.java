package us.ihmc.footstepPlanning.tools;

import us.ihmc.log.LogTools;

import java.util.*;

public class LinkedPriorityQueue<E>
{
   private Node<E> first;
   private Node<E> last;
   private int size = 0;

   /**
    * The comparator, or null if priority queue uses elements'
    * natural ordering.
    */
   @SuppressWarnings("serial") // Conditionally serializable
   private final Comparator<? super E> comparator;

   public LinkedPriorityQueue(Comparator<? super E> comparator)
   {
      this.comparator = comparator;
   }

   public void clear()
   {
      size = 0;
      first = null;
      last = null;
   }

   /**
    * Inserts the specified element into this priority queue.
    *
    * @return {@code true} (as specified by {@link Collection#add})
    * @throws ClassCastException if the specified element cannot be
    *         compared with elements currently in this priority queue
    *         according to the priority queue's ordering
    * @throws NullPointerException if the specified element is null
    */
   public boolean add(E e)
   {
      if (e == null)
         throw new NullPointerException();

      siftUp(e);
      size++;
      return true;
   }

   public E pollFirst()
   {
      E ret = first.item;
      first = first.next;
      if (first == null)
         last = null;
      else
         first.prev = null;
      size--;

      return ret;
   }

   public E peekFirst()
   {
      return first.item;
   }

   public E pollLast()
   {
      E ret = last.item;
      last = last.prev;
      last.next = null;
      size--;

      return ret;
   }

   public E peekLast()
   {
      return last.item;
   }

   public int size()
   {
      return size;
   }

   public boolean isEmpty()
   {
      return size < 1;
   }

   /**
    * Inserts item x at position k, maintaining heap invariant by
    * promoting x up the tree until it is greater than or equal to
    * its parent, or is the root.
    *
    * To simplify and speed up coercions and comparisons, the
    * Comparable and Comparator versions are separated into different
    * methods that are otherwise identical. (Similarly for siftDown.)
    *
    * @param key the position to fill
    * @param elementToAdd the item to insert
    */
   private void siftUp(E elementToAdd)
   {
      if (first == null || last == null)
         initialize(elementToAdd);
      else
         siftUpUsingComparator(elementToAdd, last, comparator);
   }

   private void initialize(E element)
   {
      Node<E> node = new Node<>(null, element, null);
      first = node;
      last = node;
   }

   private void siftUpUsingComparator(E elementToAdd, Node<E> tailToStart, Comparator<? super E> comparator)
   {
      Node<E> candidate = tailToStart;
      Node<E> next = tailToStart.next;

      while (true)
      {
         if (comparator.compare(elementToAdd, candidate.item) >= 0)
            break;
         next = candidate;
         candidate = candidate.prev;
         if (candidate == null)
            break;
      }

      if (candidate == null)
      {
         insertBefore(next, elementToAdd);
         first = next.prev;
      }
      else
      {
         insertAfter(candidate, elementToAdd);
         if (last == candidate)
            last = candidate.next;
      }
   }

   private static <E> void insertAfter(Node<E> node, E elementToInsert)
   {
      Node<E> nextToStore = null;
      if (node != null)
         nextToStore = node.next;
      Node<E> nodeToInsert = new Node<>(node, elementToInsert, nextToStore);

      if (node != null)
         node.next = nodeToInsert;
      if (nextToStore != null)
         nextToStore.prev = nodeToInsert;
   }

   private static <E> void insertBefore(Node<E> node, E elementToInsert)
   {
      Node<E> prevToStore = node.prev;
      Node<E> nodeToInsert = new Node<>(prevToStore, elementToInsert, node);
      if (prevToStore != null)
         prevToStore.next = nodeToInsert;
      node.prev = nodeToInsert;
   }

   private static class Node<E> {
      E item;
      public Node<E> next;
      public Node<E> prev;

      Node(Node<E> prev, E element, Node<E> next) {
         this.item = element;
         this.next = next;
         this.prev = prev;
      }
   }
}
