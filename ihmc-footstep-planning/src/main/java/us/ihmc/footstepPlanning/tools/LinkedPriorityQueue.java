package us.ihmc.footstepPlanning.tools;

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

   private static <T> void siftUpUsingComparator(T elementToAdd, Node<T> tailToStart, Comparator<? super T> comparator)
   {

      Node<T> candidate = tailToStart;

      while (true)
      {
         if (comparator.compare(elementToAdd, candidate.item) >= 0)
            break;
         candidate = candidate.prev;
      }

      insertAfter(candidate, elementToAdd);
   }

   private static <E> void insertAfter(Node<E> node, E elementToInsert)
   {
      Node<E> nextToStore = null;
      if (node != null)
         nextToStore = node.next;
      node.next = new Node<>(node, elementToInsert, nextToStore);
   }

   private static <E> void insertBefore(Node<E> node, E elementToInsert)
   {
      Node<E> prevToStore = node.prev;
      node.prev = new Node<>(prevToStore, elementToInsert, node);
   }

   private static class Node<E> {
      E item;
      Node<E> next;
      Node<E> prev;

      Node(Node<E> prev, E element, Node<E> next) {
         this.item = element;
         this.next = next;
         this.prev = prev;
      }
   }
}
