package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.commons.lists.ListWrappingIndexTools;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;

/**
 * A circular array list
 *
 * @param <E>
 */
public class CircularArrayList<E>
//      extends ArrayList<E> // uncomment to easily add methods
      implements Iterable<E>
{
   private ArrayList<E> list;

   public CircularArrayList()
   {
      list = new ArrayList<>();
   }

   public CircularArrayList(int initialCapacity)
   {
      list = new ArrayList<>(initialCapacity);
   }

   /**
    * Moves the zero point of the circular list by spin amount.
    */
   public void reindexTo(int spinAmount)
   {
      int wrappedSpinAmount = wrapIndex(spinAmount);

      ArrayList<E> reindexedList = new ArrayList<>(list.size());

      int i = 0;
      boolean startAdding = false;
      while (reindexedList.size() < list.size())
      {
         if (!startAdding && i == wrappedSpinAmount)
         {
            startAdding = true;
         }

         if (startAdding)
         {
            reindexedList.add(list.get(i));
         }

         i = getNextIndex(i);
      }

      list = reindexedList;
   }

   public int wrapIndex(int index)
   {
      return ListWrappingIndexTools.wrap(index, list);
   }

   public int getNextIndex(int index)
   {
      return ListWrappingIndexTools.next(index, list);
   }

   public int getPreviousIndex(int index)
   {
      return ListWrappingIndexTools.previous(index, list);
   }

   public E get(int index)
   {
      return ListWrappingIndexTools.getWrap(index, list);
   }

   public E getNext(int index)
   {
      return ListWrappingIndexTools.getNext(index, list);
   }

   public E getPrevious(int index)
   {
      return ListWrappingIndexTools.getPrevious(index, list);
   }

   public E set(int index, E element)
   {
      return ListWrappingIndexTools.setWrap(index, element, list);
   }

   public boolean add(E e)
   {
      return list.add(e);
   }

   public int size()
   {
      return list.size();
   }

   public ArrayList<E> getList()
   {
      return list;
   }

   @Override
   public Iterator<E> iterator()
   {
      return this.iterator();
   }

   public int indexOf(Object o)
   {
      return list.indexOf(o);
   }

   public boolean contains(Object o)
   {
      return list.contains(o);
   }

   public void sort(Comparator<? super E> c)
   {
      this.sort(c);
   }
}
