package us.ihmc.robotics.lists;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

/**
 * {@code QuickSort} offers a garbage free implementation of the
 * <a href="https://en.wikipedia.org/wiki/Quicksort#Choice_of_pivot">Quicksort</a> algorithm for
 * sorting the elements of a list or an array.
 */
public class QuickSort
{
   private static final Random random = new Random();

   private QuickSort()
   {
      // Disallow construction
   }

   /**
    * Sorts the specified array of objects according to the order induced by the specified
    * comparator.
    * <p>
    * The <a href="https://en.wikipedia.org/wiki/Quicksort#Choice_of_pivot">Quicksort</a> algorithm
    * is used.
    * </p>
    *
    * @param <T>        type of elements the array contains.
    * @param array      the array to be sorted. Modified.
    * @param comparator used to compare any two elements of the array.
    */
   public static <T> void sort(T[] array, Comparator<T> comparator)
   {
      sort(array, comparator, 0, array.length - 1);
   }

   /**
    * Sorts the region [{@code startIndex}; {@code endIndex}] of the specified array of objects
    * according to the order induced by the specified comparator.
    * <p>
    * The <a href="https://en.wikipedia.org/wiki/Quicksort#Choice_of_pivot">Quicksort</a> algorithm
    * is used.
    * </p>
    *
    * @param <T>        type of elements the array contains.
    * @param array      the array to be sorted. Modified.
    * @param comparator used to compare any two elements of the array.
    * @param startIndex first index from which the elements are to be sorted.
    * @param endIndex   last index after which the elements are to not be sorted.
    */
   public static <T> void sort(T[] array, Comparator<T> comparator, int startIndex, int endIndex)
   {
      if (endIndex <= startIndex)
         return;

      // Choose the pivot randomly
      int pivotIndex = random.nextInt(endIndex - startIndex) + startIndex;

      /*
       * Partitioning:
       * // @formatter:off
       * +------------------------+
       * |  <= pivot  |  > pivot  |
       * +------------------------+
       *              ^
       *              |
       *          pivotIndex
       * // @formatter:on
       */
      pivotIndex = partition(array, comparator, startIndex, endIndex, pivotIndex);

      // Recurse on each side of the pivot
      sort(array, comparator, startIndex, pivotIndex - 1);
      sort(array, comparator, pivotIndex + 1, endIndex);
   }

   /**
    * Sorts the specified list of objects according to the order induced by the specified
    * comparator.
    * <p>
    * The <a href="https://en.wikipedia.org/wiki/Quicksort#Choice_of_pivot">Quicksort</a> algorithm
    * is used.
    * </p>
    *
    * @param <T>        type of elements the list contains.
    * @param list       the list to be sorted. Modified.
    * @param comparator used to compare any two elements of the list.
    */
   public static <T> void sort(List<? extends T> list, Comparator<T> comparator)
   {
      sort(list, comparator, 0, list.size() - 1);
   }

   /**
    * Sorts the region [{@code startIndex}; {@code endIndex}] of the specified list of objects
    * according to the order induced by the specified comparator.
    * <p>
    * The <a href="https://en.wikipedia.org/wiki/Quicksort#Choice_of_pivot">Quicksort</a> algorithm
    * is used.
    * </p>
    *
    * @param <T>        type of elements the list contains.
    * @param list       the list to be sorted. Modified.
    * @param comparator used to compare any two elements of the list.
    * @param startIndex first index from which the elements are to be sorted.
    * @param endIndex   last index after which the elements are to not be sorted.
    */
   public static <T> void sort(List<? extends T> list, Comparator<T> comparator, int startIndex, int endIndex)
   {
      if (endIndex <= startIndex)
         return;

      // Choose the pivot randomly
      int pivotIndex = random.nextInt(endIndex - startIndex) + startIndex;

      /*
       * Partitioning:
       * // @formatter:off
       * +------------------------+
       * |  <= pivot  |  > pivot  |
       * +------------------------+
       *              ^
       *              |
       *          pivotIndex
       * // @formatter:on
       */
      pivotIndex = partition(list, comparator, startIndex, endIndex, pivotIndex);

      // Recurse on each side of the pivot
      sort(list, comparator, startIndex, pivotIndex - 1);
      sort(list, comparator, pivotIndex + 1, endIndex);
   }

   /**
    * Partition the array around the pivot:
    *
    * <pre>
    * +------------------------+
    * |  <= pivot  |  > pivot  |
    * +------------------------+
    *              ^
    *              |
    *          pivotIndex
    * </pre>
    *
    * @param <T>        type of elements the array contains.
    * @param array      the array to be partitioned around the pivot. Modified.
    * @param comparator used to compare any two elements of the array.
    * @param startIndex first index of the array's region to be partitioned.
    * @param endIndex   last index of the array's region to be partitioned.
    * @param pivotIndex the initial index of the object that the rest of the array's region will be
    *                   compared to.
    * @return the new index of the pivot element.
    */
   private static <T> int partition(T[] array, Comparator<T> comparator, int startIndex, int endIndex, int pivotIndex)
   {
      T pivot = array[pivotIndex];

      // Push the pivot to the endIndex
      swap(array, pivotIndex, endIndex);

      // Index where the array is partitioned between smaller and larger elements than the pivot
      int partitionIndex = startIndex;

      for (int i = startIndex; i < endIndex; i++)
      {
         if (comparator.compare(array[i], pivot) <= 0)
         { // Element is lower or equal to the pivot: it is part of the first partition.
            swap(array, i, partitionIndex++);
         }
      }

      // Move the pivot back to the partition index.
      swap(array, endIndex, partitionIndex);

      return partitionIndex;
   }

   /**
    * Partition the list around the pivot:
    *
    * <pre>
    * +------------------------+
    * |  <= pivot  |  > pivot  |
    * +------------------------+
    *              ^
    *              |
    *          pivotIndex
    * </pre>
    *
    * @param <T>        type of elements the list contains.
    * @param list       the list to be partitioned around the pivot. Modified.
    * @param comparator used to compare any two elements of the list.
    * @param startIndex first index of the list's region to be partitioned.
    * @param endIndex   last index of the list's region to be partitioned.
    * @param pivotIndex the initial index of the object that the rest of the list's region will be
    *                   compared to.
    * @return the new index of the pivot element.
    */
   private static <T> int partition(List<? extends T> list, Comparator<T> comparator, int startIndex, int endIndex, int pivotIndex)
   {
      T pivot = list.get(pivotIndex);

      // Push the pivot to the endIndex
      Collections.swap(list, pivotIndex, endIndex);

      // Index where the array is partitioned between smaller and larger elements than the pivot
      int partitionIndex = startIndex;

      for (int i = startIndex; i < endIndex; i++)
      {
         if (comparator.compare(list.get(i), pivot) <= 0)
         { // Element is lower or equal to the pivot: it is part of the first partition.
            Collections.swap(list, i, partitionIndex);
            partitionIndex++;
         }
      }

      // Move the pivot back to the partition index.
      Collections.swap(list, endIndex, partitionIndex);

      return partitionIndex;
   }

   /**
    * Swaps {@code array[i]} with {@code array[j]}
    *
    * @param <T>   type of elements the array contains.
    * @param array the array in which two elements are to be swapped. Modified.
    * @param i     index of the first element to swap.
    * @param j     index of the second element to swap.
    */
   private static <T> void swap(T[] array, int i, int j)
   {
      T temp = array[i];
      array[i] = array[j];
      array[j] = temp;
   }
}
