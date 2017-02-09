package us.ihmc.robotics.lists;

import java.util.ArrayList;
import java.util.List;

public class ListWrappingIndexTools
{
   /**
    * Wrap the given index such that 0 <= wrappedIndex < list.size().
    * @param index the index to wrap
    * @param list the list for which the index is to be used.
    * @return the wrapped index.
    */
   public static int wrap(int index, List<?> list)
   {
      index %= list.size();
      if (index < 0)
         index += list.size();
      return index;
   }

   /**
    * Safe increment that will always the next index that is inside [0, list.size() - 1].
    * @returns {@code (index + 1) % list.size()}.
    */
   public static int next(int index, List<?> list)
   {
      return wrap(index + 1, list);
   }

   /**
    * Safe increment that will always the previous index that is inside [0, list.size() - 1].
    * @returns {@code (index - 1 + list.size()) % list.size()}.
    */
   public static int previous(int index, List<?> list)
   {
      return wrap(index - 1, list);
   }

   /**
    * Calls {@link List#get(int)},
    * but first computes index %= list.size() such that the index is always inside [0, list.size() - 1].
    */
   public static <T> T getWrap(int index, List<T> list)
   {
      index = wrap(index, list);
      return list.get(index);
   }

   /**
    * Calls {@link List#set(int, T)},
    * but first computes index %= list.size() such that the index is always inside [0, list.size() - 1].
    */
   public static <T> T setWrap(int index, T newValue, List<T> list)
   {
      index = wrap(index, list);
      return list.set(index, newValue);
   }

   /**
    * Returns the element that is right after the given index.
    */
   public static <T> T getNext(int index, List<T> list)
   {
      return list.get(next(index, list));
   }

   /**
    * Returns the element that is right before the given index.
    */
   public static <T> T getPrevious(int index, List<T> list)
   {
      return list.get(previous(index, list));
   }

   /**
    * Returns the number of elements between startIndex (included) and endIndex (included).
    */
   public static int subLengthInclusive(int startIndex, int endIndex, List<?> list)
   {
      if (endIndex == startIndex)
         return 1;
      else if (endIndex > startIndex)
         return endIndex - startIndex + 1;
      else
         return (endIndex + list.size()) - startIndex + 1;
   }

   /**
    * Returns the number of elements between startIndex (excluded) and endIndex (excluded).
    */
   public static int subLengthExclusive(int startIndex, int endIndex, List<?> list)
   {
      if (endIndex == startIndex)
         return 0;
      else if (endIndex > startIndex)
         return endIndex - startIndex - 1;
      else
         return (endIndex + list.size()) - startIndex - 1;
   }

   /**
    * Returns the minimal distance between startIndex (excluded) and endIndex (excluded).
    * <p>
    * In other words, returns the minimum between {@code subLengthExclusive(startIndex, endIndex, list)} and  {@code subLengthExclusive(endIndex, startIndex, list)}.
    */
   public static int minDistanceExclusive(int startIndex, int endIndex, List<?> list)
   {
      if (endIndex == startIndex)
         return 0;
      else
         return Math.min(subLengthExclusive(startIndex, endIndex, list), subLengthExclusive(endIndex, startIndex, list));
   }

   /**
    * Returns a list that contains all the elements of the input from startIndex (included) to endIndex (included).
    */
   public static <T> List<T> subListInclusive(int startIndex, int endIndex, List<T> input)
   {
      List<T> output = new ArrayList<>();
   
      int outputLenth = subLengthInclusive(startIndex, endIndex, input);
   
      int i = startIndex;
      while (output.size() != outputLenth)
         output.add(getWrap(i++, input));
   
      return output;
   }

   /**
    * Returns a list that contains all the elements of the input from startIndex (excluded) to endIndex (excluded).
    */
   public static <T> List<T> subListExclusive(int startIndex, int endIndex, List<T> input)
   {
      List<T> output = new ArrayList<>();

      if (startIndex == endIndex)
         return output;
   
      int outputLenth = subLengthExclusive(startIndex, endIndex, input);
   
      int i = startIndex + 1;
      while (output.size() != outputLenth)
         output.add(getWrap(i++, input));
   
      return output;
   }

   /**
    * Removes all the elements from the list from startIndex (included) to endIndex (included).
    */
   public static int removeAllInclusive(int startIndex, int endIndex, List<?> list)
   {
      int numberOfElementsToRemove = subLengthInclusive(startIndex, endIndex, list);
   
      for (int count = 0; count < numberOfElementsToRemove; count++)
      {
         startIndex = wrap(startIndex, list);
         list.remove(startIndex);
      }

      return numberOfElementsToRemove;
   }

   /**
    * Removes all the elements from the list from startIndex (excluded) to endIndex (excluded).
    */
   public static int removeAllExclusive(int startIndex, int endIndex, List<?> list)
   {
      int numberOfElementsToRemove = subLengthExclusive(startIndex, endIndex, list);
   
      startIndex = next(startIndex, list);

      for (int count = 0; count < numberOfElementsToRemove; count++)
      {
         startIndex = wrap(startIndex, list);
         list.remove(startIndex);
      }

      return numberOfElementsToRemove;
   }
}
