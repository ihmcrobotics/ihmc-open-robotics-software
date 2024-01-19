package us.ihmc.robotics.lists;

import us.ihmc.commons.lists.RecyclingArrayList;

/**
 * These tools allow to do a few extra things with RecyclingArrayList.
 */
public class RecyclingArrayListTools
{
   /**
    * Remove the last item. Could be added to RecyclingArrayList sometime.
    */
   public static <T> T removeLast(RecyclingArrayList<T> recyclingArrayList)
   {
      return recyclingArrayList.remove(recyclingArrayList.size() - 1);
   }

   /**
    * Adds an item to all the lists passed in.
    */
   public static void addToAll(RecyclingArrayList... recyclingArrayLists)
   {
      for (RecyclingArrayList recyclingArrayList : recyclingArrayLists)
      {
         recyclingArrayList.add();
      }
   }

   /**
    * This makes sure the pre-allocated internal size of listToModify is the same as source.
    * This is important for {@link #getUnsafe} to work.
    */
   public static void synchronizeSize(RecyclingArrayList listToModify, RecyclingArrayList source)
   {
      synchronizeSize(listToModify, source.size());
   }

   /**
    * This makes sure the pre-allocated internal size of listToModify is the same as source.
    * This is important for {@link #getUnsafe} to work.
    */
   public static void synchronizeSize(RecyclingArrayList listToModify, int sourceSize)
   {
      while (listToModify.size() < sourceSize)
         listToModify.add();

      while (listToModify.size() > sourceSize)
         removeLast(listToModify);
   }

   /**
    * Allows to access an item in the pre-allocated underlying list whether it's currently
    * within the current active size or not.
    */
   public static <T> T getUnsafe(RecyclingArrayList<T> recyclingArrayList, int elementToAccess)
   {
      int originalSize = recyclingArrayList.size();
      T potentiallyOutOfBoundsElement = recyclingArrayList.getAndGrowIfNeeded(elementToAccess);

      while (recyclingArrayList.size() > originalSize)
         removeLast(recyclingArrayList);

      return potentiallyOutOfBoundsElement;
   }
}
