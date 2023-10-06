package us.ihmc.robotics.lists;

import us.ihmc.commons.lists.RecyclingArrayList;

public class RecyclingArrayListTools
{
   public static <T> T removeLast(RecyclingArrayList<T> recyclingArrayList)
   {
      return recyclingArrayList.remove(recyclingArrayList.size() - 1);
   }

   public static void addToAll(RecyclingArrayList... recyclingArrayLists)
   {
      for (RecyclingArrayList recyclingArrayList : recyclingArrayLists)
      {
         recyclingArrayList.add();
      }
   }

   public static void synchronizeSize(RecyclingArrayList listToModify, RecyclingArrayList source)
   {
      while (listToModify.size() < source.size())
         listToModify.add();

      while (listToModify.size() > source.size())
         removeLast(listToModify);
   }

   public static <T> T getUnsafe(RecyclingArrayList<T> recyclingArrayList, int elementToAccess)
   {
      int originalSize = recyclingArrayList.size();
      T potentiallyOutOfBoundsElement = recyclingArrayList.getAndGrowIfNeeded(elementToAccess);

      while (recyclingArrayList.size() > originalSize)
         removeLast(recyclingArrayList);

      return potentiallyOutOfBoundsElement;
   }
}
