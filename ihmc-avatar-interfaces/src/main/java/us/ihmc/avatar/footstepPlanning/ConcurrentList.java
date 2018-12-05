package us.ihmc.avatar.footstepPlanning;

import us.ihmc.concurrent.ConcurrentCopier;

import java.util.ArrayList;
import java.util.List;

class ConcurrentList<T> extends ConcurrentCopier<List<T>>
{
   public ConcurrentList()
   {
      super(ArrayList::new);
   }

   public boolean isEmpty()
   {
      List<T> readCopy = this.getCopyForReading();
      if (readCopy == null)
         return true;
      else
         return readCopy.isEmpty();
   }

   public int size()
   {
      List<T> currentList = getCopyForReading();
      if (currentList == null)
         return 0;
      else
         return currentList.size();
   }

   public void clear()
   {
      List<T> updatedList = getCopyForWriting();
      updatedList.clear();
      commit();
   }

   public void add(T objectToAdd)
   {
      List<T> existingList = getCopyForReading();
      List<T> updatedList = getCopyForWriting();
      updatedList.clear();
      if (existingList != null)
         updatedList.addAll(existingList);
      updatedList.add(objectToAdd);

      this.commit();
   }

   public T remove(int indexToRemove)
   {
      List<T> existingList = getCopyForReading();
      List<T> updatedList = getCopyForWriting();
      updatedList.clear();
      if (existingList != null)
         updatedList.addAll(existingList);
      T objectToReturn = updatedList.remove(indexToRemove);

      this.commit();

      return objectToReturn;
   }

   public Iterable<T> iterable()
   {
      return getCopyForReading();
   }
}
