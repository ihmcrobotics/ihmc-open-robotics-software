package us.ihmc.avatar.footstepPlanning;

import us.ihmc.concurrent.ConcurrentCopier;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

class ConcurrentSet<T> extends ConcurrentCopier<Set<T>>
{
   public ConcurrentSet()
   {
      super(HashSet::new);
   }

   public boolean isEmpty()
   {
      Set<T> readCopy = this.getCopyForReading();
      if (readCopy == null)
         return true;
      else
         return readCopy.isEmpty();
   }

   public int size()
   {
      Set<T> currentList = getCopyForReading();
      if (currentList == null)
         return 0;
      else
         return currentList.size();
   }

   public void clear()
   {
      Set<T> updatedList = getCopyForWriting();
      updatedList.clear();
      commit();
   }

   public void add(T objectToAdd)
   {
      Set<T> existingList = getCopyForReading();
      Set<T> updatedList = getCopyForWriting();
      updatedList.clear();
      if (existingList != null)
         updatedList.addAll(existingList);
      updatedList.add(objectToAdd);

      this.commit();
   }

   public boolean remove(T objectToRemove)
   {
      Set<T> existingList = getCopyForReading();
      Set<T> updatedList = getCopyForWriting();
      updatedList.clear();
      if (existingList != null)
         updatedList.addAll(existingList);
      boolean success = updatedList.remove(objectToRemove);

      this.commit();

      return success;
   }

   public boolean contains(T object)
   {
      return getCopyForReading().contains(object);
   }

   public Iterable<T> iterable()
   {
      return getCopyForReading();
   }
}
