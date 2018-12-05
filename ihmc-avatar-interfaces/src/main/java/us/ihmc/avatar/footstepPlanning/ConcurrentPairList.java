package us.ihmc.avatar.footstepPlanning;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.tools.lists.PairList;

class ConcurrentPairList<L, R> extends ConcurrentCopier<PairList<L, R>>
{
   public ConcurrentPairList()
   {
      super(PairList::new);
   }

   public boolean isEmpty()
   {
      PairList<L, R> readCopy = getCopyForReading();
      if (readCopy == null)
         return true;
      else
         return readCopy.isEmpty();
   }

   public void clear()
   {
      PairList<L, R> updatedList = getCopyForWriting();
      updatedList.clear();

      commit();
   }

   public void add(L leftObjectToAdd, R rightObjectToAdd)
   {
      PairList<L, R> existingList = getCopyForReading();
      PairList<L, R> updatedList = getCopyForWriting();
      updatedList.clear();
      if (existingList != null)
         updatedList.addAll(existingList);
      updatedList.add(new ImmutablePair<>(leftObjectToAdd, rightObjectToAdd));

      this.commit();
   }

   public ImmutablePair<L, R> remove(int indexToRemove)
   {
      PairList<L, R> existingList = getCopyForReading();
      PairList<L, R> updatedList = getCopyForWriting();
      updatedList.clear();
      if (existingList != null)
         updatedList.addAll(existingList);
      ImmutablePair<L, R> objectToReturn = updatedList.remove(indexToRemove);

      this.commit();

      return objectToReturn;
   }

   public Iterable<ImmutablePair<L, R>> iterable()
   {
      return getCopyForReading();
   }
}
