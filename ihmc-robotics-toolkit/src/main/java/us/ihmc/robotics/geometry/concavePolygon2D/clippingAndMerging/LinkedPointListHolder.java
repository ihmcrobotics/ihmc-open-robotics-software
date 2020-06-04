package us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging;

import java.util.Collection;

class LinkedPointListHolder
{
   private final Collection<LinkedPoint> listAPool;
   private final Collection<LinkedPoint> listBPool;

   public LinkedPointListHolder(Collection<LinkedPoint> listAPool, Collection<LinkedPoint> listBPool)
   {
      this.listAPool = listAPool;
      this.listBPool = listBPool;
   }

   public void removePoint(LinkedPoint pointToRemove)
   {
      removePointFromList(listAPool, pointToRemove);
      removePointFromList(listBPool, pointToRemove);
   }

   private static void removePointFromList(Collection<LinkedPoint> listToEdit, LinkedPoint pointToRemove)
   {
      for (LinkedPoint other : listToEdit)
      {
         if (other.getPoint().epsilonEquals(pointToRemove.getPoint(), 1e-7))
         {
            listToEdit.remove(other);
            break;
         }
      }
   }

   public int getNumberOfPoints()
   {
      return listAPool.size() + listBPool.size();
   }
}
