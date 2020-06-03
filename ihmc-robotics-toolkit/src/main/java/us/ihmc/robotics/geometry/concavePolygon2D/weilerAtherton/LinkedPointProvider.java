package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import java.util.Collection;

class LinkedPointProvider
{
   private final LinkedPointList listA;
   private final LinkedPointList listB;

   private final Collection<LinkedPoint> listAPool;
   private final Collection<LinkedPoint> listBPool;

   private LinkedPoint startVertex;
   private LinkedPointList activeList;
   private LinkedPoint currentPoint;
   private boolean usingListA;

   public LinkedPointProvider(LinkedPointList listA,
                              LinkedPointList listB,
                              Collection<LinkedPoint> listAPool,
                              Collection<LinkedPoint> listBPool)
   {
      this.listA = listA;
      this.listB = listB;
      this.listAPool = listAPool;
      this.listBPool = listBPool;
   }

   public void setStart(LinkedPoint startVertex, boolean usingListA)
   {
      this.startVertex = startVertex;
      this.usingListA = usingListA;
      this.currentPoint = startVertex;
      if (usingListA)
         activeList = listA;
      else
         activeList = listB;
   }

   public LinkedPoint switchList()
   {
      if (usingListA)
         activeList = listB;
      else
         activeList = listA;
      usingListA = !usingListA;
      currentPoint = activeList.getLinkedPointAtLocation(currentPoint.getPoint());
      return currentPoint;
   }

   public LinkedPoint getStartVertex()
   {
      return startVertex;
   }

   public LinkedPoint getCurrentPoint()
   {
      return currentPoint;
   }

   public LinkedPoint incrementPoint()
   {
      currentPoint = currentPoint.getSuccessor();
      return currentPoint;
   }

   public void removePoint(LinkedPoint pointToRemove)
   {
      if (pointToRemove.isLinkedToOtherList())
      {
         removePointFromList(listAPool, pointToRemove);
         removePointFromList(listBPool, pointToRemove);
      }
      else
      {
         if (usingListA)
            removePointFromList(listAPool, pointToRemove);
         else
            removePointFromList(listBPool, pointToRemove);
      }
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
}
