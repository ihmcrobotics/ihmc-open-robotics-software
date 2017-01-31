package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.robotics.lists.RecyclingArrayList;

public class ConvexPolygonConstructorFromInteriorOfRays
{
   private final ArrayList<Point2d> pointPool = new ArrayList<Point2d>();
   
   private final ArrayList<Point2d> intersectionArrayList = new ArrayList<Point2d>();
   private final ArrayList<Point2d> intersectionPoints = new ArrayList<Point2d>();
   private final RecyclingArrayList<MutableBoolean> removeRay = new RecyclingArrayList<MutableBoolean>(MutableBoolean.class);
   
   public boolean constructFromInteriorOfRays(List<Line2d> rays, ConvexPolygon2d polygonToPack)
   {      
      removeRay.clear();
      
      intersectionPoints.clear();
      for (int i=0; i<rays.size(); i++)
      {
         intersectionPoints.add(null);
         removeRay.getAndGrowIfNeeded(i).setValue(false);
      }

      for (int rayIndex = 0; rayIndex < rays.size(); rayIndex++)
      {
         if (removeRay.get(rayIndex).booleanValue())
         {
            continue;    // This ray was removed already. Don't consider it.
         }

         Line2d ray = rays.get(rayIndex);
         if (ray == null)
            continue;

         // Check previous non-null rays. For each one If its point is outside this ray, mark the ray as removed and delete its point.
         int previousIndexToCheck = rayIndex;

         while (true)
         {
            previousIndexToCheck--;
            if (previousIndexToCheck < 0)
               previousIndexToCheck = rays.size() - 1;

            if (previousIndexToCheck == rayIndex)
            {
               throw new RuntimeException("Should never get here!!!");
            }

            if (removeRay.get(previousIndexToCheck).booleanValue())
               continue;    // That ray was removed already. Check a previous ray.

            Point2d previousPointToCheck = intersectionPoints.get(previousIndexToCheck);

            // If the point is not to be in the polygon, as determined by the current ray under consideration, remove it and try the previous one.
            if ((previousPointToCheck != null) && (ray.isPointOnLeftSideOfLine(previousPointToCheck)))
            {
               removeRay.get(previousIndexToCheck).setValue(true);
               removeIntersectionPoint(intersectionPoints, previousIndexToCheck);
//               intersectionPoints[previousIndexToCheck] = null;

               continue;
            }

            // Found a previous Ray that may be ok. Use it to find this rays potential intersection point:
            Line2d previousRayToCheck = rays.get(previousIndexToCheck);

            Point2d intersection = getAndRemovePointFromPool();
            if (intersection == null) throw new RuntimeException("intersection == null!");
            
            boolean foundIntersection = ray.intersectionWith(previousRayToCheck, intersection);
            
            if (!foundIntersection)
            {
               returnPointsToPool(intersectionPoints);
               return false;    // If two lines don't intersect, then it must be null interior, when using this as intended...
            }
            intersectionPoints.set(rayIndex, intersection);

            // Now keep looking back to make sure that this point is valid:
            int evenMorePreviousIndexToCheck = previousIndexToCheck;
            while (true)
            {
               evenMorePreviousIndexToCheck--;
               if (evenMorePreviousIndexToCheck < 0)
                  evenMorePreviousIndexToCheck = rays.size() - 1;

               if (evenMorePreviousIndexToCheck == rayIndex)
               {
                  returnPointsToPool(intersectionPoints);
                  return false;    // If loop around, must be null interior.

                  // throw new RuntimeException("Should never get here!!!");
               }

               if (removeRay.get(evenMorePreviousIndexToCheck).booleanValue())
                  continue;    // That ray was removed already. Check a previous ray.
               Line2d evenMorePreviousRayToCheck = rays.get(evenMorePreviousIndexToCheck);

               // If the intersection point is not valid, then need to remove the previousRayToCheck and continue looking for more to remove...
               if (evenMorePreviousRayToCheck.isPointOnLeftSideOfLine(intersection))
               {
                  removeRay.get(previousIndexToCheck).setValue(true);
                  removeIntersectionPoint(intersectionPoints, previousIndexToCheck);
//                  intersectionPoints[previousIndexToCheck] = null;

                  previousIndexToCheck = evenMorePreviousIndexToCheck;
                  
                  intersection = getAndRemovePointFromPool();
                  foundIntersection = ray.intersectionWith(evenMorePreviousRayToCheck, intersection);
                  if (!foundIntersection)
                  {
                     returnPointsToPool(intersectionPoints);
                     return false;    // If two rays don't intersect, then it must be null interior, when using this as intended...
                  }

                  intersectionPoints.set(rayIndex, intersection);

               }
               else
               {
                  break;
               }
            }

            break;
         }
      }

      intersectionArrayList.clear();

      for (int i=0; i<intersectionPoints.size(); i++)
      {
         Point2d point = intersectionPoints.get(i);
         
         if (point != null)
            intersectionArrayList.add(point);
      }

      if (intersectionArrayList.size() < 3)
      {
         returnPointsToPool(intersectionPoints);
         return false;
      }

      polygonToPack.setAndUpdate(intersectionArrayList, intersectionArrayList.size());
      
      returnPointsToPool(intersectionPoints);
      return true;
   }

   private Point2d getAndRemovePointFromPool()
   {
      if (pointPool.isEmpty())
      {
         pointPool.add(new Point2d());
      }
      
      return pointPool.remove(pointPool.size()-1);
   }
   
   private void removeIntersectionPoint(ArrayList<Point2d> intersectionPoints, int index)
   {
      Point2d pointToRemove = intersectionPoints.get(index);
      
      if (pointToRemove != null)
      {
         pointPool.add(pointToRemove);
         intersectionPoints.set(index, null);
      }
      
   }
   
   private void returnPointsToPool(ArrayList<Point2d> intersectionPoints)
   {
      for (int i=0; i<intersectionPoints.size(); i++)
      {
         Point2d point = intersectionPoints.get(i);
         if (point != null) pointPool.add(point);
      }
      
      intersectionPoints.clear();
   }
}
