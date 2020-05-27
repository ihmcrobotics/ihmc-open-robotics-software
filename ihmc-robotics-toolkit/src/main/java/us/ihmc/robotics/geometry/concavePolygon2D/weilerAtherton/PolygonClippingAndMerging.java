package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import sun.awt.image.ImageWatched.Link;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class PolygonClippingAndMerging
{
   private static final int STUPID_LARGE = 1000;
   public static void mergeAllPossible(List<ConcavePolygon2DBasics> regionsToMerge)
   {
      int i = 0;
      // don't need to iterate on the last one
      while (i < regionsToMerge.size() - 1)
      {
         int j = i + 1;
         boolean shouldRemoveA = false;
         while (j < regionsToMerge.size())
         {
            ConcavePolygon2DBasics polygonA = regionsToMerge.get(i);
            ConcavePolygon2DBasics polygonB = regionsToMerge.get(j);
            if (GeometryPolygonTools.doPolygonsIntersect(polygonA, polygonB))
            {
               ConcavePolygon2D newPolygon = new ConcavePolygon2D();
               PolygonClippingAndMerging.merge(polygonA, polygonB, newPolygon);

               regionsToMerge.set(i, newPolygon);
               regionsToMerge.remove(j);

               // reset the search, as we modified the first polygon
               j = i + 1;
            }
            else if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonB, polygonA))
            {
               regionsToMerge.remove(j);
            }
            else if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonA, polygonB))
            {
               shouldRemoveA = true;
               break;
            }
            else
            {
               j++;
            }
         }

         if (shouldRemoveA)
            regionsToMerge.remove(i);
         else
            i++;
      }
   }
   public static void merge(ConcavePolygon2DReadOnly polygonA, ConcavePolygon2DReadOnly polygonB, ConcavePolygon2DBasics mergedPolygon)
   {
      List<ConcavePolygon2DBasics> holesToReturn = new ArrayList<>();

      if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonA, polygonB))
      {
         mergedPolygon.set(polygonB);
         return;
      }
      else if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonB, polygonA))
      {
         mergedPolygon.set(polygonA);
         return;
      }

      LinkedPointList polygonAList = ClippingTools.createLinkedPointList(polygonA);
      LinkedPointList polygonBList = ClippingTools.createLinkedPointList(polygonB);

      ClippingTools.insertIntersectionsIntoList(polygonBList, polygonA);
      ClippingTools.insertIntersectionsIntoList(polygonAList, polygonB);

      Collection<LinkedPoint> unassignedAPoints = polygonAList.getPointsCopy();
      Collection<LinkedPoint> unassignedBPoints = polygonBList.getPointsCopy();
      LinkedPointProvider pointProvider = getPointProvider(polygonA, polygonAList, unassignedAPoints, true, polygonB, polygonBList, unassignedBPoints, true);

      // FIXME the point provider should iterate over points on the outside.
      while (pointProvider != null)
      {
         ConcavePolygon2D polygon = new ConcavePolygon2D();
         walkAlongEdgeOfPolygon(pointProvider, polygon);
         holesToReturn.add(polygon);

         pointProvider = getPointProvider(polygonA, polygonAList, unassignedAPoints, true, polygonB, polygonBList, unassignedBPoints, true);
      }

      ConcavePolygon2DBasics polygonWithLargestArea = holesToReturn.get(0);
      double largestArea = polygonWithLargestArea.getArea();
      for (int i = 1; i < holesToReturn.size(); i++)
      {
         ConcavePolygon2DBasics polygon = holesToReturn.get(i);
         if (polygon.getArea() > largestArea)
         {
            polygonWithLargestArea = polygon;
            largestArea = polygon.getArea();
         }
      }

      holesToReturn.remove(polygonWithLargestArea);
      mergedPolygon.set(polygonWithLargestArea);
   }

   public static LinkedPointProvider getPointProvider(ConcavePolygon2DReadOnly polygonA,
                                                      LinkedPointList polygonAList,
                                                      Collection<LinkedPoint> unassignedListA,
                                                      boolean checkOnListA,
                                                      ConcavePolygon2DReadOnly polygonB,
                                                      LinkedPointList polygonBList,
                                                      Collection<LinkedPoint> unassignedListB,
                                                      boolean checkOnListB)
   {
      LinkedPoint linkedPoint = null;
      boolean useListA = false;
      if (checkOnListB)
      {
         linkedPoint = findVertexOutsideOfPolygon(polygonA, unassignedListB);
         useListA = false;
      }
      if (linkedPoint == null && checkOnListA)
      {
         linkedPoint = findVertexOutsideOfPolygon(polygonB, unassignedListA);
         useListA = true;
      }

//      if (linkedPoint == null)
//      {
//         linkedPoint = findIntersectionPoint(unassignedListB);
//         useListA = false;
//      }

      if (linkedPoint == null)
         return null;

      return new LinkedPointProvider(linkedPoint, polygonAList, polygonBList, unassignedListA, unassignedListB, useListA);
   }

   public static List<ConcavePolygon2DBasics> removeAreaInsideClip(ConcavePolygon2DReadOnly clippingPolygon, ConcavePolygon2DReadOnly polygonToClip)
   {
      List<ConcavePolygon2DBasics> clippedPolygonsToReturn = new ArrayList<>();
      if (GeometryPolygonTools.isPolygonInsideOtherPolygon(clippingPolygon, polygonToClip))
      {
         ConcavePolygon2D polygonToReturn = new ConcavePolygon2D(clippingPolygon);
         clippedPolygonsToReturn.add(polygonToReturn);

         return clippedPolygonsToReturn;
      }
      else if (!GeometryPolygonTools.doPolygonsIntersect(clippingPolygon, polygonToClip))
      {
         ConcavePolygon2D polygonToReturn = new ConcavePolygon2D(polygonToClip);
         clippedPolygonsToReturn.add(polygonToReturn);

         return clippedPolygonsToReturn;
      }

      LinkedPointList clippingPolygonList = ClippingTools.createLinkedPointList(clippingPolygon);
      LinkedPointList polygonToClipList = ClippingTools.createLinkedPointList(polygonToClip);

      ClippingTools.insertIntersectionsIntoList(polygonToClipList, clippingPolygon);
      ClippingTools.insertIntersectionsIntoList(clippingPolygonList, polygonToClip);

      // gotta make this guy counter clockwise
      clippingPolygonList.reverseOrder();

      Collection<LinkedPoint> unassignedToClipPoints = polygonToClipList.getPointsCopy();
      Collection<LinkedPoint> unassignedClippingPoints = clippingPolygonList.getPointsCopy();
      LinkedPointProvider pointProvider = getPointProvider(clippingPolygon, clippingPolygonList, unassignedClippingPoints, false, polygonToClip, polygonToClipList, unassignedToClipPoints, true);

      int counter = 0;
      while (pointProvider != null && counter++ < STUPID_LARGE)
      {
         ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
         walkAlongEdgeOfPolygon(pointProvider, clippedPolygon);
         clippedPolygonsToReturn.add(clippedPolygon);

         pointProvider = getPointProvider(clippingPolygon, clippingPolygonList, unassignedClippingPoints, false, polygonToClip, polygonToClipList, unassignedToClipPoints, true);
      }

      return clippedPolygonsToReturn;
   }

   static void walkAlongEdgeOfPolygon(LinkedPointProvider pointProvider, ConcavePolygon2DBasics polygonToPack)
   {
      LinkedPoint linkedPoint = pointProvider.getCurrentPoint();
      LinkedPoint previousPoint = linkedPoint;

      polygonToPack.addVertex(linkedPoint.getPoint());
      int counter = 0;
      while (counter++ < STUPID_LARGE)
      {
         linkedPoint = pointProvider.incrementPoint();
         pointProvider.removePoint(previousPoint);

         if (linkedPoint.getPoint().epsilonEquals(pointProvider.getStartVertex().getPoint(), 1e-7))
            break;

         polygonToPack.addVertex(linkedPoint.getPoint());

         if (linkedPoint.getIsIntersectionPoint())
         {
            // we're switching polygons
            previousPoint = linkedPoint;
            linkedPoint = pointProvider.switchList();

            if (linkedPoint == null)
               throw new RuntimeException("Was unable to find the intersection point in the other list.");
         }
         else
         {
            previousPoint = linkedPoint;
         }
      }

      polygonToPack.update();
   }

   private static LinkedPoint findVertexOutsideOfPolygon(ConcavePolygon2DReadOnly polygon, Collection<LinkedPoint> points)
   {
      for (LinkedPoint point : points)
      {
         if (!polygon.isPointInside(point.getPoint()))
            return point;
      }

      return null;
   }

   private static LinkedPoint findIntersectionInsideOfPolygon(ConcavePolygon2DReadOnly polygon, Collection<LinkedPoint> points)
   {
      for (LinkedPoint point : points)
      {
         if (polygon.isPointInside(point.getPoint()))
            return point;
      }

      return null;
   }

   private static LinkedPoint findIntersectionPoint( Collection<LinkedPoint> polygon)
   {
      for (LinkedPoint point : polygon)
      {
         if (point.getIsIntersectionPoint())
            return point;
      }

      return null;
   }

   private static void removePointsFromList(Collection<LinkedPoint> listToEdit, Collection<LinkedPoint> pointsToRemove)
   {
      for (LinkedPoint pointToRemove : pointsToRemove)
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
}
