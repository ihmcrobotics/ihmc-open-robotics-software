package us.ihmc.robotics.geometry;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class ConvexPolygon2dIntersectionSetCalculator
{
   private BoundingBoxKDTree2D kdTree;
   private final ArrayList<ConvexPolygon2D> convexPolygon2dsToSearchForIntersection = new ArrayList<ConvexPolygon2D>();
   private final ArrayList<BoundingBox2D> boundingBoxes = new ArrayList<>();

   public ConvexPolygon2dIntersectionSetCalculator(ArrayList<ConvexPolygon2D> convexPolygon2d)
   {
      resetBaseConvexPolygon2ds(convexPolygon2d);
   }


   /**
    *    Finds the Tentative List of Polygons that intersect with the target polygon.
    *
    *    @param ConvexPolygon2D targetPolygon
    *    @param ArrayList<ConvexPolygon2d> convexPolygon2d The list of the polygon to search from.
    *    @return ArrayList<ConvexPolygon2d> The list of the polygons that might intersect with the target polygon.
    */
   public ArrayList<ConvexPolygon2D> findTentativeListOfPolygonsIntersectingTargetPolygon(ConvexPolygon2D targetPolygon)
   {
      ArrayList<ConvexPolygon2D> tentativeListOfPolygonsIntersectingTargetPolygon = new ArrayList<ConvexPolygon2D>();
      ArrayList<Object> intersectingObjects = kdTree.getIntersectingObjects(targetPolygon.getBoundingBox());
      for (Object o : intersectingObjects)
      {
         tentativeListOfPolygonsIntersectingTargetPolygon.add((ConvexPolygon2D) o);
      }

      return tentativeListOfPolygonsIntersectingTargetPolygon;
   }

   public ArrayList<ConvexPolygon2D> findIntersectionPolygonList(ConvexPolygon2D targetPolygon)
   {
      ArrayList<ConvexPolygon2D> tentativeList = findTentativeListOfPolygonsIntersectingTargetPolygon(targetPolygon);

      if (tentativeList == null || tentativeList.isEmpty())
         return null;

      ArrayList<ConvexPolygon2D> ret = new ArrayList<ConvexPolygon2D>();

      for (ConvexPolygon2D tentativePolygon : tentativeList)
      {
         ConvexPolygon2D intersection = new ConvexPolygon2D();
         boolean success = ConvexPolygonTools.computeIntersectionOfPolygons(targetPolygon, tentativePolygon, intersection);

         if (success)
            ret.add(intersection);
      }

      return ret;
   }


   public void resetBaseConvexPolygon2ds(ArrayList<ConvexPolygon2D> convexPolygon2ds)
   {
      this.convexPolygon2dsToSearchForIntersection.clear();
      this.boundingBoxes.clear();

      for (ConvexPolygon2D convexPolygon2d : convexPolygon2ds)
      {
         this.convexPolygon2dsToSearchForIntersection.add(convexPolygon2d);
      }

      for (int i = 0; i < convexPolygon2dsToSearchForIntersection.size(); i++)
      {
         ConvexPolygon2D convexPolygon2d = convexPolygon2dsToSearchForIntersection.get(i);
         boundingBoxes.add(convexPolygon2d.getBoundingBox());
      }

      ArrayList<Object> allObjects = new ArrayList<Object>(convexPolygon2dsToSearchForIntersection);
      this.kdTree = new BoundingBoxKDTree2D(boundingBoxes, allObjects);

   }

}
