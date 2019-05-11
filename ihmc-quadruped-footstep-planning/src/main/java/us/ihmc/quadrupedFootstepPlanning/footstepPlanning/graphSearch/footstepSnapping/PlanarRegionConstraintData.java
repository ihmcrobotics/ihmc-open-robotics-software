package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.HashMap;
import java.util.List;

public class PlanarRegionConstraintData
{
   private final PlanarRegion planarRegion;
   private final List<ConvexPolygon2D> convexPolygons;
   private final HashMap<ConvexPolygon2DReadOnly, TIntArrayList> sharedIndicesByRegion = new HashMap<>();

   public PlanarRegionConstraintData(PlanarRegion planarRegion)
   {
      this.planarRegion = planarRegion;
      this.convexPolygons = planarRegion.getConvexPolygons();

      computeIndicesToIgnoreForEachRegion();
   }

   private void computeIndicesToIgnoreForEachRegion()
   {
      for (ConvexPolygon2D region : convexPolygons)
      {
         sharedIndicesByRegion.put(region, computeIndicesToIgnoreForRegion(region, convexPolygons));
      }
   }

   private static TIntArrayList computeIndicesToIgnoreForRegion(ConvexPolygon2D containingRegion, List<ConvexPolygon2D> allRegions)
   {
      TIntArrayList indicesToIgnore = new TIntArrayList();
      for (int index = 0; index < containingRegion.getNumberOfVertices(); index++)
      {
         Point2DReadOnly vertex = containingRegion.getVertex(index);
         Point2DReadOnly nextVertex = containingRegion.getNextVertex(index);
         if (isPointInOtherRegion(vertex, containingRegion, allRegions) && isPointInOtherRegion(nextVertex, containingRegion, allRegions))
            indicesToIgnore.add(index);
      }
      return indicesToIgnore;
   }

   private static boolean isPointInOtherRegion(Point2DReadOnly point, ConvexPolygon2DReadOnly regionToIgnore, List<ConvexPolygon2D> allRegions)
   {
      for (ConvexPolygon2D convexPolygon : allRegions)
      {
         if (regionToIgnore.equals(convexPolygon))
            continue;

         if (convexPolygon.isPointInside(point))
            return true;

         for (Point2DReadOnly vertex : convexPolygon.getVertexBufferView())
         {
            if (vertex.epsilonEquals(point, 1e-8))
               return true;
         }
      }

      return false;
   }

   public ConvexPolygon2DReadOnly getContainingConvexRegion(Point2DReadOnly pointToCheck)
   {
      return getContainingConvexRegion(pointToCheck, convexPolygons);
   }

   public TIntArrayList getIndicesToIgnore(ConvexPolygon2DReadOnly region)
   {
      return sharedIndicesByRegion.get(region);
   }

   public static ConvexPolygon2DReadOnly getContainingConvexRegion(Point2DReadOnly pointToCheck, List<ConvexPolygon2D> convexPolygons)
   {
      int size = convexPolygons.size();
      for (int i = 0; i < size; i++)
      {
         ConvexPolygon2DReadOnly convexPolygon = convexPolygons.get(i);
         if (convexPolygon.isPointInside(pointToCheck))
            return convexPolygon;
      }

      return null;
   }


   @Override
   public int hashCode()
   {
      return planarRegion.hashCode();
   }

   @Override
   public boolean equals(Object o)
   {
      return planarRegion.equals(o);
   }
}
