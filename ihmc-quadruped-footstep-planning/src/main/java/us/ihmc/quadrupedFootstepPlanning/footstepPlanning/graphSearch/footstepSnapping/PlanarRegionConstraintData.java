package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.HashMap;
import java.util.List;

public class PlanarRegionConstraintData
{
   private final PlanarRegion planarRegion;
   private final List<ConvexPolygon2D> convexPolygons;
   private final HashMap<ConvexPolygon2DReadOnly, ConvexPolygon2DReadOnly> scaledPolygons = new HashMap<>();
   private final HashMap<ConvexPolygon2DReadOnly, TIntArrayList> polygonIndicesToIgnore = new HashMap<>();

   private final ConvexPolygonScaler polygonScaler;

   private final ConvexPolygon2DReadOnly scaledConvexHull;

   private final boolean projectInsideUsingConvexHull;
   private final double projectionInsideDelta;

   public PlanarRegionConstraintData(ConvexPolygonScaler polygonScaler, PlanarRegion planarRegion, boolean projectInsideUsingConvexHull, double projectionInsideDelta)
   {
      this.polygonScaler = polygonScaler;
      this.planarRegion = planarRegion;
      this.convexPolygons = planarRegion.getConvexPolygons();
      this.projectInsideUsingConvexHull = projectInsideUsingConvexHull;
      this.projectionInsideDelta = projectionInsideDelta;

      if (projectInsideUsingConvexHull)
         scaledConvexHull = computedScaledConvexHull(polygonScaler, projectionInsideDelta);
      else
         scaledConvexHull = null;
   }

   private ConvexPolygon2DReadOnly computeScaledInternalPolygon(ConvexPolygon2DReadOnly internalRegionToScale, ConvexPolygonScaler polygonScaler, double projectionInsideDelta)
   {
      ConvexPolygon2D scaledRegionPolygon = new ConvexPolygon2D();
      TIntArrayList indicesToIgnore = getPolygonIndicesToIgnore(internalRegionToScale);
      if (polygonScaler.scaleConvexPolygon(internalRegionToScale, projectionInsideDelta, scaledRegionPolygon, indicesToIgnore.toArray()))
         return scaledRegionPolygon;
      else
         return null;
   }

   private ConvexPolygon2DReadOnly computedScaledConvexHull(ConvexPolygonScaler polygonScaler, double projectionInsideDelta)
   {
      ConvexPolygon2D scaledRegionPolygon = new ConvexPolygon2D();
      if (polygonScaler.scaleConvexPolygon(planarRegion.getConvexHull(), projectionInsideDelta, scaledRegionPolygon))
         return scaledRegionPolygon;
      else
         return null;
   }

   private static TIntArrayList computeIndicesToIgnoreForRegion(ConvexPolygon2DReadOnly containingRegion, List<ConvexPolygon2D> allRegions)
   {
      TIntArrayList indicesToIgnore = new TIntArrayList();
      for (int index = 0; index < containingRegion.getNumberOfVertices(); index++)
      {
         Point2DReadOnly vertex = containingRegion.getVertex(index);
         Point2DReadOnly nextVertex = containingRegion.getNextVertex(index);
         if (PlanarRegionSnapTools.isPointInOtherRegion(vertex, containingRegion, allRegions) && PlanarRegionSnapTools
               .isPointInOtherRegion(nextVertex, containingRegion, allRegions))
            indicesToIgnore.add(index);
      }
      return indicesToIgnore;
   }

   public TIntArrayList getPolygonIndicesToIgnore(ConvexPolygon2DReadOnly polygonRegion)
   {
      if (polygonIndicesToIgnore.containsKey(polygonRegion))
      {
         return polygonIndicesToIgnore.get(polygonRegion);
      }
      else
      {
         TIntArrayList indicesToIgnore = computeIndicesToIgnoreForRegion(polygonRegion, convexPolygons);
         polygonIndicesToIgnore.put(polygonRegion, indicesToIgnore);
         return indicesToIgnore;
      }
   }

   public ConvexPolygon2DReadOnly getScaledRegionPolygon(Point2DReadOnly pointToCheck)
   {
      if (projectInsideUsingConvexHull)
      {
         return scaledConvexHull;
      }
      else
      {
         ConvexPolygon2DReadOnly containingRegion = PlanarRegionSnapTools.getContainingConvexRegion(pointToCheck, convexPolygons);
         if (containingRegion == null)
            return null;

         if (scaledPolygons.containsKey(containingRegion))
            return scaledPolygons.get(containingRegion);
         else
         {
            ConvexPolygon2DReadOnly scaledPolygon = computeScaledInternalPolygon(containingRegion, polygonScaler, projectionInsideDelta);
            scaledPolygons.put(containingRegion, scaledPolygon);
            return scaledPolygon;
         }
      }
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
