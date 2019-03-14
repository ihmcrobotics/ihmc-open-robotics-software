package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.List;

public class SimplePlanarRegionFootstepNodeSnapper extends FootstepNodeSnapper
{
   private final Point2D footPosition = new Point2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final ConvexPolygon2D scaledRegionPolygon = new ConvexPolygon2D();

   public SimplePlanarRegionFootstepNodeSnapper()
   {
      this(new DefaultFootstepPlannerParameters());
   }

   public SimplePlanarRegionFootstepNodeSnapper(FootstepPlannerParameters parameters)
   {
      super(parameters);
   }

   @Override
   public FootstepNodeSnapData snapInternal(int xIndex, int yIndex)
   {
      FootstepNodeTools.getFootPosition(xIndex, yIndex, footPosition);
      PlanarRegion highestRegion = findHighestRegion(footPosition.getX(), footPosition.getY());

      // TODO either lower grid size or search cell if no region found
      if(highestRegion == null)
      {
         return FootstepNodeSnapData.emptyData();
      }
      else
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         if(parameters.getProjectInsideDistance() > 0.0)
         {
            projectPointIntoRegion(highestRegion, footPosition.getX(), footPosition.getY(), transform);

            // projection failed
            if(transform.containsNaN())
               return FootstepNodeSnapData.emptyData();
            else
               return new FootstepNodeSnapData(transform);
         }
         else
         {
            transform.setTranslationZ(highestRegion.getPlaneZGivenXY(footPosition.getX(), footPosition.getY()));
            return new FootstepNodeSnapData(transform);
         }
      }
   }

   private PlanarRegion findHighestRegion(double x, double y)
   {
      List<PlanarRegion> intersectingRegions = PlanarRegionTools
            .findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegionsList.getPlanarRegionsAsList(), footPosition);

      if (intersectingRegions == null || intersectingRegions.isEmpty())
         return null;

      double highestPoint = Double.NEGATIVE_INFINITY;
      PlanarRegion highestPlanarRegion = null;

      for (int i = 0; i < intersectingRegions.size(); i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);
         double height = planarRegion.getPlaneZGivenXY(x, y);

         if (height > highestPoint)
         {
            highestPoint = height;
            highestPlanarRegion = planarRegion;
         }
      }

      return highestPlanarRegion;
   }

   private void projectPointIntoRegion(PlanarRegion region, double x, double y, RigidBodyTransform transformToPack)
   {
      Point3D pointToSnap = new Point3D();
      pointToSnap.set(x, y, region.getPlaneZGivenXY(x, y));
      double projectionDistance = parameters.getProjectInsideDistance();
      boolean successfulScale = polygonScaler.scaleConvexPolygon(region.getConvexHull(), projectionDistance, scaledRegionPolygon);

      // region is too small to wiggle inside
      // TODO either search nearby or make the grid size match the wiggle-inside distance to avoid missing valid footholds inside the cell
      if(!successfulScale)
      {
         transformToPack.setToNaN();
         return;
      }

      region.transformFromWorldToLocal(pointToSnap);
      Point2D projectedPoint = new Point2D(pointToSnap.getX(), pointToSnap.getY());
      boolean successfulProjection = scaledRegionPolygon.orthogonalProjection(projectedPoint);
      if(!successfulProjection)
      {
         transformToPack.setToNaN();
         return;
      }

      Vector3D snapTranslation = new Vector3D();
      snapTranslation.set(projectedPoint.getX(), projectedPoint.getY(), 0.0);
      snapTranslation.sub(pointToSnap.getX(), pointToSnap.getY(), 0.0);
      region.transformFromLocalToWorld(snapTranslation);

      // set horizontal translation from projection
      transformToPack.setTranslationX(snapTranslation.getX());
      transformToPack.setTranslationY(snapTranslation.getY());

      // set translation height from region
      double projectedX = x + snapTranslation.getX();
      double projectedY = y + snapTranslation.getY();
      transformToPack.setTranslationZ(region.getPlaneZGivenXY(projectedX, projectedY));
   }
}