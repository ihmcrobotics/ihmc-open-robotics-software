package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.List;

public class PlanarRegionSnapTools
{
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final ConvexPolygon2D scaledRegionPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();

   private final FootstepPlannerParameters parameters;

   public PlanarRegionSnapTools(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   public PlanarRegion findHighestRegion(double x, double y, Vector2D projectionTranslationToPack, List<PlanarRegion> planarRegions)
   {
      tempPolygon.clearAndUpdate();
      tempPolygon.addVertex(0.5 * FootstepNode.gridSizeXY, 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.addVertex(0.5 * FootstepNode.gridSizeXY, - 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.addVertex(- 0.5 * FootstepNode.gridSizeXY, 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.addVertex(- 0.5 * FootstepNode.gridSizeXY, - 0.5 * FootstepNode.gridSizeXY);
      tempPolygon.update();
      tempPolygon.translate(x, y);

      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(tempPolygon, planarRegions);
      if (intersectingRegions == null || intersectingRegions.isEmpty())
      {
         return null;
      }

      double highestPoint = Double.NEGATIVE_INFINITY;
      PlanarRegion highestPlanarRegion = null;

      for (int i = 0; i < intersectingRegions.size(); i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);
         Vector3D projectionTranslation = projectPointIntoRegion(planarRegion, x, y);
         double height;

         if(projectionTranslation.containsNaN())
         {
            // even if projection fails, remember highest region. this will be considered an obstacle
            height = planarRegion.getPlaneZGivenXY(x, y);
         }
         else
         {
            height = planarRegion.getPlaneZGivenXY(x + projectionTranslation.getX(), y + projectionTranslation.getY());
         }

         if (height > highestPoint)
         {
            highestPoint = height;
            highestPlanarRegion = planarRegion;
            projectionTranslationToPack.set(projectionTranslation);
         }
      }

      return highestPlanarRegion;
   }

   private Vector3D projectPointIntoRegion(PlanarRegion region, double x, double y)
   {
      Vector3D projectionTranslation = new Vector3D();
      Point3D pointToSnap = new Point3D();

      pointToSnap.set(x, y, region.getPlaneZGivenXY(x, y));
      double projectionDistance = parameters.getProjectInsideDistance();
      boolean successfulScale = polygonScaler.scaleConvexPolygon(region.getConvexHull(), projectionDistance, scaledRegionPolygon);

      // region is too small to wiggle inside
      if(!successfulScale)
      {
         projectionTranslation.setToNaN();
         return projectionTranslation;
      }

      region.transformFromWorldToLocal(pointToSnap);
      Point2D projectedPoint = new Point2D(pointToSnap.getX(), pointToSnap.getY());

      double signedDistanceToPolygon = scaledRegionPolygon.signedDistance(projectedPoint);
      if(signedDistanceToPolygon <= 0.0)
      {
         // return, no need to project
         projectionTranslation.setToZero();
         return projectionTranslation;
      }

      boolean successfulProjection = scaledRegionPolygon.orthogonalProjection(projectedPoint);
      if(!successfulProjection)
      {
         projectionTranslation.setToNaN();
         return projectionTranslation;
      }

      projectionTranslation.set(projectedPoint.getX(), projectedPoint.getY(), 0.0);
      projectionTranslation.sub(pointToSnap.getX(), pointToSnap.getY(), 0.0);
      region.transformFromLocalToWorld(projectionTranslation);
      projectionTranslation.setZ(0.0);

      return projectionTranslation;
   }
}
