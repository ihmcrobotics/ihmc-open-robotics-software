package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.List;

public class SimplePlanarRegionFootstepNodeSnapper extends FootstepNodeSnapper
{
   private final Point2D footPosition = new Point2D();

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

      List<PlanarRegion> intersectingRegions = PlanarRegionTools
            .findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegionsList.getPlanarRegionsAsList(), footPosition);

      if(intersectingRegions == null || intersectingRegions.isEmpty())
         return FootstepNodeSnapData.emptyData();

      double highestPoint = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < intersectingRegions.size(); i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);
         double height = planarRegion.getPlaneZGivenXY(footPosition.getX(), footPosition.getY());

         if(height > highestPoint)
            highestPoint = height;
      }

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationZ(highestPoint);
      return new FootstepNodeSnapData(transform);
   }
}