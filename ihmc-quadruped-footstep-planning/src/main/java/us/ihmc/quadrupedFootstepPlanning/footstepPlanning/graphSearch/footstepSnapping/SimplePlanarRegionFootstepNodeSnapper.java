package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;

public class SimplePlanarRegionFootstepNodeSnapper extends FootstepNodeSnapper
{

   private final PlanarRegion planarRegionToPack = new PlanarRegion();
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
      RigidBodyTransform snapTransform = PlanarRegionsListPointSnapper
            .snapPointToPlanarRegionsList(footPosition, planarRegionsList.getPlanarRegionsAsList(), planarRegionToPack);

      if (snapTransform == null)
         return FootstepNodeSnapData.emptyData();

      return new FootstepNodeSnapData(snapTransform);
   }
}