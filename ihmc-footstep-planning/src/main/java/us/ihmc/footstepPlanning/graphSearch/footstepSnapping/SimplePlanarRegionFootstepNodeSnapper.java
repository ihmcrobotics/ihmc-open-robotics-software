package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

public class SimplePlanarRegionFootstepNodeSnapper extends FootstepNodeSnapper
{
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;

   private final PlanarRegion planarRegionToPack = new PlanarRegion();
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   public SimplePlanarRegionFootstepNodeSnapper(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      this(footPolygonsInSoleFrame, new DefaultFootstepPlanningParameters());
   }

   public SimplePlanarRegionFootstepNodeSnapper(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParameters parameters)
   {
      super(parameters);

      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
   }

   @Override
   public FootstepNodeSnapData snapInternal(FootstepNode footstepNode)
   {
      FootstepNodeTools.getFootPolygon(footstepNode, footPolygonsInSoleFrame.get(footstepNode.getRobotSide()), footPolygon);

      List<PlanarRegion> planarRegionsList = getOrCreateSteppableRegions(footstepNode.getRoundedX(), footstepNode.getRoundedY());
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, planarRegionToPack);

      if (snapTransform == null)
         return FootstepNodeSnapData.emptyData();

      ConvexPolygon2D footholdPolygon = FootstepNodeSnappingTools.getConvexHullOfPolygonIntersections(planarRegionToPack, footPolygon, snapTransform);

      if (footholdPolygon.isEmpty())
         return FootstepNodeSnapData.emptyData();

      FootstepNodeSnappingTools.changeFromPlanarRegionToSoleFrame(planarRegionToPack, footstepNode, snapTransform, footholdPolygon);
      return new FootstepNodeSnapData(snapTransform, footholdPolygon);
   }
}