package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SnapBasedNodeChecker implements FootstepNodeChecker
{
   private final FootstepPlannerParameters parameters;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepNodeSnapper snapper;

   public SnapBasedNodeChecker(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.footPolygons = footPolygons;
      this.snapper = snapper;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      snapper.setPlanarRegions(planarRegions);
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(node);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();
      if (snapTransform.containsNaN())
         return false;

      ConvexPolygon2D footholdAfterSnap = snapData.getCroppedFoothold();
      double area = footholdAfterSnap.getArea();
      double footArea = footPolygons.get(node.getRobotSide()).getArea();
      if (area < parameters.getMinimumFootholdPercent() * footArea)
         return false;

      if (previousNode == null)
         return true;

      FootstepNodeSnapData previousNodeSnapData = snapper.snapFootstepNode(previousNode);
      RigidBodyTransform previousSnapTransform = previousNodeSnapData.getSnapTransform();
      double heightChange = Math.abs(snapTransform.getTranslationZ() - previousSnapTransform.getTranslationZ());
      if (heightChange > parameters.getMaximumStepZ())
         return false;

      return true;
   }
}
