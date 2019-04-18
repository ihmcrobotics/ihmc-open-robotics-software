package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class EnoughAreaSnapChecker implements SnapBasedCheckerComponent
{
   private static final boolean DEBUG = false;

   private final FootstepPlannerParameters parameters;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepNodeSnapper snapper;

   private BipedalFootstepPlannerNodeRejectionReason rejectionReason;

   public EnoughAreaSnapChecker(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.footPolygons = footPolygons;
      this.snapper = snapper;
   }

   @Override
   public void setFootstepGraph(FootstepGraph graph)
   {

   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {

   }

   @Override
   public boolean isNodeValid(FootstepNode nodeToCheck, FootstepNode previousNode)
   {
      FootstepNodeSnapData snapData = snapper.getSnapData(nodeToCheck);
      ConvexPolygon2D footholdAfterSnap = snapData.getCroppedFoothold();
      double area = footholdAfterSnap.getArea();
      double footArea = footPolygons.get(nodeToCheck.getRobotSide()).getArea();

      double epsilonAreaPercentage = 1e-4;
      if (!footholdAfterSnap.isEmpty() && area < (parameters.getMinimumFootholdPercent() - epsilonAreaPercentage) * footArea)
      {
         if (DEBUG)
         {
            PrintTools.debug("Node does not have enough foothold area. It only has " + Math.floor(100.0 * area / footArea) + "% foothold:\n" + nodeToCheck);
         }
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA;
         return false;
      }

      rejectionReason = null;
      return true;
   }

   @Override
   public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
   {
      return rejectionReason;
   }
}
