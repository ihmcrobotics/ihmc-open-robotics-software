package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.PrintTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class ValidSnapChecker implements SnapBasedCheckerComponent
{
   private static final boolean DEBUG = false;

   private final FootstepNodeSnapper snapper;

   private BipedalFootstepPlannerNodeRejectionReason rejectionReason;

   public ValidSnapChecker(FootstepNodeSnapper snapper)
   {
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
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(node);
      if (snapData.getSnapTransform().containsNaN())
      {
         if (DEBUG)
         {
            PrintTools.debug("Was not able to snap node:\n" + node);
         }
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP;
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
