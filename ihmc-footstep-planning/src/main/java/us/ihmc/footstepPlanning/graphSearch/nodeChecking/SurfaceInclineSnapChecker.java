package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SurfaceInclineSnapChecker implements SnapBasedCheckerComponent
{
   private static final boolean DEBUG = false;

   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapper snapper;

   private BipedalFootstepPlannerNodeRejectionReason rejectionReason;

   public SurfaceInclineSnapChecker(FootstepPlannerParameters parameters, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
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

      RigidBodyTransform snappedSoleTransform = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransform(nodeToCheck, snapData.getSnapTransform(), snappedSoleTransform);
      if (snappedSoleTransform.getM22() < Math.cos(parameters.getMinimumSurfaceInclineRadians()))
      {
         if (DEBUG)
         {
            PrintTools.debug("Surface incline was too steep at radians = " + Math.acos(snappedSoleTransform.getM22()) + "\n" + nodeToCheck);
         }
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP;
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
