package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SnapBasedNodeChecker extends FootstepNodeChecker
{
   private static final boolean DEBUG = false;

   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapper snapper;

   public SnapBasedNodeChecker(FootstepPlannerParameters parameters, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      super.setPlanarRegions(planarRegions);
      snapper.setPlanarRegions(planarRegions);
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode nodeToCheck)
   {
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(nodeToCheck);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();
      if (snapTransform.containsNaN())
      {
         if (DEBUG)
         {
            PrintTools.debug("Was not able to snap node:\n" + nodeToCheck);
         }
         rejectNode(nodeToCheck, QuadrupedFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return false;
      }

      if (snapTransform.getM22() < Math.cos(parameters.getMinimumSurfaceInclineRadians()))
      {
         if (DEBUG)
         {
            PrintTools.debug("Surface incline was too steep at radians = " + Math.acos(snapTransform.getM22()) + "\n" + nodeToCheck);
         }
         rejectNode(nodeToCheck, QuadrupedFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);

         return false;
      }


      return true;
   }

   @Override
   public void addStartNode(FootstepNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransforms)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         snapper.addSnapData(startNode.getXIndex(robotQuadrant), startNode.getYIndex(robotQuadrant),
                             new FootstepNodeSnapData(startNodeTransforms.get(robotQuadrant)));
      }
   }
}
