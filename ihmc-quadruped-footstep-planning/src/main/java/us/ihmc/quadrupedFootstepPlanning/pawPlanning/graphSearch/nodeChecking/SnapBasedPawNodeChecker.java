package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawStepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SnapBasedPawNodeChecker extends PawNodeChecker
{
   private static final boolean DEBUG = false;

   private final PawStepPlannerParametersReadOnly parameters;
   private final PawNodeSnapper snapper;

   public SnapBasedPawNodeChecker(PawStepPlannerParametersReadOnly parameters, PawNodeSnapper snapper)
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
   public boolean isNodeValidInternal(PawNode nodeToCheck)
   {
      PawNodeSnapData snapData = snapper.snapPawNode(nodeToCheck);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();
      if (snapTransform.containsNaN())
      {
         if (DEBUG)
         {
            PrintTools.debug("Was not able to snap node:\n" + nodeToCheck);
         }
         rejectNode(nodeToCheck, PawStepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return false;
      }

      if (snapTransform.getM22() < Math.cos(parameters.getMinimumSurfaceInclineRadians()))
      {
         if (DEBUG)
         {
            PrintTools.debug("Surface incline was too steep at radians = " + Math.acos(snapTransform.getM22()) + "\n" + nodeToCheck);
         }
         rejectNode(nodeToCheck, PawStepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);

         return false;
      }


      return true;
   }

   @Override
   public void addStartNode(PawNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransforms)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         snapper.addSnapData(startNode.getXIndex(robotQuadrant), startNode.getYIndex(robotQuadrant),
                             new PawNodeSnapData(startNodeTransforms.get(robotQuadrant)));
      }
   }
}
