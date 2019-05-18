package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.CliffDetectionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PlanarRegionCliffAvoider extends FootstepNodeChecker
{
   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapperReadOnly snapper;

   private FootstepNode startNode;

   public PlanarRegionCliffAvoider(FootstepPlannerParameters parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public void addStartNode(FootstepNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransform)
   {
      this.startNode = startNode;
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode)
   {
      if (startNode != null && startNode.equals(node))
         return true;

      if (!hasPlanarRegions())
         return true;



      RobotQuadrant movingQuadrant = node.getMovingQuadrant();
      int xIndex = node.getXIndex(movingQuadrant);
      int yIndex = node.getYIndex(movingQuadrant);
      RigidBodyTransform footTransformToWorld = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransformToWorld(xIndex, yIndex, snapper.getSnapData(xIndex, yIndex).getSnapTransform(), footTransformToWorld);

      Point3D footInWorld = new Point3D();
      footTransformToWorld.transform(footInWorld);

      boolean isNearCliff = CliffDetectionTools.isNearCliff(movingQuadrant, planarRegionsList, footInWorld, node.getNominalYaw(), parameters);

      if (isNearCliff)
      {
         rejectNode(node, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.AT_CLIFF_BOTTOM);
         return false;
      }

      return true;
   }


}
