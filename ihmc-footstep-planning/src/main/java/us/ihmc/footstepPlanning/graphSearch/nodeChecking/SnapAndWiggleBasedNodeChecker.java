package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SnapAndWiggleBasedNodeChecker extends FootstepNodeChecker
{
   private final FootstepNodeSnapAndWiggler snapAndWiggler;
   private final SnapBasedNodeChecker nodeChecker;

   public SnapAndWiggleBasedNodeChecker(SideDependentList<ConvexPolygon2D> footPolygons, FootstepPlannerParametersReadOnly parameters)
   {
      this.snapAndWiggler = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      this.nodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapAndWiggler);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.snapAndWiggler.setPlanarRegions(planarRegionsList);
      this.nodeChecker.setPlanarRegions(planarRegionsList);
   }

   @Override
   public void addPlannerListener(BipedalFootstepPlannerListener listener)
   {
      if (listener != null)
      {
         snapAndWiggler.addPlannerListener(listener);
         nodeChecker.addPlannerListener(listener);
      }
   }

   @Override
   public void addFootstepGraph(FootstepGraph graph)
   {
      nodeChecker.addFootstepGraph(graph);
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
      snapAndWiggler.addSnapData(startNode, new FootstepNodeSnapData(startNodeTransform));
      nodeChecker.addStartNode(startNode, startNodeTransform);
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode nodeToExpand, FootstepNode previousNode)
   {
      return nodeChecker.isNodeValidInternal(nodeToExpand, previousNode);
   }
}
