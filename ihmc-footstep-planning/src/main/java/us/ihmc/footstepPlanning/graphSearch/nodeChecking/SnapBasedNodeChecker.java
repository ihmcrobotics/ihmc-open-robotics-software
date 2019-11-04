package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class SnapBasedNodeChecker extends FootstepNodeChecker
{
   private final FootstepNodeSnapper snapper;

   private final List<SnapBasedCheckerComponent> checkerComponents = new ArrayList<>();

   public SnapBasedNodeChecker(FootstepPlannerParametersReadOnly parameters, SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapper snapper)
   {
      this.snapper = snapper;

      addSnapBasedCheckerComponent(new ValidSnapChecker(snapper));
      addSnapBasedCheckerComponent(new SurfaceInclineSnapChecker(parameters, snapper));
      addSnapBasedCheckerComponent(new ObstacleBetweenNodesChecker(parameters, snapper));
      addSnapBasedCheckerComponent(new EnoughAreaSnapChecker(parameters, footPolygons, snapper));
      addSnapBasedCheckerComponent(new GoodFootstepPositionChecker(parameters, snapper));
   }

   public void addSnapBasedCheckerComponent(SnapBasedCheckerComponent component)
   {
      checkerComponents.add(component);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      super.setPlanarRegions(planarRegions);
      for (int i = 0; i < checkerComponents.size(); i++)
      {
         checkerComponents.get(i).setPlanarRegions(planarRegions);
      }
      snapper.setPlanarRegions(planarRegions);
   }

   @Override
   public void addFootstepGraph(FootstepGraph graph)
   {
      this.graph = graph;
      for (int i = 0; i < checkerComponents.size(); i++)
      {
         checkerComponents.get(i).setFootstepGraph(graph);
      }
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
      snapper.addSnapData(startNode, new FootstepNodeSnapData(startNodeTransform));
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode)
   {
      for (int i = 0; i < checkerComponents.size(); i++)
      {
         SnapBasedCheckerComponent component = checkerComponents.get(i);
         if (!component.isNodeValid(node, previousNode))
         {
            rejectNode(node, previousNode, component.getRejectionReason());
            return false;
         }
      }

      return true;
   }



}
