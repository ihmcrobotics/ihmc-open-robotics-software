package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;

import java.util.ArrayList;
import java.util.List;

public class NavigableRegionsListener
{
   private final List<VisibilityGraphNode> expandedNodes = new ArrayList<>();

   public void addExpandedNode(VisibilityGraphNode expandedNode)
   {
      expandedNodes.add(expandedNode);
   }

   public void addEdge(VisibilityGraphEdge edge)
   {
   }

   public List<VisibilityGraphNode> getExpandedNodes()
   {
      return expandedNodes;
   }
}
