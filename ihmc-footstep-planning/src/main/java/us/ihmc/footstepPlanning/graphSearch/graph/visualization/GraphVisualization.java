package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public interface GraphVisualization
{
   public void addNode(FootstepNode node, boolean active);

   public void setNodeActive(FootstepNode node);

   public void setNodeInactive(FootstepNode node);

   public boolean nodeExists(FootstepNode node);

   public void tickAndUpdate();
}
