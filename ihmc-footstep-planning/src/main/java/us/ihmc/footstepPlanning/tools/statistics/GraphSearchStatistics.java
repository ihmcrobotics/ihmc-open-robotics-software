package us.ihmc.footstepPlanning.tools.statistics;

import org.jgrapht.Graph;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerLatticeMap;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerNodeDataList;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.statistics.StatisticsType;

public class GraphSearchStatistics implements PlannerStatistics<GraphSearchStatistics>
{
   private PlannerLatticeMap expandedNodes;
   private PlannerNodeDataList fullGraph;

   public GraphSearchStatistics()
   {
   }

   public GraphSearchStatistics(GraphSearchStatistics other)
   {
      this();
      set(other);
   }

   public StatisticsType getStatisticsType()
   {
      return StatisticsType.GRAPH_SEARCH;
   }

   public void set(GraphSearchStatistics other)
   {
      setExpandedNodes(other.expandedNodes);
      setFullGraph(other.fullGraph);
   }

   public void setExpandedNodes(PlannerLatticeMap expandedNodes)
   {
      this.expandedNodes = expandedNodes;
   }

   public void setFullGraph(PlannerNodeDataList fullGraph)
   {
      this.fullGraph = fullGraph;
   }

   public PlannerLatticeMap getExpandedNodes()
   {
      return expandedNodes;
   }

   public PlannerNodeDataList getFullGraph()
   {
      return fullGraph;
   }
}
