package us.ihmc.footstepPlanning.aStar;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

public class AStarFootstepPlanner implements FootstepPlanner
{
   private FootstepGraph graph;
   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;

   private final GraphVisualization visualization;

   public AStarFootstepPlanner()
   {
      this(null);
   }

   public AStarFootstepPlanner(GraphVisualization visualization)
   {
      this.visualization = visualization;
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public FootstepPlanningResult plan()
   {
      FootstepNode startNode = new FootstepNode(0.0, 0.0);
      FootstepNode goalNode = new FootstepNode(1.0, 0.0);

      if (visualization != null)
      {
         visualization.addNode(startNode, true);
         visualization.addNode(goalNode, true);
         visualization.tickAndUpdate();
      }

      graph = new FootstepGraph(startNode);
      stack = new PriorityQueue<>(new NodeComparator(graph, goalNode));
      expandedNodes = new HashSet<>();

      stack.add(startNode);

      while (!stack.isEmpty())
      {
         FootstepNode nodeToExpand = stack.poll();
         if (expandedNodes.contains(nodeToExpand))
            continue;
         expandedNodes.add(nodeToExpand);

         if (visualization != null)
         {
            visualization.addNode(nodeToExpand, false);
            visualization.tickAndUpdate();
         }

         if (nodeToExpand.equals(goalNode))
            break;

         List<FootstepNode> neighbors = computeNeighbors(nodeToExpand);
         for (FootstepNode neighbor : neighbors)
         {
            double cost = nodeToExpand.euclideanDistance(neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);
            stack.add(neighbor);
         }
      }

      List<FootstepNode> path = graph.getPathFromStart(goalNode);
      if (visualization != null)
      {
         for (FootstepNode node : path)
            visualization.setNodeActive(node);
         visualization.tickAndUpdate();
      }

      if (stack.isEmpty())
         return FootstepPlanningResult.NO_PATH_EXISTS;
      return FootstepPlanningResult.OPTIMAL_SOLUTION;
   }

   private List<FootstepNode> computeNeighbors(FootstepNode node)
   {
      ArrayList<FootstepNode> neighbors = new ArrayList<>();
      neighbors.add(new FootstepNode(node.getX() + FootstepNode.gridSizeX, node.getY()));
      neighbors.add(new FootstepNode(node.getX() - FootstepNode.gridSizeX, node.getY()));
      neighbors.add(new FootstepNode(node.getX(), node.getY() + FootstepNode.gridSizeY));
      neighbors.add(new FootstepNode(node.getX(), node.getY() - FootstepNode.gridSizeY));
      return neighbors;
   }

   @Override
   public FootstepPlan getPlan()
   {
      // TODO Auto-generated method stub
      return null;
   }

   private class NodeComparator implements Comparator<FootstepNode>
   {
      private final FootstepGraph graph;
      private final FootstepNode goalNode;

      public NodeComparator(FootstepGraph graph, FootstepNode goalNode)
      {
         this.graph = graph;
         this.goalNode = goalNode;
      }

      @Override
      public int compare(FootstepNode o1, FootstepNode o2)
      {
         double cost1 = graph.getCostFromStart(o1) + FootstepHeuristics.computeEuclidianHeuristics(o1, goalNode);
         double cost2 = graph.getCostFromStart(o2) + FootstepHeuristics.computeEuclidianHeuristics(o2, goalNode);
         if (cost1 == cost2) return 0;
         return cost1 < cost2 ? -1 : 1;
      }

   }

}
