package us.ihmc.footstepPlanning.aStar;

import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class AStarFootstepPlanner implements FootstepPlanner
{
   private FootstepGraph graph;
   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;
   private FootstepNodeValidityChecker nodeChecker;

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
      nodeChecker = new FootstepNodeValidityChecker(planarRegionsList);
   }

   @Override
   public FootstepPlanningResult plan()
   {
      FootstepNode startNode = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      FootstepNode goalNode = new FootstepNode(1.0, 0.0, 0.0, RobotSide.LEFT);

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

         HashSet<FootstepNode> neighbors = computeNeighbors(nodeToExpand);
         for (FootstepNode neighbor : neighbors)
         {
            if (!nodeChecker.isNodeValid(neighbor))
               continue;

            double cost = nodeToExpand.euclideanDistance(neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);
            stack.add(neighbor);
         }
      }

      if (stack.isEmpty())
         return FootstepPlanningResult.NO_PATH_EXISTS;

      if (visualization != null)
      {
         List<FootstepNode> path = graph.getPathFromStart(goalNode);
         for (FootstepNode node : path)
            visualization.setNodeActive(node);
         visualization.tickAndUpdate();
      }

      return FootstepPlanningResult.OPTIMAL_SOLUTION;
   }

   private HashSet<FootstepNode> computeNeighbors(FootstepNode node)
   {
      RobotSide stepSide = node.getRobotSide().getOppositeSide();
      double yOffset = stepSide.negateIfLeftSide(FootstepNode.gridSizeY);
      double stanceYaw = node.getYaw();

      HashSet<FootstepNode> neighbors = new HashSet<>();
      double yawGrid = FootstepNode.gridSizeYaw;
      double[] neighborYaws = new double[] {stanceYaw - yawGrid, stanceYaw, stanceYaw + yawGrid};

      for (int i = 0; i < neighborYaws.length; i++)
      {
         double neighborYaw = neighborYaws[i];
         Vector3d offset1 = new Vector3d(FootstepNode.gridSizeX, yOffset, 0.0);
         Vector3d offset2 = new Vector3d(-FootstepNode.gridSizeX, yOffset, 0.0);
         Vector3d offset3 = new Vector3d(0.0, yOffset, 0.0);

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setRotationYawAndZeroTranslation(neighborYaw);
         transform.transform(offset1);
         transform.transform(offset2);
         transform.transform(offset3);

         neighbors.add(new FootstepNode(node.getX() + offset1.getX(), node.getY() + offset1.getY(), neighborYaw, stepSide));
         neighbors.add(new FootstepNode(node.getX() + offset2.getX(), node.getY() + offset2.getY(), neighborYaw, stepSide));
         neighbors.add(new FootstepNode(node.getX() + offset3.getX(), node.getY() + offset3.getY(), neighborYaw, stepSide));
      }

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
