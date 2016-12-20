package us.ihmc.footstepPlanning.aStar;

import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class AStarFootstepPlanner implements FootstepPlanner
{
   private FootstepGraph graph;
   private FootstepNode goalNode;
   private FootstepNode startNode;
   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;

   private final FootstepNodeChecker nodeChecker;
   private final GraphVisualization visualization;
   private final CostToGoHeuristics heuristics;
   private final FootstepNodeExpansion nodeExpansion;

   public AStarFootstepPlanner(FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics, FootstepNodeExpansion expansion)
   {
      this(nodeChecker, heuristics, expansion, null);
   }

   public AStarFootstepPlanner(FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics, FootstepNodeExpansion nodeExpansion,
         GraphVisualization visualization)
   {
      this.nodeChecker = nodeChecker;
      this.heuristics = heuristics;
      this.nodeExpansion = nodeExpansion;
      this.visualization = visualization;
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side)
   {
      startNode = new FootstepNode(stanceFootPose.getX(), stanceFootPose.getY(), stanceFootPose.getYaw(), side);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      checkGoalType(goal);
      FramePose goalPose = goal.getGoalPoseBetweenFeet();
      goalNode = new FootstepNode(goalPose.getX(), goalPose.getY(), goalPose.getYaw(), RobotSide.LEFT);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      nodeChecker.setPlanarRegions(planarRegionsList);
   }

   @Override
   public FootstepPlanningResult plan()
   {
      initialize();
      planInternal();
      return checkResult();
   }

   @Override
   public FootstepPlan getPlan()
   {
      FootstepPlan plan = new FootstepPlan();
      List<FootstepNode> path = graph.getPathFromStart(goalNode);
      for (FootstepNode node : path)
         plan.addFootstep(node.getRobotSide(), createPoseFromNode(node));
      return plan;
   }

   private FramePose createPoseFromNode(FootstepNode node)
   {
      FramePose pose = new FramePose(ReferenceFrame.getWorldFrame());
      pose.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      pose.setX(node.getX());
      pose.setY(node.getY());
      return pose;
   }

   private void initialize()
   {
      if (startNode == null)
         throw new RuntimeException("Need to set initial conditions before planning.");
      if (goalNode == null)
         throw new RuntimeException("Need to set goal before planning.");

      graph = new FootstepGraph(startNode);
      NodeComparator nodeComparator = new NodeComparator(graph, goalNode, heuristics);
      stack = new PriorityQueue<>(nodeComparator);
      stack.add(startNode);
      expandedNodes = new HashSet<>();

      if (visualization != null)
      {
         visualization.addNode(startNode, true);
         visualization.addNode(goalNode, true);
         visualization.tickAndUpdate();
      }
   }

   private void planInternal()
   {
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

         HashSet<FootstepNode> neighbors = nodeExpansion.expandNode(nodeToExpand);
         for (FootstepNode neighbor : neighbors)
         {
            if (!nodeChecker.isNodeValid(neighbor))
               continue;

            double cost = nodeToExpand.euclideanDistance(neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);
            stack.add(neighbor);
         }
      }
   }

   private FootstepPlanningResult checkResult()
   {
      if (stack.isEmpty())
         return FootstepPlanningResult.NO_PATH_EXISTS;

      if (visualization != null)
      {
         List<FootstepNode> path = graph.getPathFromStart(goalNode);
         for (FootstepNode node : path)
            visualization.setNodeActive(node);
         visualization.tickAndUpdate();
      }

      if (heuristics.getWeight() <= 1.0)
         return FootstepPlanningResult.OPTIMAL_SOLUTION;
      return FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;
   }

   private void checkGoalType(FootstepPlannerGoal goal)
   {
      FootstepPlannerGoalType supportedGoalType = FootstepPlannerGoalType.POSE_BETWEEN_FEET;
      if (!(goal.getFootstepPlannerGoalType() == supportedGoalType))
         throw new RuntimeException("Planner does not support goals other then " + supportedGoalType);
   }
}
