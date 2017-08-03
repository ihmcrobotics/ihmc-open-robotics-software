package us.ihmc.footstepPlanning.aStar;

import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.aStar.implementations.*;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class AStarFootstepPlanner implements FootstepPlanner
{
   public static final double DEFAULT_COST_PER_STEP = 0.005;
   public static final double DEFAULT_YAW_WEIGHT = 0.1;
   public static final double DEFAULT_STEP_WIDTH = 0.25;

   private FootstepGraph graph;
   private SideDependentList<FootstepNode> goalNodes;
   private FootstepNode startNode;
   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;
   private FootstepNode goalNode;

   private PlanarRegionsList planarRegionsList;
   private SideDependentList<ConvexPolygon2D> footPolygons;

   private final FootstepNodeChecker nodeChecker;
   private final GraphVisualization visualization;
   private final CostToGoHeuristics heuristics;
   private final FootstepNodeExpansion nodeExpansion;
   private final FootstepCost stepCostCalculator;
   private final FootstepNodeSnapper snapper;

   private double timeout = Double.POSITIVE_INFINITY;

   public AStarFootstepPlanner(FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics, FootstepNodeExpansion expansion, FootstepCost stepCostCalculator,
                               FootstepNodeSnapper snapper)
   {
      this(nodeChecker, heuristics, expansion, stepCostCalculator, snapper, null);
   }

   public AStarFootstepPlanner(FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics, FootstepNodeExpansion nodeExpansion,
         FootstepCost stepCostCalculator, FootstepNodeSnapper snapper, GraphVisualization visualization)
   {
      this.nodeChecker = nodeChecker;
      this.heuristics = heuristics;
      this.nodeExpansion = nodeExpansion;
      this.stepCostCalculator = stepCostCalculator;
      this.visualization = visualization;
      this.snapper = snapper;
   }

   public void setWeight(double weight)
   {
      heuristics.setWeight(weight);
   }

   public void setTimeout(double timeoutInSeconds)
   {
      timeout = timeoutInSeconds;
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
      ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
      goalNodes = new SideDependentList<FootstepNode>();

      for (RobotSide side : RobotSide.values)
      {
         FramePose goalNodePose = new FramePose(goalFrame);
         goalNodePose.setY(side.negateIfRightSide(DEFAULT_STEP_WIDTH / 2.0));
         goalNodePose.changeFrame(goalPose.getReferenceFrame());
         FootstepNode goalNode = new FootstepNode(goalNodePose.getX(), goalNodePose.getY(), goalNodePose.getYaw(), side);

         boolean validGoalNode = snapper.snapFootstepNode(goalNode);
//         if(!validGoalNode)
//            throw new RuntimeException("Invalid goal node. Failed to snap to planar regions");

         goalNodes.put(side, goalNode);
      }
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      snapper.setPlanarRegions(planarRegionsList);
   }

   public void setFootPolygons(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.footPolygons = footPolygons;
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
      if (!graph.doesNodeExist(goalNode))
         return null;

      FootstepPlan plan = new FootstepPlan();
      List<FootstepNode> path = graph.getPathFromStart(goalNode);
      for (int i = 1; i < path.size(); i++)
      {
         RobotSide robotSide = path.get(i).getRobotSide();
         RigidBodyTransform soleTransform = path.get(i).getSoleTransform();
         plan.addFootstep(robotSide, new FramePose(ReferenceFrame.getWorldFrame(), soleTransform));
      }

      return plan;
   }

   private void initialize()
   {
      if (startNode == null)
         throw new RuntimeException("Need to set initial conditions before planning.");
      if (goalNodes == null)
         throw new RuntimeException("Need to set goal before planning.");

      graph = new FootstepGraph(startNode);
      NodeComparator nodeComparator = new NodeComparator(graph, goalNodes, heuristics);
      stack = new PriorityQueue<>(nodeComparator);

      boolean validStartNode = snapper.snapFootstepNode(startNode);
      if(!validStartNode)
         throw new RuntimeException("Start node doesn't snap");

      for(RobotSide robotSide : RobotSide.values)
      {
         boolean validGoalNode = snapper.snapFootstepNode(goalNodes.get(robotSide));
         if(!validGoalNode)
            throw new RuntimeException("Goal node doesn't snap to planar regions");
      }
      
      stack.add(startNode);
      expandedNodes = new HashSet<>();
      goalNode = null;

      if (visualization != null)
      {
         visualization.addNode(startNode, true);
         for (RobotSide side : RobotSide.values)
            visualization.addNode(goalNodes.get(side), true);
         visualization.tickAndUpdate();
      }
   }

   private void planInternal()
   {
      long planningStartTime = System.nanoTime();

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

         RobotSide nodeSide = nodeToExpand.getRobotSide();
         if (nodeToExpand.equals(goalNodes.get(nodeSide)))
         {
            goalNode = goalNodes.get(nodeSide.getOppositeSide());
            graph.checkAndSetEdge(nodeToExpand, goalNode, 0.0);
            break;
         }

         HashSet<FootstepNode> neighbors = nodeExpansion.expandNode(nodeToExpand);
         for (FootstepNode neighbor : neighbors)
         {
            boolean successfulSnap = snapper.snapFootstepNode(neighbor);
            if(!successfulSnap)
               continue;

            if (!nodeChecker.isNodeValid(neighbor, nodeToExpand))
               continue;

            double cost = stepCostCalculator.compute(nodeToExpand, neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);
            stack.add(neighbor);
         }

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime) > timeout)
            break;
      }
   }

   private FootstepPlanningResult checkResult()
   {
      if (stack.isEmpty())
         return FootstepPlanningResult.NO_PATH_EXISTS;
      if (!graph.doesNodeExist(goalNode))
         return FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION;

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

   public static AStarFootstepPlanner createFlatGroundPlanner(GraphVisualization viz)
   {
      AlwaysValidNodeChecker nodeChecker = new AlwaysValidNodeChecker();
      SimpleSideBasedExpansion expansion = new SimpleSideBasedExpansion();
      FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(DEFAULT_YAW_WEIGHT);
      DistanceAndYawBasedCost stepCostCalculator = new DistanceAndYawBasedCost(DEFAULT_COST_PER_STEP, DEFAULT_YAW_WEIGHT);

      return new AStarFootstepPlanner(nodeChecker, heuristics, expansion, stepCostCalculator, snapper, viz);
   }

   public static AStarFootstepPlanner createRoughTerrainPlanner(GraphVisualization viz, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      StepHeightBasedNodeChecker nodeChecker = new StepHeightBasedNodeChecker();
      SimpleSideBasedExpansion expansion = new SimpleSideBasedExpansion();
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(DEFAULT_YAW_WEIGHT);
      DistanceAndYawBasedCost stepCostCalculator = new DistanceAndYawBasedCost(DEFAULT_COST_PER_STEP, DEFAULT_YAW_WEIGHT);

      heuristics.setWeight(1.5);

      AStarFootstepPlanner planner = new AStarFootstepPlanner(nodeChecker, heuristics, expansion, stepCostCalculator, snapper, viz);
      planner.setFootPolygons(footPolygons);
      return planner;
   }
}
