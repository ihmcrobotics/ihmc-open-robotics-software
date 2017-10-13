package us.ihmc.footstepPlanning.graphSearch.planners;

import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.*;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.GraphVisualization;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.NodeComparator;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AStarFootstepPlanner implements FootstepPlanner
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final FootstepPlannerParameters parameters;

   private SideDependentList<FootstepNode> goalNodes;
   private FootstepNode startNode;
   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;
   private FootstepNode goalNode;

   private final FootstepGraph graph;
   private final FootstepNodeChecker nodeChecker;
   private final GraphVisualization visualization;
   private final CostToGoHeuristics heuristics;
   private final FootstepNodeExpansion nodeExpansion;
   private final FootstepCost stepCostCalculator;
   private final FootstepNodeSnapper snapper;

   private final YoDouble timeout;

   public AStarFootstepPlanner(FootstepPlannerParameters parameters, FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics,
                               FootstepNodeExpansion expansion, FootstepCost stepCostCalculator, FootstepNodeSnapper snapper, YoVariableRegistry parentRegistry)
   {
      this(parameters, nodeChecker, heuristics, expansion, stepCostCalculator, snapper, null, parentRegistry);
   }

   public AStarFootstepPlanner(FootstepPlannerParameters parameters, FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics,
                               FootstepNodeExpansion nodeExpansion, FootstepCost stepCostCalculator, FootstepNodeSnapper snapper,
                               GraphVisualization visualization, YoVariableRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.nodeChecker = nodeChecker;
      this.heuristics = heuristics;
      this.nodeExpansion = nodeExpansion;
      this.stepCostCalculator = stepCostCalculator;
      this.visualization = visualization;
      this.snapper = snapper;
      this.graph = new FootstepGraph();

      this.timeout = new YoDouble("timeout", registry);
      timeout.set(Double.POSITIVE_INFINITY);

      parentRegistry.addChild(registry);
   }

   public void setWeight(double weight)
   {
      heuristics.setWeight(weight);
   }

   public void setTimeout(double timeoutInSeconds)
   {
      timeout.set(timeoutInSeconds);
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
         goalNodePose.setY(side.negateIfRightSide(parameters.getIdealFootstepWidth() / 2.0));
         goalNodePose.changeFrame(goalPose.getReferenceFrame());
         FootstepNode goalNode = new FootstepNode(goalNodePose.getX(), goalNodePose.getY(), goalNodePose.getYaw(), side);
         goalNodes.put(side, goalNode);
      }
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      nodeChecker.setPlanarRegions(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);
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

         RigidBodyTransform footstepPose = new RigidBodyTransform();
         footstepPose.setRotationYawAndZeroTranslation(path.get(i).getYaw());
         footstepPose.setTranslationX(path.get(i).getX());
         footstepPose.setTranslationY(path.get(i).getY());

         FootstepNodeSnapData snapData = snapper.snapFootstepNode(path.get(i));
         RigidBodyTransform snapTransform = snapData.getSnapTransform();
         snapTransform.transform(footstepPose);
         plan.addFootstep(robotSide, new FramePose(ReferenceFrame.getWorldFrame(), footstepPose));
      }

      return plan;
   }

   private void initialize()
   {
      if (startNode == null)
         throw new RuntimeException("Need to set initial conditions before planning.");
      if (goalNodes == null)
         throw new RuntimeException("Need to set goal before planning.");

      graph.initialize(startNode);
      snapper.addStartNode(startNode);
      NodeComparator nodeComparator = new NodeComparator(graph, goalNodes, heuristics);
      stack = new PriorityQueue<>(nodeComparator);

      boolean validStartNode = nodeChecker.isNodeValid(startNode, null);
      if (!validStartNode)
         throw new RuntimeException("Start node isn't valid");

      for (RobotSide robotSide : RobotSide.values)
      {
         boolean validGoalNode = nodeChecker.isNodeValid(goalNodes.get(robotSide), null);
         if (!validGoalNode)
            throw new RuntimeException("Goal node isn't valid");
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
         if (nodeToExpand.equals(goalNodes.get(nodeSide))) // ?
         {
            goalNode = goalNodes.get(nodeSide.getOppositeSide());
            graph.checkAndSetEdge(nodeToExpand, goalNode, 0.0);
            break;
         }

         HashSet<FootstepNode> neighbors = nodeExpansion.expandNode(nodeToExpand);
         for (FootstepNode neighbor : neighbors)
         {
            /** Checks if the footstep (center of the foot) is on a planar region*/
            if (!nodeChecker.isNodeValid(neighbor, nodeToExpand))
               continue;

            double cost = stepCostCalculator.compute(nodeToExpand, neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);
            stack.add(neighbor);
         }

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime) > timeout.getDoubleValue())
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

   public static AStarFootstepPlanner createFlatGroundPlanner(FootstepPlannerParameters parameters, GraphVisualization viz,
                                                              SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeExpansion expansion,
                                                              YoVariableRegistry registry)
   {
      AlwaysValidNodeChecker nodeChecker = new AlwaysValidNodeChecker();
      FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(parameters, registry);
      DistanceAndYawBasedCost stepCostCalculator = new DistanceAndYawBasedCost(parameters);

      return new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, stepCostCalculator, snapper, viz, registry);
   }

   public static AStarFootstepPlanner createRoughTerrainPlanner(FootstepPlannerParameters parameters, GraphVisualization viz,
                                                                SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeExpansion expansion,
                                                                YoVariableRegistry registry)
   {
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeSnapAndWiggler postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters, null);

      SnapBasedNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(parameters, registry);
      DistanceAndYawBasedCost stepCostCalculator = new DistanceAndYawBasedCost(parameters);

      heuristics.setWeight(1.5);

      AStarFootstepPlanner planner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, stepCostCalculator, postProcessingSnapper, viz, registry);
      return planner;
   }
}
