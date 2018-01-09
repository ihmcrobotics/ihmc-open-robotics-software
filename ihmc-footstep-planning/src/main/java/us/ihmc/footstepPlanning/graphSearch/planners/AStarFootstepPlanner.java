package us.ihmc.footstepPlanning.graphSearch.planners;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.*;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.GraphVisualization;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.NodeComparator;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeCheckerOfCheckers;
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
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class AStarFootstepPlanner implements FootstepPlanner
{
   private static final boolean debug = false;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final FootstepPlannerParameters parameters;

   private SideDependentList<FootstepNode> goalNodes;
   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;
   private FootstepNode startNode;
   private FootstepNode endNode;

   private final FootstepGraph graph;
   private final FootstepNodeChecker nodeChecker;
   private final GraphVisualization visualization;
   private final CostToGoHeuristics heuristics;
   private final FootstepNodeExpansion nodeExpansion;
   private final FootstepCost stepCostCalculator;
   private final FootstepNodeSnapper snapper;

   private final YoDouble timeout;
   private final YoDouble planningTime = new YoDouble("PlanningTime", registry);
   private final YoLong numberOfExpandedNodes = new YoLong("NumberOfExpandedNodex", registry);
   private final YoDouble percentRejectedNodes = new YoDouble("PercentRejectedNodes", registry);
   private final YoLong itarationCount = new YoLong("ItarationCount", registry);

   private final YoBoolean validGoalNode = new YoBoolean("validGoalNode", registry);

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

   @Override
   public void setTimeout(double timeoutInSeconds)
   {
      timeout.set(timeoutInSeconds);
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side)
   {
      startNode = new FootstepNode(stanceFootPose.getX(), stanceFootPose.getY(), stanceFootPose.getYaw(), side);
      RigidBodyTransform startNodeSnapTransform = FootstepNodeSnappingTools.computeSnapTransform(startNode, stanceFootPose.getGeometryObject());
      snapper.addSnapData(startNode, new FootstepNodeSnapData(startNodeSnapTransform));
      nodeChecker.addStartNode(startNode, startNodeSnapTransform);
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
      
      if(debug)
         PrintTools.info("A* planner has initialized");

      planInternal();
      FootstepPlanningResult result = checkResult();
      if (debug)
      {
         PrintTools.info("A* Footstep planning statistics for " + result);
         System.out.println("   Finished planning after " + Precision.round(planningTime.getDoubleValue(), 2) + " seconds.");
         System.out.println("   Expanded each node to an average of " + numberOfExpandedNodes.getLongValue() + " children nodes.");
         System.out.println("   Planning took a total of "+ itarationCount.getLongValue() + " iterations.");
         System.out.println("   During the planning " + percentRejectedNodes.getDoubleValue() + "% of nodes were rejected as invalid.");
      }
      return result;
   }

   @Override
   public FootstepPlan getPlan()
   {
      if (endNode == null || !graph.doesNodeExist(endNode))
         return null;

      FootstepPlan plan = new FootstepPlan();
      List<FootstepNode> path = graph.getPathFromStart(endNode);
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

         ConvexPolygon2D foothold = snapData.getCroppedFoothold();
         if (!foothold.isEmpty())
            plan.getFootstep(i - 1).setFoothold(foothold);
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
      NodeComparator nodeComparator = new NodeComparator(graph, goalNodes, heuristics);
      stack = new PriorityQueue<>(nodeComparator);

      validGoalNode.set(true);
      for (RobotSide robotSide : RobotSide.values)
      {
         boolean validGoalNode = nodeChecker.isNodeValid(goalNodes.get(robotSide), null);
         if (!validGoalNode && !parameters.getReturnBestEffortPlan())
            throw new RuntimeException("Goal node isn't valid. To plan without a valid goal node, best effort planning must be enabled");

         this.validGoalNode.set(validGoalNode && this.validGoalNode.getBooleanValue());
      }

      stack.add(startNode);
      expandedNodes = new HashSet<>();
      endNode = null;

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
      
      long rejectedNodesCount = 0;
      long expandedNodesCount = 0;
      long iterations = 0;

      while (!stack.isEmpty())
      {
         iterations++;
         
         FootstepNode nodeToExpand = stack.poll();
         if (expandedNodes.contains(nodeToExpand))
            continue;
         expandedNodes.add(nodeToExpand);

         if (visualization != null)
         {
            visualization.addNode(nodeToExpand, false);
            visualization.tickAndUpdate();
         }

         if (checkAndHandleNodeAtGoal(nodeToExpand))
            break;

         checkAndHandleBestEffortNode(nodeToExpand);

         HashSet<FootstepNode> neighbors = nodeExpansion.expandNode(nodeToExpand);
         expandedNodesCount += neighbors.size();
         for (FootstepNode neighbor : neighbors)
         {
            /** Checks if the footstep (center of the foot) is on a planar region*/
            if (!nodeChecker.isNodeValid(neighbor, nodeToExpand))
            {
               rejectedNodesCount++;
               continue;
            }

            double cost = stepCostCalculator.compute(nodeToExpand, neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);

            if(endNode == null || stack.comparator().compare(neighbor, endNode) < 0)
               stack.add(neighbor);
         }

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime) > timeout.getDoubleValue())
            break;
      }

      long timeInNano = System.nanoTime();
      planningTime.set(Conversions.nanosecondsToSeconds(timeInNano - planningStartTime));
      percentRejectedNodes.set(100.0 * rejectedNodesCount / expandedNodesCount);
      itarationCount.set(iterations);
      numberOfExpandedNodes.set(expandedNodesCount / iterations);
   }

   private boolean checkAndHandleNodeAtGoal(FootstepNode nodeToExpand)
   {
      if(!validGoalNode.getBooleanValue())
         return false;

      RobotSide nodeSide = nodeToExpand.getRobotSide();
      if (goalNodes.get(nodeSide).equals(nodeToExpand))
      {
         endNode = goalNodes.get(nodeSide.getOppositeSide());
         graph.checkAndSetEdge(nodeToExpand, endNode, 0.0);
         return true;
      }

      return false;
   }

   private void checkAndHandleBestEffortNode(FootstepNode nodeToExpand)
   {
      if(!parameters.getReturnBestEffortPlan())
         return;

      if(graph.getPathFromStart(nodeToExpand).size() - 1 < parameters.getMinimumStepsForBestEffortPlan())
         return;

      if(endNode == null || heuristics.compute(nodeToExpand, goalNodes.get(nodeToExpand.getRobotSide())) < heuristics.compute(endNode, goalNodes.get(endNode.getRobotSide())))
      {
         endNode = nodeToExpand;
      }
  }

   private FootstepPlanningResult checkResult()
   {
      if (stack.isEmpty() && endNode == null)
         return FootstepPlanningResult.NO_PATH_EXISTS;
      if (!graph.doesNodeExist(endNode))
         return FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION;

      if (visualization != null)
      {
         List<FootstepNode> path = graph.getPathFromStart(endNode);
         for (FootstepNode node : path)
            visualization.setNodeActive(node);
         visualization.tickAndUpdate();
      }

      if (heuristics.getWeight() <= 1.0)
         return FootstepPlanningResult.OPTIMAL_SOLUTION;
      return FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;
   }

   public static void checkGoalType(FootstepPlannerGoal goal)
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
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(parameters, footPolygons);
      FootstepNodeCheckerOfCheckers checkerOfCheckers = new FootstepNodeCheckerOfCheckers(Arrays.asList(nodeChecker, cliffAvoider));

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(parameters, registry);
      DistanceAndYawBasedCost stepCostCalculator = new DistanceAndYawBasedCost(parameters);

      heuristics.setWeight(1.5);

      AStarFootstepPlanner planner = new AStarFootstepPlanner(parameters, checkerOfCheckers, heuristics, expansion, stepCostCalculator, postProcessingSnapper, viz,
                                                              registry);
      return planner;
   }
}
