package us.ihmc.footstepPlanning.graphSearch.planners;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.BodyPathAndFootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.NodeComparator;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.listeners.HeuristicSearchAndActionPolicyDefinitions;
import us.ihmc.footstepPlanning.graphSearch.listeners.StartAndGoalListener;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.BodyCollisionNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeCheckerOfCheckers;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.PlanarRegionBaseOfCliffAvoider;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class AStarFootstepPlanner implements BodyPathAndFootstepPlanner
{
   private static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final FootstepPlannerParameters parameters;

   private SideDependentList<FootstepNode> goalNodes;
   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;
   private FootstepNode startNode;
   private FootstepNode endNode;

   private PlanarRegionsList planarRegionsList;

   private final FramePose3D goalPoseInWorld = new FramePose3D();

   private final FootstepGraph graph;
   private final FootstepNodeChecker nodeChecker;
   private final BipedalFootstepPlannerListener listener;
   private final CostToGoHeuristics heuristics;
   private final FootstepNodeExpansion nodeExpansion;
   private final FootstepCost stepCostCalculator;
   private final FootstepNodeSnapper snapper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private final ArrayList<StartAndGoalListener> startAndGoalListeners = new ArrayList<>();

   private final YoDouble timeout = new YoDouble("footstepPlannerTimeout", registry);
   private final YoDouble planningTime = new YoDouble("PlanningTime", registry);
   private final YoLong numberOfExpandedNodes = new YoLong("NumberOfExpandedNodes", registry);
   private final YoDouble percentRejectedNodes = new YoDouble("PercentRejectedNodes", registry);
   private final YoLong itarationCount = new YoLong("ItarationCount", registry);

   private final YoBoolean initialize = new YoBoolean("initialize", registry);

   private final YoBoolean validGoalNode = new YoBoolean("validGoalNode", registry);
   private final YoBoolean abortPlanning = new YoBoolean("abortPlanning", registry);

   public AStarFootstepPlanner(FootstepPlannerParameters parameters, FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics,
                               FootstepNodeExpansion expansion, FootstepCost stepCostCalculator, FootstepNodeSnapper snapper, YoVariableRegistry parentRegistry)
   {
      this(parameters, nodeChecker, heuristics, expansion, stepCostCalculator, snapper, null, null, parentRegistry);
   }

   public AStarFootstepPlanner(FootstepPlannerParameters parameters, FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics,
                               FootstepNodeExpansion nodeExpansion, FootstepCost stepCostCalculator, FootstepNodeSnapper snapper,
                               BipedalFootstepPlannerListener listener, SideDependentList<ConvexPolygon2D> footPolygons, YoVariableRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.nodeChecker = nodeChecker;
      this.heuristics = heuristics;
      this.nodeExpansion = nodeExpansion;
      this.stepCostCalculator = stepCostCalculator;
      this.listener = listener;
      this.snapper = snapper;
      this.graph = new FootstepGraph();
      timeout.set(Double.POSITIVE_INFINITY);
      this.initialize.set(true);
      this.footPolygons = footPolygons;

      nodeChecker.addFootstepGraph(graph);
      parentRegistry.addChild(registry);
   }

   public void addStartAndGoalListener(StartAndGoalListener startAndGoalListener)
   {
      startAndGoalListeners.add(startAndGoalListener);
   }

   @Override
   public void setTimeout(double timeoutInSeconds)
   {
      timeout.set(timeoutInSeconds);
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (debug)
            LogTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }
      startNode = new FootstepNode(stanceFootPose.getX(), stanceFootPose.getY(), stanceFootPose.getYaw(), side);
      RigidBodyTransform startNodeSnapTransform = FootstepNodeSnappingTools.computeSnapTransform(startNode, stanceFootPose);
      snapper.addSnapData(startNode, new FootstepNodeSnapData(startNodeSnapTransform));
      nodeChecker.addStartNode(startNode, startNodeSnapTransform);

      startAndGoalListeners.parallelStream().forEach(listener -> listener.setInitialPose(stanceFootPose));
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      checkGoalType(goal);

      goalNodes = new SideDependentList<>();

      SideDependentList<FramePose3D> goalPoses = new SideDependentList<>();

      if (goal.getFootstepPlannerGoalType().equals(FootstepPlannerGoalType.POSE_BETWEEN_FEET))
      {
         FramePose3D goalPose = goal.getGoalPoseBetweenFeet();
         ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
         for (RobotSide side : RobotSide.values)
         {
            FramePose3D goalNodePose = new FramePose3D(goalFrame);
            goalNodePose.setY(side.negateIfRightSide(parameters.getIdealFootstepWidth() / 2.0));
            goalNodePose.changeFrame(goalPose.getReferenceFrame());
            FootstepNode goalNode = new FootstepNode(goalNodePose.getX(), goalNodePose.getY(), goalNodePose.getYaw(), side);
            goalNodes.put(side, goalNode);

            goalNodePose.changeFrame(ReferenceFrame.getWorldFrame());
            goalPoses.put(side, goalNodePose);
         }
      }
      else if (goal.getFootstepPlannerGoalType().equals(FootstepPlannerGoalType.DOUBLE_FOOTSTEP))
      {
         SideDependentList<SimpleFootstep> goalSteps = goal.getDoubleFootstepGoal();
         for (RobotSide side : RobotSide.values)
         {
            FramePose3D goalNodePose = new FramePose3D();
            goalSteps.get(side).getSoleFramePose(goalNodePose);
            FootstepNode goalNode = new FootstepNode(goalNodePose.getX(), goalNodePose.getY(), goalNodePose.getYaw(), side);
            goalNodes.put(side, goalNode);

            goalNodePose.changeFrame(ReferenceFrame.getWorldFrame());
            goalPoses.put(side, goalNodePose);
         }
      }

      goalPoseInWorld.interpolate(goalPoses.get(RobotSide.LEFT), goalPoses.get(RobotSide.RIGHT), 0.5);
      startAndGoalListeners.parallelStream().forEach(listener -> listener.setGoalPose(goalPoseInWorld));
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      nodeChecker.setPlanarRegions(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      if (initialize.getBooleanValue())
      {
         boolean success = initialize();
         initialize.set(false);
         if (!success)
            return FootstepPlanningResult.PLANNER_FAILED;
      }

      if (debug)
      {
         LogTools.info("A* planner has initialized");
         LogTools.info("Planning. startNode = " + startNode, this);
         LogTools.info("endNode = " + endNode, this);
      }

      boolean success = planInternal();
      if (!success)
         return FootstepPlanningResult.PLANNER_FAILED;

      FootstepPlanningResult result = checkResult();

      if (result.validForExecution() && listener != null)
         listener.plannerFinished(null);

      if (debug)
      {
         LogTools.info("A* Footstep planning statistics for " + result);
         System.out.println("   Finished planning after " + Precision.round(planningTime.getDoubleValue(), 2) + " seconds.");
         System.out.println("   Expanded each node to an average of " + numberOfExpandedNodes.getLongValue() + " children nodes.");
         System.out.println("   Planning took a total of " + itarationCount.getLongValue() + " iterations.");
         System.out.println("   During the planning " + percentRejectedNodes.getDoubleValue() + "% of nodes were rejected as invalid.");
         System.out.println("   Goal was : " + goalPoseInWorld);
      }

      initialize.set(true);
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
         plan.addFootstep(robotSide, new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPose));

         ConvexPolygon2D foothold = snapData.getCroppedFoothold();
         SimpleFootstep footstep = plan.getFootstep(i - 1);

         if(foothold.isEmpty() && footPolygons != null)
            footstep.setFoothold(footPolygons.get(footstep.getRobotSide()));
         if (!foothold.isEmpty())
            footstep.setFoothold(foothold);
      }

      plan.setLowLevelPlanGoal(goalPoseInWorld);

      return plan;
   }

   @Override
   public double getPlanningDuration()
   {
      return planningTime.getDoubleValue();
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizon)
   {
   }

   private boolean initialize()
   {
      if (debug)
      {
         LogTools.info("Initializing AStarFootstepPlanner. startNodes = " + startNode + ", goalNodes = " + goalNodes);
      }
      if (startNode == null)
         throw new NullPointerException("Need to set initial conditions before planning.");
      if (goalNodes == null)
         throw new NullPointerException("Need to set goal before planning.");

      abortPlanning.set(false);

      graph.initialize(startNode);
      NodeComparator nodeComparator = new NodeComparator(graph, goalNodes, heuristics);
      stack = new PriorityQueue<>(nodeComparator);

      validGoalNode.set(true);
      for (RobotSide robotSide : RobotSide.values)
      {
         boolean validGoalNode = nodeChecker.isNodeValid(goalNodes.get(robotSide), null);

         if (!validGoalNode)
         {
            if (debug)
            {
               LogTools.info("GoalNode is not valid: " + goalNodes.get(robotSide));
            }
         }

         if (!validGoalNode && !parameters.getReturnBestEffortPlan())
         {
            if (debug)
               LogTools.info("Goal node isn't valid. To plan without a valid goal node, best effort planning must be enabled");

            return false;
         }

         this.validGoalNode.set(validGoalNode && this.validGoalNode.getBooleanValue());
      }

      stack.add(startNode);
      expandedNodes = new HashSet<>();
      endNode = null;

      if (listener != null)
      {
         listener.addNode(startNode, null);
         listener.tickAndUpdate();
      }

      return true;
   }

   @Override
   public void cancelPlanning()
   {
      if (debug)
         LogTools.info("Cancel has been requested.");
      abortPlanning.set(true);
   }

   public void requestInitialize()
   {
      initialize.set(true);
   }

   private boolean planInternal()
   {
      long planningStartTime = System.nanoTime();

      long rejectedNodesCount = 0;
      long expandedNodesCount = 0;
      long iterations = 0;

      while (!stack.isEmpty())
      {
         if (initialize.getBooleanValue())
         {
            boolean success = initialize();
            rejectedNodesCount = 0;
            expandedNodesCount = 0;
            iterations = 0;
            initialize.set(false);
            if (!success)
               return false;
         }

         iterations++;

         FootstepNode nodeToExpand = stack.poll();
         if (expandedNodes.contains(nodeToExpand))
            continue;
         expandedNodes.add(nodeToExpand);

         if (checkAndHandleNodeAtGoal(nodeToExpand))
            break;

         checkAndHandleBestEffortNode(nodeToExpand);

         HashSet<FootstepNode> neighbors = nodeExpansion.expandNode(nodeToExpand);
         expandedNodesCount += neighbors.size();
         for (FootstepNode neighbor : neighbors)
         {
            if (listener != null)
               listener.addNode(neighbor, nodeToExpand);

            // Checks if the footstep (center of the foot) is on a planar region
            if (!nodeChecker.isNodeValid(neighbor, nodeToExpand))
            {
               rejectedNodesCount++;
               continue;
            }

            double cost = stepCostCalculator.compute(nodeToExpand, neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);

            if (!parameters.getReturnBestEffortPlan() || endNode == null || stack.comparator().compare(neighbor, endNode) < 0)
               stack.add(neighbor);
         }

         if (listener != null)
            listener.tickAndUpdate();

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime) > timeout.getDoubleValue() || abortPlanning.getBooleanValue())
         {
            if (abortPlanning.getBooleanValue())
               LogTools.info("Abort planning requested.");
            abortPlanning.set(false);
            break;
         }
      }

      long timeInNano = System.nanoTime();
      planningTime.set(Conversions.nanosecondsToSeconds(timeInNano - planningStartTime));
      percentRejectedNodes.set(100.0 * rejectedNodesCount / expandedNodesCount);
      itarationCount.set(iterations);
      numberOfExpandedNodes.set(expandedNodesCount / Math.max(iterations, 1));

      return true;
   }

   private boolean checkAndHandleNodeAtGoal(FootstepNode nodeToExpand)
   {
      if (!validGoalNode.getBooleanValue())
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
      if (!parameters.getReturnBestEffortPlan())
         return;

      if (graph.getPathFromStart(nodeToExpand).size() - 1 < parameters.getMinimumStepsForBestEffortPlan())
         return;

      if (endNode == null || heuristics.compute(nodeToExpand, goalNodes.get(nodeToExpand.getRobotSide())) < heuristics
            .compute(endNode, goalNodes.get(endNode.getRobotSide())))
      {
         if (listener != null)
            listener.reportLowestCostNodeList(graph.getPathFromStart(nodeToExpand));
         endNode = nodeToExpand;
      }
   }

   private FootstepPlanningResult checkResult()
   {
      if (stack.isEmpty() && endNode == null)
         return FootstepPlanningResult.NO_PATH_EXISTS;
      if (!graph.doesNodeExist(endNode))
         return FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION;

      if (heuristics.getWeight() <= 1.0)
         return FootstepPlanningResult.OPTIMAL_SOLUTION;

      return FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;
   }

   public static void checkGoalType(FootstepPlannerGoal goal)
   {
      FootstepPlannerGoalType supportedGoalType1 = FootstepPlannerGoalType.POSE_BETWEEN_FEET;
      FootstepPlannerGoalType supportedGoalType2 = FootstepPlannerGoalType.DOUBLE_FOOTSTEP;
      if (!goal.getFootstepPlannerGoalType().equals(supportedGoalType1) && !goal.getFootstepPlannerGoalType().equals(supportedGoalType2))
         throw new IllegalArgumentException("Planner does not support goals other than " + supportedGoalType1 + " and " + supportedGoalType2);
   }

   public static AStarFootstepPlanner createPlanner(FootstepPlannerParameters parameters, BipedalFootstepPlannerListener listener,
                                                    SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeExpansion expansion,
                                                    YoVariableRegistry registry)
   {
      return createPlanner(parameters, listener, footPolygons, expansion, null, registry);
   }

   public static AStarFootstepPlanner createPlanner(FootstepPlannerParameters parameters, BipedalFootstepPlannerListener listener,
                                                    SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeExpansion expansion,
                                                    HeuristicSearchAndActionPolicyDefinitions policyDefinitions, YoVariableRegistry registry)
   {
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeSnapAndWiggler postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      FootstepNodeBodyCollisionDetector collisionDetector = new FootstepNodeBodyCollisionDetector(parameters);

      SnapBasedNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      BodyCollisionNodeChecker bodyCollisionNodeChecker = new BodyCollisionNodeChecker(collisionDetector, parameters, snapper);
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(parameters, snapper, footPolygons);

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(parameters.getCostParameters().getAStarHeuristicsWeight(), parameters);

      FootstepNodeChecker nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, bodyCollisionNodeChecker, cliffAvoider));
      nodeChecker.addPlannerListener(listener);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludeBoundingBoxCost(true);
      costBuilder.setIncludePitchAndRollCost(true);
      costBuilder.setCollisionDetector(collisionDetector);

      FootstepCost footstepCost = costBuilder.buildCost();

      AStarFootstepPlanner planner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, footstepCost, postProcessingSnapper, listener,
                                                              footPolygons, registry);

      if (policyDefinitions != null)
      {
         policyDefinitions.setCollisionNodeChecker(bodyCollisionNodeChecker);
         policyDefinitions.setNodeSnapper(snapper);
         policyDefinitions.build();

         policyDefinitions.getPlannerListeners().forEach(nodeChecker::addPlannerListener);
         policyDefinitions.getStartAndGoalListeners().forEach(planner::addStartAndGoalListener);
      }

      return planner;
   }
}
