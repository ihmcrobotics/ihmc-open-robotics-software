package us.ihmc.footstepPlanning.graphSearch.planners;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashSet;
import java.util.List;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public class DepthFirstFootstepPlanner implements BodyPathAndFootstepPlanner
{
   private static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final FootstepPlannerParametersReadOnly parameters;
   private final YoInteger maximumNumberOfNodesToExpand = new YoInteger("maximumNumberOfNodesToExpand", registry);
   private final YoDouble timeout = new YoDouble("Timeout", registry);
   private final YoDouble planningDuration = new YoDouble("planningDuration", registry);
   private final YoBoolean exitAfterInitialSolution = new YoBoolean("exitAfterInitialSolution", registry);

   private final FootstepGraph footstepGraph;
   protected final Deque<FootstepNode> stack = new ArrayDeque<FootstepNode>();
   private final ParameterBasedNodeExpansion nodeExpansion;
   private final FootstepNodeSnapper snapper;
   private final FootstepNodeChecker checker;
   private final FootstepCost stepCostCalculator;
   private SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final Comparator<FootstepNode> nodeComparator;

   private FootstepNode startNode;
   private final SideDependentList<FootstepNode> goalNodes = new SideDependentList<>();
   private FootstepNode bestGoalNode;

   private final List<BipedalFootstepPlannerListener> listeners = new ArrayList<>();
   private final YoInteger numberOfNodesExpanded = new YoInteger("numberOfNodesExpanded", registry);
   private final YoLong planningStartTime = new YoLong("planningStartTime", registry);

   public DepthFirstFootstepPlanner(FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapper snapper, FootstepNodeChecker checker,
                                    FootstepCost stepCostCalculator, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.parameters = parameters;
      this.footstepGraph = new FootstepGraph();
      this.snapper = snapper;
      this.checker = checker;
      this.stepCostCalculator = stepCostCalculator;
      nodeExpansion = new ParameterBasedNodeExpansion(parameters);

      DistanceAndYawBasedHeuristics costToGoHeuristics = new DistanceAndYawBasedHeuristics(snapper, parameters.getDepthFirstHeuristicsWeight(),
                                                                                           parameters);
      this.nodeComparator = (node1, node2) -> {
         double cost1 = costToGoHeuristics.compute(node1, goalNodes.get(node1.getRobotSide()));
         double cost2 = costToGoHeuristics.compute(node2, goalNodes.get(node2.getRobotSide()));
         if (cost1 == cost2)
            return 0;
         return cost1 > cost2 ? -1 : 1;
      };

      exitAfterInitialSolution.set(true);
      timeout.set(Double.POSITIVE_INFINITY);
   }

   public void addBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      listeners.add(listener);
   }

   public void setMaximumNumberOfNodesToExpand(int maximumNumberOfNodesToExpand)
   {
      this.maximumNumberOfNodesToExpand.set(maximumNumberOfNodesToExpand);
   }

   @Override
   public void setTimeout(double timeoutInSeconds)
   {
      this.timeout.set(timeoutInSeconds);
   }

   @Override
   public double getPlanningDuration()
   {
      return planningDuration.getDoubleValue();
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizonLength)
   {
   }

   public void setExitAfterInitialSolution(boolean exitAfterInitialSolution)
   {
      this.exitAfterInitialSolution.set(exitAfterInitialSolution);
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, SideDependentList<ConvexPolygon2D> controllerPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
   }

   public SideDependentList<ConvexPolygon2D> getFootPolygonsInSoleFrame()
   {
      return footPolygonsInSoleFrame;
   }

   @Override
   public final void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide initialSide)
   {
      if (initialSide == null)
      {
         if (debug)
            PrintTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         initialSide = defaultStartNodeSide;
      }

      stanceFootPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      startNode = new FootstepNode(stanceFootPose.getX(), stanceFootPose.getY(), stanceFootPose.getYaw(), initialSide);
      RigidBodyTransform startNodeSnapTransform = FootstepNodeSnappingTools.computeSnapTransform(startNode, stanceFootPose);
      snapper.addSnapData(startNode, new FootstepNodeSnapData(startNodeSnapTransform));
      checker.addStartNode(startNode, startNodeSnapTransform);
   }

   @Override
   public final void setGoal(FootstepPlannerGoal goal)
   {
      checkGoalType(goal);
      FramePose3D goalPose = goal.getGoalPoseBetweenFeet();
      ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);

      for (RobotSide side : RobotSide.values)
      {
         FramePose3D goalNodePose = new FramePose3D(goalFrame);
         goalNodePose.setY(side.negateIfRightSide(parameters.getIdealFootstepWidth() / 2.0));
         goalNodePose.changeFrame(goalPose.getReferenceFrame());
         FootstepNode goalNode = new FootstepNode(goalNodePose.getX(), goalNodePose.getY(), goalNodePose.getYaw(), side);
         goalNodes.put(side, goalNode);
      }

      nodeExpansion.setGoalNodes(goalNodes);
   }

   private void checkGoalType(FootstepPlannerGoal goal)
   {
      FootstepPlannerGoalType supportedGoalType = FootstepPlannerGoalType.POSE_BETWEEN_FEET;
      if (!(goal.getFootstepPlannerGoalType() == supportedGoalType))
         throw new RuntimeException("Planner does not support goals other then " + supportedGoalType);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      checker.setPlanarRegions(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);
   }

   @Override
   public FootstepPlan getPlan()
   {
      if (!footstepGraph.doesNodeExist(bestGoalNode))
         return null;

      FootstepPlan plan = new FootstepPlan();
      List<FootstepNode> path = footstepGraph.getPathFromStart(bestGoalNode);
      path.add(goalNodes.get(bestGoalNode.getRobotSide().getOppositeSide()));

      for (int i = 1; i < path.size(); i++)
      {
         RobotSide robotSide = path.get(i).getRobotSide();

         RigidBodyTransform footstepPose = new RigidBodyTransform();
         footstepPose.setRotationYawAndZeroTranslation(path.get(i).getYaw());
         footstepPose.setTranslationX(path.get(i).getX());
         footstepPose.setTranslationY(path.get(i).getY());

         RigidBodyTransform snapTransform = snapper.snapFootstepNode(path.get(i)).getSnapTransform();
         if (!snapTransform.containsNaN())
            snapTransform.transform(footstepPose);

         plan.addFootstep(robotSide, new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPose));
      }

      return plan;
   }

   private void initialize()
   {
      stack.clear();
      stack.add(startNode);
      footstepGraph.initialize(startNode);

      numberOfNodesExpanded.set(0);
      planningStartTime.set(System.nanoTime());
      bestGoalNode = null;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      double smallestCostToGoal = Double.POSITIVE_INFINITY;
      initialize();

      while (!stack.isEmpty())
      {
         numberOfNodesExpanded.increment();

         FootstepNode nodeToExpand;
         if (bestGoalNode == null)
            nodeToExpand = stack.pollFirst();
         else
            nodeToExpand = stack.pollLast();

         if (goalNodes.get(nodeToExpand.getRobotSide()).equals(nodeToExpand))
         {
            bestGoalNode = goalNodes.get(nodeToExpand.getRobotSide());
            notifyListenerSolutionWasFound(nodeToExpand);
            if (exitAfterInitialSolution.getBooleanValue())
               break;
            else
               continue;
         }

         ArrayList<FootstepNode> childNodes = getSortedNodeList(nodeExpansion.expandNode(nodeToExpand));
         for (FootstepNode childNode : childNodes)
         {
            if (!checker.isNodeValid(childNode, nodeToExpand))
               continue;

            // if going to the node is more expensive then going to the goal there is no point in expanding it.
            double stepCost = stepCostCalculator.compute(nodeToExpand, childNode);
            if (stepCost + footstepGraph.getCostFromStart(nodeToExpand) > smallestCostToGoal)
               continue;

            // only add to stack if the node hasn't been explored
            if (!footstepGraph.doesNodeExist(childNode))
               stack.push(childNode);

            footstepGraph.checkAndSetEdge(nodeToExpand, childNode, stepCost);
         }

         if (numberOfNodesExpanded.getIntegerValue() > maximumNumberOfNodesToExpand.getIntegerValue())
            break;

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime.getLongValue()) > timeout.getDoubleValue())
         {
            planningDuration.set(-1.0);
            break;
         }
      }

      if (bestGoalNode == null)
      {
         planningDuration.set(-1.0);
         return FootstepPlanningResult.NO_PATH_EXISTS;
      }

      planningDuration.set(Conversions.nanosecondsToSeconds(System.nanoTime() - planningStartTime.getLongValue()));

      FootstepPlanningResult result;
      if (stack.isEmpty())
         result = FootstepPlanningResult.OPTIMAL_SOLUTION;
      else
         result = FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;

      if (debug)
      {
         PrintTools.info("Time taken = " + planningDuration.getDoubleValue());
         PrintTools.info("Result = " + result);
      }

      return result;
   }

   private ArrayList<FootstepNode> getSortedNodeList(HashSet<FootstepNode> nodeSet)
   {
      ArrayList<FootstepNode> nodeList = new ArrayList<>();
      nodeList.addAll(nodeSet);
      nodeList.sort(nodeComparator);
      return nodeList;
   }

   private void notifyListenerSolutionWasFound(FootstepNode endNode)
   {
      FootstepPlan plan = new FootstepPlan();
      List<FootstepNode> path = footstepGraph.getPathFromStart(bestGoalNode);
      path.add(goalNodes.get(bestGoalNode.getRobotSide().getOppositeSide()));

      for (int i = 1; i < path.size(); i++)
      {
         RobotSide robotSide = path.get(i).getRobotSide();

         RigidBodyTransform footstepPose = new RigidBodyTransform();
         footstepPose.setRotationYawAndZeroTranslation(path.get(i).getYaw());
         footstepPose.setTranslationX(path.get(i).getX());
         footstepPose.setTranslationY(path.get(i).getY());

         RigidBodyTransform snapTransform = snapper.snapFootstepNode(path.get(i)).getSnapTransform();
         if (!snapTransform.containsNaN())
            snapTransform.transform(footstepPose);

         plan.addFootstep(robotSide, new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPose));
      }
   }
}
