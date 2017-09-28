package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.aStar.*;
import us.ihmc.footstepPlanning.aStar.implementations.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.aStar.implementations.PlanarRegionBipedalFootstepNodeExpansion;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

public class PlanarRegionBipedalFootstepPlanner implements FootstepPlanner
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final BipedalFootstepPlannerParameters parameters;
   private final YoInteger maximumNumberOfNodesToExpand = new YoInteger("maximumNumberOfNodesToExpand", registry);
   private final YoDouble timeout = new YoDouble("Timeout", registry);
   private final YoBoolean exitAfterInitialSolution = new YoBoolean("exitAfterInitialSolution", registry);

   private final FootstepGraph footstepGraph;
   private final PriorityQueue<FootstepNode> stack;
   private final PlanarRegionBipedalFootstepNodeExpansion nodeExpansion;
   private final FootstepNodeSnapper snapper;
   private final FootstepNodeChecker checker;
   private final FootstepCost stepCostCalculator;
   private SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private FootstepNode startNode;
   private final SideDependentList<FootstepNode> goalNodes = new SideDependentList<>();
   private FootstepNode bestGoalNode;

   private BipedalFootstepPlannerListener listener;
   private final YoInteger numberOfNodesExpanded = new YoInteger("numberOfNodesExpanded", registry);
   private final YoLong planningStartTime = new YoLong("planningStartTime", registry);

   public PlanarRegionBipedalFootstepPlanner(BipedalFootstepPlannerParameters parameters, FootstepNodeSnapper snapper,
                                             FootstepNodeChecker checker, FootstepCost stepCostCalculator, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.parameters = parameters;
      this.footstepGraph = new FootstepGraph();
      this.snapper = snapper;
      this.checker = checker;
      this.stepCostCalculator = stepCostCalculator;
      nodeExpansion = new PlanarRegionBipedalFootstepNodeExpansion(parameters);

      DistanceAndYawBasedHeuristics costToGoHeuristics = new DistanceAndYawBasedHeuristics(0.5, registry);
      Comparator<FootstepNode> nodeComparator = (node1, node2) ->
      {
         double cost1 = costToGoHeuristics.compute(node1, goalNodes.get(node1.getRobotSide()));
         double cost2 = costToGoHeuristics.compute(node2, goalNodes.get(node2.getRobotSide()));
         if (cost1 == cost2) return 0;
         return cost1 < cost2 ? -1 : 1;
      };
      stack = new PriorityQueue<>(nodeComparator);

      exitAfterInitialSolution.set(true);
      timeout.set(Double.POSITIVE_INFINITY);
   }

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
      ((BipedalFootstepPlannerNodeChecker) checker).setBipedalFootstepPlannerListener(listener);
   }

   public void setMaximumNumberOfNodesToExpand(int maximumNumberOfNodesToExpand)
   {
      this.maximumNumberOfNodesToExpand.set(maximumNumberOfNodesToExpand);
   }

   public void setTimeout(double timeoutInSeconds)
   {
      this.timeout.set(timeoutInSeconds);
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
   public final void setInitialStanceFoot(FramePose stanceFootPose, RobotSide initialSide)
   {
      stanceFootPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      startNode = new FootstepNode(stanceFootPose.getX(), stanceFootPose.getY(), stanceFootPose.getYaw(), initialSide);
   }

   @Override
   public final void setGoal(FootstepPlannerGoal goal)
   {
      checkGoalType(goal);
      FramePose goalPose = goal.getGoalPoseBetweenFeet();
      ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);

      for (RobotSide side : RobotSide.values)
      {
         FramePose goalNodePose = new FramePose(goalFrame);
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
      if(listener != null)
         listener.planarRegionsListSet(planarRegionsList);
   }

   @Override
   public FootstepPlan getPlan()
   {
      if (!footstepGraph.doesNodeExist(bestGoalNode))
         return null;

      FootstepPlan plan = new FootstepPlan();
      List<FootstepNode> path = footstepGraph.getPathFromStart(bestGoalNode);
      for (int i = 1; i < path.size(); i++)
      {
         System.out.println(path.get(i).getYaw());
         RobotSide robotSide = path.get(i).getRobotSide();
         RigidBodyTransform snapTransform = snapper.snapFootstepNode(path.get(i), null);

         RigidBodyTransform footstepPose = new RigidBodyTransform();
         footstepPose.setRotationYawAndZeroTranslation(path.get(i).getYaw());
         footstepPose.setTranslationX(path.get(i).getX());
         footstepPose.setTranslationY(path.get(i).getY());
         snapTransform.transform(footstepPose);

         plan.addFootstep(robotSide, new FramePose(ReferenceFrame.getWorldFrame(), footstepPose));
      }

      return plan;
   }

   private void initialize()
   {
      stack.clear();
      stack.add(startNode);
      footstepGraph.initialize(startNode);
      notifiyListenersStartNodeWasAdded(startNode);

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
         FootstepNode nodeToExpand = stack.poll();
         notifyListenerNodeIsBeingExpanded(nodeToExpand);

         if (goalNodes.get(nodeToExpand.getRobotSide()).equals(nodeToExpand))
         {
            bestGoalNode = goalNodes.get(nodeToExpand.getRobotSide());
            if (exitAfterInitialSolution.getBooleanValue())
               break;
            else
               continue;
         }

         HashSet<FootstepNode> childNodes = nodeExpansion.expandNode(nodeToExpand);
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
               stack.add(childNode);

            footstepGraph.checkAndSetEdge(nodeToExpand, childNode, stepCost);
         }

         if (numberOfNodesExpanded.getIntegerValue() > maximumNumberOfNodesToExpand.getIntegerValue())
            break;

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime.getLongValue()) > timeout.getDoubleValue())
            break;
      }

      if (bestGoalNode == null)
      {
         notifyListenerSolutionWasNotFound();
         return FootstepPlanningResult.NO_PATH_EXISTS;
      }

      if (stack.isEmpty())
         return FootstepPlanningResult.OPTIMAL_SOLUTION;
      return FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;
   }

   private void notifyListenerSolutionWasFound(FootstepPlan footstepPlan)
   {
      if (listener != null)
      {
         listener.solutionWasFound(footstepPlan);
      }
   }

   private void notifyListenerSolutionWasNotFound()
   {
      if (listener != null)
      {
         listener.solutionWasNotFound();
      }
   }

   private void notifyListenerNodeIsBeingExpanded(FootstepNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeIsBeingExpanded(nodeToExpand);
      }
   }

   private void notifiyListenersStartNodeWasAdded(FootstepNode startNode)
   {
      if (listener != null)
      {
         listener.startNodeWasAdded(startNode);
      }
   }
}
