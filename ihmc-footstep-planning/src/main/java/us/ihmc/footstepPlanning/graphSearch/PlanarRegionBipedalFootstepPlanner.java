package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.aStar.*;
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

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

public class PlanarRegionBipedalFootstepPlanner implements FootstepPlanner
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final BipedalFootstepPlannerParameters parameters;
   private final YoInteger maximumNumberOfNodesToExpand = new YoInteger("maximumNumberOfNodesToExpand", registry);
   private final YoDouble timeout = new YoDouble("Timeout", registry);
   private final YoBoolean exitAfterInitialSolution = new YoBoolean("exitAfterInitialSolution", registry);

   private final FootstepGraph footstepGraph;
   private final Deque<FootstepNode> stack = new ArrayDeque<FootstepNode>();
   private final FootstepNodeExpansion nodeExpansion;
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

   public PlanarRegionBipedalFootstepPlanner(BipedalFootstepPlannerParameters parameters, FootstepNodeExpansion nodeExpansion, FootstepNodeSnapper snapper,
                                             FootstepNodeChecker checker, FootstepCost stepCostCalculator, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.parameters = parameters;
      exitAfterInitialSolution.set(true);
      timeout.set(Double.POSITIVE_INFINITY);
      this.nodeExpansion = nodeExpansion;
      this.footstepGraph = new FootstepGraph();
      this.snapper = snapper;
      this.checker = checker;
      this.stepCostCalculator = stepCostCalculator;
   }

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
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
      for (int i = 1; i < path.size(); i++)
      {
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
      stack.push(startNode);
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

         FootstepNode nodeToExpand;
         // find a path to the goal fast using depth first then refine using breath first
         if (bestGoalNode == null)
            nodeToExpand = stack.pop();
         else
            nodeToExpand = stack.pollLast();

         notifyListenerNodeIsBeingExpanded(nodeToExpand);

         if (goalNodes.get(nodeToExpand.getRobotSide()).equals(nodeToExpand))
         {
            bestGoalNode = goalNodes.get(nodeToExpand.getRobotSide());
            if (exitAfterInitialSolution.getBooleanValue())
               break;
            else
               continue;
         }

         ArrayList<FootstepNode> childNodes = new ArrayList<>();
         childNodes.addAll(nodeExpansion.expandNode(nodeToExpand));
         childNodes.sort((node1, node2) ->
                         {
                            double goalDistance1 = node1.euclideanDistance(goalNodes.get(node1.getRobotSide()));
                            double goalDistance2 = node2.euclideanDistance(goalNodes.get(node2.getRobotSide()));
                            return goalDistance1 < goalDistance2 ? 1 : -1;
                         });

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
               stack.addFirst(childNode);

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
