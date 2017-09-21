package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.*;

public class PlanarRegionBipedalFootstepPlanner implements FootstepPlanner
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();

   private final BipedalFootstepPlannerParameters parameters;
   private final PlanarRegionPotentialNextStepCalculator planarRegionPotentialNextStepCalculator;
   private final HashSet<BipedalFootstepPlannerNode> expandedNodes = new HashSet<>();

   private SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private BipedalFootstepPlannerNode startNode;

   private final YoInteger maximumNumberOfNodesToExpand = new YoInteger("maximumNumberOfNodesToExpand", registry);
   private final YoInteger numberOfNodesExpanded = new YoInteger("numberOfNodesExpanded", registry);
   private final YoDouble timeout = new YoDouble("Timeout", registry);
   private final YoLong planningStartTime = new YoLong("planningStartTime", registry);

   private final ArrayList<BipedalFootstepPlannerNode> goalNodes = new ArrayList<>();
   private final YoBoolean exitAfterInitialSolution = new YoBoolean("exitAfterInitialSolution", registry);
   private BipedalFootstepPlannerNode bestGoalNode;
   private PlanarRegionsList planarRegionsList;

   protected BipedalFootstepPlannerListener listener;

   public PlanarRegionBipedalFootstepPlanner(BipedalFootstepPlannerParameters parameters, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.parameters = parameters;

      planarRegionPotentialNextStepCalculator = new PlanarRegionPotentialNextStepCalculator(parameters, parentRegistry, null);
      exitAfterInitialSolution.set(true);
      timeout.set(Double.POSITIVE_INFINITY);
   }

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
      planarRegionPotentialNextStepCalculator.setBipedalFootstepPlannerListener(listener);
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
      planarRegionPotentialNextStepCalculator.setFeetPolygons(footPolygonsInSoleFrame, footPolygonsInSoleFrame);
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, SideDependentList<ConvexPolygon2D> controllerPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      planarRegionPotentialNextStepCalculator.setFeetPolygons(footPolygonsInSoleFrame, controllerPolygonsInSoleFrame);
   }

   public SideDependentList<ConvexPolygon2D> getFootPolygonsInSoleFrame()
   {
      return footPolygonsInSoleFrame;
   }

   protected boolean initialStanceFootWasSet = false;
   protected boolean goalWasSet = false;

   @Override
   public final void setInitialStanceFoot(FramePose stanceFootPose, RobotSide initialSide)
   {
      stanceFootPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      startNode = new BipedalFootstepPlannerNode(stanceFootPose.getX(), stanceFootPose.getY(), stanceFootPose.getYaw(), initialSide);
      initialStanceFootWasSet = true;
   }

   @Override
   public final void setGoal(FootstepPlannerGoal goal)
   {
      planarRegionPotentialNextStepCalculator.setGoal(goal);
      goalWasSet = true;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      planarRegionPotentialNextStepCalculator.setPlanarRegions(planarRegionsList);
   }

   @Override
   public FootstepPlan getPlan()
   {
      if (bestGoalNode == null)
         return null;

      return BipedalFootstepPlannerNodeUtils.createFootstepPlanFromEndNode(bestGoalNode, planarRegionsList, parameters, footPolygonsInSoleFrame);
   }

   protected void initialize()
   {
      stack.clear();
      notifiyListenersStartNodeWasAdded(startNode);
      stack.push(startNode);
      expandedNodes.clear();
   }

   @Override
   public FootstepPlanningResult plan()
   {
      bestGoalNode = null;
      goalNodes.clear();
      double smallestCostToGoal = Double.POSITIVE_INFINITY;

      if (!initialStanceFootWasSet || !goalWasSet)
      {
         return FootstepPlanningResult.NO_PATH_EXISTS;
      }

      initialize();
      planarRegionPotentialNextStepCalculator.setStartNode(startNode);
      numberOfNodesExpanded.set(0);
      planningStartTime.set(System.nanoTime());

      while (!stack.isEmpty())
      {
         BipedalFootstepPlannerNode nodeToExpand;
         // find a path to the goal fast using depth first then refine using breath first
         if (bestGoalNode == null)
            nodeToExpand = stack.pop();
         else
            nodeToExpand = stack.pollLast();

         // if going to the node is more expensive then going to the goal there is no point in expanding it.
         double costToNode = BipedalFootstepPlannerNodeUtils.getCostFromStartToNode(nodeToExpand);
         if (costToNode > smallestCostToGoal)
            continue;

         // if we already found this node make sure we update its parent in case we found a better path here.
         if (expandedNodes.contains(nodeToExpand))
            continue;
         expandedNodes.add(nodeToExpand);

         numberOfNodesExpanded.increment();
         notifyListenerNodeIsBeingExpanded(nodeToExpand);

         if (nodeToExpand.isAtGoal())
         {
            goalNodes.add(nodeToExpand);
            smallestCostToGoal = updateGoalPath(smallestCostToGoal);
            if (exitAfterInitialSolution.getBooleanValue())
               break;
         }
         else
            expandChildrenAndAddNodes(stack, nodeToExpand, smallestCostToGoal);

         // keep checking if the goal cost decreased from time to time
         if (numberOfNodesExpanded.getIntegerValue() % 500 == 0)
            smallestCostToGoal = updateGoalPath(smallestCostToGoal);

         if (numberOfNodesExpanded.getIntegerValue() > maximumNumberOfNodesToExpand.getIntegerValue())
            break;

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime.getLongValue()) > timeout.getDoubleValue())
            break;
      }

      if (goalNodes.isEmpty())
      {
         notifyListenerSolutionWasNotFound();
         return FootstepPlanningResult.NO_PATH_EXISTS;
      }

      updateGoalPath(smallestCostToGoal);

      if (stack.isEmpty())
         return FootstepPlanningResult.OPTIMAL_SOLUTION;
      return FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;
   }

   private double updateGoalPath(double smallestCostToGoal)
   {
      if (goalNodes.isEmpty())
         return Double.POSITIVE_INFINITY;

      Collections.sort(goalNodes, new Comparator<BipedalFootstepPlannerNode>()
      {
         @Override
         public int compare(BipedalFootstepPlannerNode o1, BipedalFootstepPlannerNode o2)
         {
            double cost1 = BipedalFootstepPlannerNodeUtils.getCostFromStartToNode(o1);
            double cost2 = BipedalFootstepPlannerNodeUtils.getCostFromStartToNode(o2);
            if(cost1 == cost2) return 0;
            return cost1 < cost2 ? -1 : 1;
         }
      });

      bestGoalNode = goalNodes.get(0);
      double costToGoal = BipedalFootstepPlannerNodeUtils.getCostFromStartToNode(bestGoalNode);
      if (costToGoal < smallestCostToGoal)
      {
         PrintTools.info("Reduced cost to goal: " + costToGoal);
         smallestCostToGoal = costToGoal;
      }

      return smallestCostToGoal;
   }

   protected void expandChildrenAndAddNodes(Deque<BipedalFootstepPlannerNode> stack, BipedalFootstepPlannerNode nodeToExpand, double smallestCostToGoal)
   {
      ArrayList<BipedalFootstepPlannerNode> nodesToAddFromWorstToBest = planarRegionPotentialNextStepCalculator.computeChildrenNodes(nodeToExpand, smallestCostToGoal);

      for (BipedalFootstepPlannerNode node : nodesToAddFromWorstToBest)
      {
         if (BipedalFootstepPlannerNodeUtils.getCostFromStartToNode(node) > smallestCostToGoal)
            continue;

         stack.push(node);
      }
   }

   protected void notifyListenerSolutionWasFound(FootstepPlan footstepPlan)
   {
      if (listener != null)
      {
         listener.solutionWasFound(footstepPlan);
      }
   }

   protected void notifyListenerSolutionWasNotFound()
   {
      if (listener != null)
      {
         listener.solutionWasNotFound();
      }
   }

   protected void notifyListenerNodeIsBeingExpanded(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeIsBeingExpanded(nodeToExpand);
      }
   }

   protected void notifiyListenersStartNodeWasAdded(BipedalFootstepPlannerNode startNode)
   {
      if (listener != null)
      {
         listener.startNodeWasAdded(startNode);
      }
   }
}
