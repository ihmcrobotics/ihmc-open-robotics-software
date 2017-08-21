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
   protected final Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();

   protected final PlanarRegionPotentialNextStepCalculator planarRegionPotentialNextStepCalculator;
   protected final HashMap<Integer, List<BipedalFootstepPlannerNode>> mapToAllExploredNodes = new HashMap<>();

   protected SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;

   protected RobotSide initialSide;
   protected BipedalFootstepPlannerNode startNode;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   protected final YoInteger maximumNumberOfNodesToExpand = new YoInteger("maximumNumberOfNodesToExpand", registry);
   protected final YoInteger numberOfNodesExpanded = new YoInteger("numberOfNodesExpanded", registry);
   protected final YoDouble timeout = new YoDouble("Timeout", registry);
   protected final YoLong planningStartTime = new YoLong("planningStartTime", registry);

   protected final ArrayList<BipedalFootstepPlannerNode> goalNodes = new ArrayList<>();
   protected final YoBoolean exitAfterInitialSolution = new YoBoolean("exitAfterInitialSolution", registry);
   protected BipedalFootstepPlannerNode bestGoalNode;
   protected FootstepPlan footstepPlan = null;

   protected BipedalFootstepPlannerListener listener;

   public PlanarRegionBipedalFootstepPlanner(BipedalFootstepPlannerParameters parameters, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

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
      planarRegionPotentialNextStepCalculator.setPlanarRegions(planarRegionsList);
   }

   @Override
   public FootstepPlan getPlan()
   {
      if (bestGoalNode == null)
         return null;

      return new FootstepPlan(bestGoalNode);
   }

   protected void initialize()
   {
      stack.clear();
      notifiyListenersStartNodeWasAdded(startNode);
      stack.push(startNode);
      mapToAllExploredNodes.clear();
   }

   @Override
   public FootstepPlanningResult plan()
   {
      bestGoalNode = null;
      goalNodes.clear();
      footstepPlan = null;
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
         BipedalFootstepPlannerNode equivalentNode = checkIfNearbyNodeAlreadyExistsAndStoreIfNot(nodeToExpand);
         if (equivalentNode != null)
         {
            double costToGoToEquivalentNode = BipedalFootstepPlannerNodeUtils.getCostFromStartToNode(equivalentNode);

            if (MathTools.epsilonEquals(costToNode, costToGoToEquivalentNode, 1.0e-5))
               nodeToExpand = equivalentNode;
            else if (costToNode > costToGoToEquivalentNode)
               continue;
            else
            {
               equivalentNode.setParentNode(nodeToExpand.getParentNode());
               nodeToExpand = equivalentNode;
            }
         }

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

   protected double updateGoalPath(double smallestCostToGoal)
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

   /**
    * Will return null if no near node exists. Otherwise will return the nearest node.
    * @param nodeToExpand
    * @return
    */
   protected BipedalFootstepPlannerNode checkIfNearbyNodeAlreadyExistsAndStoreIfNot(BipedalFootstepPlannerNode nodeToExpand)
   {
      int hashCode = nodeToExpand.hashCode();

      List<BipedalFootstepPlannerNode> nodesWithThisHash = mapToAllExploredNodes.get(hashCode);

      if (nodesWithThisHash == null)
      {
         nodesWithThisHash = new ArrayList<>();
         nodesWithThisHash.add(nodeToExpand);
         mapToAllExploredNodes.put(hashCode, nodesWithThisHash);

         return null;
      }
      else
      {
         int size = nodesWithThisHash.size();
//         System.out.println(size);

         for (int i = 0; i < size; i++)
         {
            BipedalFootstepPlannerNode nodeWithSameHash = nodesWithThisHash.get(i);

            if (nodeToExpand.equals(nodeWithSameHash))
            {
               return nodeWithSameHash;
            }
         }

         return null;
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
