package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanarRegionBipedalFootstepPlanner implements FootstepPlanner
{
   protected final Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();

   protected final PlanarRegionPotentialNextStepCalculator planarRegionPotentialNextStepCalculator;
   protected final HashMap<Integer, List<BipedalFootstepPlannerNode>> mapToAllExploredNodes = new HashMap<>();

   protected SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame;

   protected RobotSide initialSide;
   protected RigidBodyTransform initialFootPose = new RigidBodyTransform();

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   protected final IntegerYoVariable maximumNumberOfNodesToExpand = new IntegerYoVariable("maximumNumberOfNodesToExpand", registry);
   protected final IntegerYoVariable numberOfNodesExpanded = new IntegerYoVariable("numberOfNodesExpanded", registry);
   protected final DoubleYoVariable timeout = new DoubleYoVariable("Timeout", registry);
   protected final LongYoVariable planningStartTime = new LongYoVariable("planningStartTime", registry);

   protected final ArrayList<BipedalFootstepPlannerNode> goalNodes = new ArrayList<>();
   protected final BooleanYoVariable exitAfterInitialSolution = new BooleanYoVariable("exitAfterInitialSolution", registry);
   protected BipedalFootstepPlannerNode startNode, bestGoalNode;
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

   public void setFeetPolygons(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      planarRegionPotentialNextStepCalculator.setFeetPolygons(footPolygonsInSoleFrame, footPolygonsInSoleFrame);
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame, SideDependentList<ConvexPolygon2d> controllerPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      planarRegionPotentialNextStepCalculator.setFeetPolygons(footPolygonsInSoleFrame, controllerPolygonsInSoleFrame);
   }

   public SideDependentList<ConvexPolygon2d> getFootPolygonsInSoleFrame()
   {
      return footPolygonsInSoleFrame;
   }

   protected boolean initialStanceFootWasSet = false;
   protected boolean goalWasSet = false;

   @Override
   public final void setInitialStanceFoot(FramePose stanceFootPose, RobotSide initialSide)
   {
      stanceFootPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      this.initialSide = initialSide;
      stanceFootPose.getPose(initialFootPose);
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
      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
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
         double costToNode = nodeToExpand.getCostToHereFromStart();
         if (costToNode > smallestCostToGoal)
            continue;

         // if we already found this node make sure we update its parent in case we found a better path here.
         BipedalFootstepPlannerNode equivalentNode = checkIfNearbyNodeAlreadyExistsAndStoreIfNot(nodeToExpand);
         if (equivalentNode != null)
         {
            double costToGoToEquivalentNode = equivalentNode.getCostToHereFromStart();

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
            double cost1 = o1.getCostToHereFromStart();
            double cost2 = o2.getCostToHereFromStart();
            if(cost1 == cost2) return 0;
            return cost1 < cost2 ? -1 : 1;
         }
      });

      bestGoalNode = goalNodes.get(0);
      double costToGoal = bestGoalNode.getCostToHereFromStart();
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
         if (node.getCostToHereFromStart() > smallestCostToGoal)
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
