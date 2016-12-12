package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.printing.PrintTools;

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

   private final ArrayList<BipedalFootstepPlannerNode> goalNodes = new ArrayList<>();
   private final BooleanYoVariable exitAfterInitialSolution = new BooleanYoVariable("exitAfterInitialSolution", registry);
   protected BipedalFootstepPlannerNode startNode, goalNode;
   protected FootstepPlan footstepPlan = null;

   protected BipedalFootstepPlannerListener listener;

   public PlanarRegionBipedalFootstepPlanner(BipedalFootstepPlannerParameters parameters, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      planarRegionPotentialNextStepCalculator = new PlanarRegionPotentialNextStepCalculator(parameters, parentRegistry);
      exitAfterInitialSolution.set(true);
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

   public void setExitAfterInitialSolution(boolean exitAfterInitialSolution)
   {
      this.exitAfterInitialSolution.set(exitAfterInitialSolution);
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      planarRegionPotentialNextStepCalculator.setFeetPolygons(footPolygonsInSoleFrame);
   }

   public SideDependentList<ConvexPolygon2d> getFootPolygonsInSoleFrame()
   {
      return footPolygonsInSoleFrame;
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide initialSide)
   {
      stanceFootPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      this.initialSide = initialSide;
      stanceFootPose.getPose(initialFootPose);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      planarRegionPotentialNextStepCalculator.setGoal(goal);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      planarRegionPotentialNextStepCalculator.setPlanarRegions(planarRegionsList);
   }

   @Override
   public FootstepPlan getPlan()
   {
      if (goalNode == null)
         return null;

      return new FootstepPlan(goalNode);
   }

   protected void initialize()
   {
      stack.clear();
      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      stack.push(startNode);
      mapToAllExploredNodes.clear();
   }

   @Override
   public FootstepPlanningResult plan()
   {
      initialize();
      goalNode = null;
      goalNodes.clear();
      footstepPlan = null;
      double smallestCostToGoal = Double.POSITIVE_INFINITY;
      planarRegionPotentialNextStepCalculator.setStartNode(startNode);
      numberOfNodesExpanded.set(0);

      while (!stack.isEmpty()) // && numberOfNodesExpanded.getIntegerValue() < 1000)
      {
         BipedalFootstepPlannerNode nodeToExpand = stack.pop();
         double costToNode = nodeToExpand.getCostToHereFromStart();

         if (costToNode > smallestCostToGoal)
            continue;

         // if we already found this node make sure we update its parent in case we found a better path here.
         BipedalFootstepPlannerNode equivalentNode = checkIfNearbyNodeAlreadyExistsAndStoreIfNot(nodeToExpand);
         if (equivalentNode != null)
         {
            if (costToNode < equivalentNode.getCostToHereFromStart())
               equivalentNode.setParentNode(nodeToExpand.getParentNode());
            continue;
         }

         numberOfNodesExpanded.increment();
         notifyListenerNodeIsBeingExpanded(nodeToExpand);

         if (nodeToExpand.isAtGoal())
         {
            goalNodes.add(nodeToExpand);

            if (exitAfterInitialSolution.getBooleanValue())
            {
               updateGoalPath(smallestCostToGoal);
               return FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;
            }
         }
         else
            expandChildrenAndAddNodes(stack, nodeToExpand, smallestCostToGoal);

         if (numberOfNodesExpanded.getIntegerValue() % 500 == 0)
            smallestCostToGoal = updateGoalPath(smallestCostToGoal);

         if (numberOfNodesExpanded.getIntegerValue() > maximumNumberOfNodesToExpand.getIntegerValue())
            break;
      }

      if (goalNodes.isEmpty())
      {
         notifyListenerSolutionWasNotFound();
         return FootstepPlanningResult.NO_PATH_EXISTS;
      }

      updateGoalPath(smallestCostToGoal);
      return FootstepPlanningResult.OPTIMAL_SOLUTION;
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
            double cost1 = o1.getCostToHereFromStart();
            double cost2 = o2.getCostToHereFromStart();
            return cost1 > cost2 ? 1 : -1;
         }
      });

      goalNode = goalNodes.get(0);
      double costToGoal = goalNode.getCostToHereFromStart();
      if (costToGoal < smallestCostToGoal)
      {
         PrintTools.info("Reduced cost to goal: " + costToGoal);
         smallestCostToGoal = costToGoal;

         Iterator<BipedalFootstepPlannerNode> iterator = stack.iterator();
         while (iterator.hasNext())
         {
            BipedalFootstepPlannerNode node = iterator.next();
            if (node.getCostToHereFromStart() > smallestCostToGoal)
               stack.remove(node);
         }
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
}
