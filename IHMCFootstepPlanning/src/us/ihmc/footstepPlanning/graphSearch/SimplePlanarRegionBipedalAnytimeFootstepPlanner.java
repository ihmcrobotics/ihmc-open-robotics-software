package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.AnytimeFootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlanningUtils;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.thread.ThreadTools;

public class SimplePlanarRegionBipedalAnytimeFootstepPlanner extends PlanarRegionBipedalFootstepPlanner implements AnytimeFootstepPlanner, Runnable
{
   private final String namePrefix = "AnytimePlanner_";

   private BipedalFootstepPlannerNode closestNodeToGoal = null;
   private final AtomicReference<FootstepPlan> bestPlanYet = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlannerGoal> footstepPlannerGoalReference = new AtomicReference<>(null);
   private boolean stopRequested = false;
   private boolean isBestPlanYetOptimal = false;
   private final AtomicReference<PlanarRegionsList> planarRegionsListReference = new AtomicReference<>(null);
   private final AtomicReference<SimpleFootstep> latestExecutedFootstepReference = new AtomicReference<>(null);
   private final IntegerYoVariable stackSize = new IntegerYoVariable(namePrefix + "stackSize", registry);
   private final DoubleYoVariable smallestCostToGoal = new DoubleYoVariable(namePrefix + "SmallestCostToGoal", registry);
   private BipedalFootstepPlannerNode parentOfStartNode = null;

   private final FramePose tempFramePose = new FramePose();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   private boolean clear = false;

   public SimplePlanarRegionBipedalAnytimeFootstepPlanner(BipedalFootstepPlannerParameters parameters, YoVariableRegistry parentRegistry)
   {
      super(parameters, parentRegistry);
      smallestCostToGoal.set(Double.POSITIVE_INFINITY);
      exitAfterInitialSolution.set(false);
   }

   /**
    * @return The FootstepPlan that ends the closest to the goal of all explored yet
    */
   @Override
   public FootstepPlan getBestPlanYet()
   {
      return bestPlanYet.getAndSet(null);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      planarRegionsListReference.set(planarRegionsList);
   }

   @Override
   protected void initialize()
   {
      stack.clear();
      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      notifiyListenersStartNodeWasAdded(startNode);
      stack.push(startNode);
      closestNodeToGoal = null;
      mapToAllExploredNodes.clear();
      bestPlanYet.set(null);
      parentOfStartNode = null;
   }

   private void initializeForNewPlanarRegions()
   {
      stack.clear();
      stack.push(startNode);
      notifiyListenersStartNodeWasAdded(startNode);
      mapToAllExploredNodes.clear();

      planningStartTime.set(System.nanoTime());
      numberOfNodesExpanded.set(0);
      smallestCostToGoal.set(Double.POSITIVE_INFINITY);
      goalNodes.clear();
      bestGoalNode = null;

      closestNodeToGoal = null;
      bestPlanYet.set(null);
      //      trimBestPlanToNewPlanarRegions();
   }

   //   private void trimBestPlanToNewPlanarRegions()
   //   {
   //      List<BipedalFootstepPlannerNode> listOfNodes = FootstepPlanningUtils.createListOfNodesFromEndNode(closestNodeToGoal);
   //      PrintTools.info("trimming plan to new planar regions");
   //
   //      for (int i = 0; i < listOfNodes.size(); i++)
   //      {
   //         BipedalFootstepPlannerNode node = listOfNodes.get(i);
   //         boolean stillIsValid = planarRegionPotentialNextStepCalculator.snapNodeAndCheckIfAcceptableToExpand(node);
   //         PrintTools.info("trying to snap node with position " + node.getSolePosition());
   //
   //         if(!stillIsValid)
   //         {
   //            closestNodeToGoal = node.getParentNode();
   //            if(i < listOfNodes.size())
   //            {
   //               FootstepPlan trimmedBestPlan = FootstepPlanningUtils.createFootstepPlanFromEndNode(closestNodeToGoal);
   //               bestPlanYet.set(trimmedBestPlan);
   //            }
   //
   //            PrintTools.info("valid up to node " + i + " of " + listOfNodes.size());
   //            checkIfNearbyNodeAlreadyExistsAndStoreIfNot(node);
   //            break;
   //         }
   //         else
   //         {
   //            checkIfNearbyNodeAlreadyExistsAndStoreIfNot(node);
   //            PrintTools.info("valid node");
   //         }
   //      }
   //   }

   @Override
   public void executingFootstep(SimpleFootstep footstep)
   {
      latestExecutedFootstepReference.set(footstep);
   }

   private void replaceStartNode()
   {
      SimpleFootstep footstep = latestExecutedFootstepReference.getAndSet(null);
      if (footstep != null)
      {
         FramePose tempPose = new FramePose();
         initialSide = footstep.getRobotSide();
         footstep.getSoleFramePose(tempPose);
         tempPose.getRigidBodyTransform(initialFootPose);

         parentOfStartNode = startNode;
         startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
         notifiyListenersStartNodeWasAdded(startNode);
      }
   }

   private void replaceGoalPose()
   {
      FootstepPlannerGoal newGoal = footstepPlannerGoalReference.getAndSet(null);
      if (newGoal != null)
      {
         planarRegionPotentialNextStepCalculator.setGoal(newGoal);
      }
   }

   private void recursivelyMarkAsDead(BipedalFootstepPlannerNode node)
   {
      node.setToDead();
      ArrayList<BipedalFootstepPlannerNode> childrenNodes = new ArrayList<>();
      node.getChildren(childrenNodes);

      for (BipedalFootstepPlannerNode childNode : childrenNodes)
      {
         recursivelyMarkAsDead(childNode);
      }
   }

   @Override
   public boolean isBestPlanYetOptimal()
   {
      return isBestPlanYetOptimal;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      bestGoalNode = null;
      footstepPlan = null;

      while (!initialStanceFootWasSet || !goalWasSet)
      {
         if (exitAfterInitialSolution.getBooleanValue())
         {
            return FootstepPlanningResult.NO_PATH_EXISTS;
         }
         else
         {
            ThreadTools.sleep(100L);
         }
      }

      initialize();
      planarRegionPotentialNextStepCalculator.setStartNode(startNode);

      while (!stopRequested)
      {
         if (clear)
         {
            setPlanarRegions(new PlanarRegionsList());
            getBestPlanYet();
            clear = false;
         }

         stackSize.set(stack.size());

         replaceStartNode();
         replaceGoalPose();
         checkForNewPlanarRegionsList();

         if (stack.isEmpty())
         {
            ThreadTools.sleep(100);
            continue;
         }

         BipedalFootstepPlannerNode nodeToExpand;
         // find a path to the goal fast using depth first then refine using breath first
         if (bestGoalNode == null)
            nodeToExpand = stack.pop();
         else
            nodeToExpand = stack.pollLast();

         if (nodeToExpand.isDead())
            continue;

         // If stepping in place on first step, don't...
         if (parentOfStartNode != null)
         {
            if (parentOfStartNode.epsilonEquals(nodeToExpand, 0.04))
               continue;
         }

         if (nodeToExpand.getRobotSide() == null)
            continue;

         // if going to the node is more expensive then going to the goal there is no point in expanding it.
         double costToNode = nodeToExpand.getCostToHereFromStart();
         if (costToNode > smallestCostToGoal.getDoubleValue())
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
            smallestCostToGoal.set(updateGoalPath(smallestCostToGoal.getDoubleValue()));
            replaceBestPlan(bestGoalNode);
            if (exitAfterInitialSolution.getBooleanValue())
               break;
         }
         else
         {
            setNodesCostsAndRememberIfClosestYet(nodeToExpand);
            expandChildrenAndAddNodes(stack, nodeToExpand, smallestCostToGoal.getDoubleValue());
         }

         // keep checking if the goal cost decreased from time to time
         if (numberOfNodesExpanded.getIntegerValue() % 500 == 0)
         {
            smallestCostToGoal.set(updateGoalPath(smallestCostToGoal.getDoubleValue()));
            replaceBestPlan(closestNodeToGoal);
         }

         // some conditions where the stack is cleared
         if (maximumNumberOfNodesToExpand.getIntegerValue() > 0 && numberOfNodesExpanded.getIntegerValue() > maximumNumberOfNodesToExpand.getIntegerValue())
            stack.clear();

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime.getLongValue()) > timeout.getDoubleValue())
            stack.clear();
      }

      notifyListenerSolutionWasNotFound();
      return FootstepPlanningResult.NO_PATH_EXISTS;
   }

   private void checkForNewPlanarRegionsList()
   {
      PlanarRegionsList planarRegionsList = planarRegionsListReference.getAndSet(null);
      if (planarRegionsList != null)
      {
         PrintTools.info("updating planar regions");
         super.setPlanarRegions(planarRegionsList);
         initializeForNewPlanarRegions();
      }
   }

   private void setNodesCostsAndRememberIfClosestYet(BipedalFootstepPlannerNode nodeToSetCostOf)
   {
      Point3D currentPosition = nodeToSetCostOf.getSolePosition();
      Point3D goalPosition = planarRegionPotentialNextStepCalculator.getGoalPosition(nodeToSetCostOf.getRobotSide());
      Vector3D currentToGoalVector = new Vector3D();
      currentToGoalVector.sub(goalPosition, currentPosition);

      double euclideanDistanceToGoal = currentToGoalVector.length();
      nodeToSetCostOf.setEstimatedCostToGoal(euclideanDistanceToGoal);

      if (closestNodeToGoal == null || euclideanDistanceToGoal + 0.01 < closestNodeToGoal.getEstimatedCostToGoal())
         replaceBestPlan(nodeToSetCostOf);
   }

   private void replaceBestPlan(BipedalFootstepPlannerNode bestNode)
   {
      closestNodeToGoal = bestNode;
      FootstepPlan newBestPlan = FootstepPlanningUtils.createFootstepPlanFromEndNode(closestNodeToGoal);
      bestPlanYet.set(newBestPlan);
   }

   public void requestStop()
   {
      stopRequested = true;
   }

   @Override
   public void run()
   {
      stopRequested = false;
      plan();
   }

   public void clear()
   {
      this.clear = true;
   }

   public boolean isClear()
   {
      return !clear;
   }

}
