package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.AnytimeFootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlanningUtils;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class SimplePlanarRegionBipedalAnytimeFootstepPlanner extends PlanarRegionBipedalFootstepPlanner implements AnytimeFootstepPlanner, Runnable
{
   private final String namePrefix = "AnytimePlanner_";

   private BipedalFootstepPlannerNode closestNodeToGoal = null;
   private final AtomicReference<FootstepPlan> bestPlanYet = new AtomicReference<>(null);
   private boolean stopRequested = false;
   private boolean isBestPlanYetOptimal = false;
   private final AtomicReference<PlanarRegionsList> planarRegionsListReference = new AtomicReference<>(null);
   private final AtomicReference<SimpleFootstep> latestExecutedFootstepReference = new AtomicReference<>(null);
   private final IntegerYoVariable maxNumberOfNodesBeforeSleeping = new IntegerYoVariable(namePrefix + "maxNumberOfNodesBeforeSleeping", registry);
   private final IntegerYoVariable stackSize = new IntegerYoVariable(namePrefix + "stackSize", registry);

   private final FramePose tempFramePose = new FramePose();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   private BipedalFootstepPlannerNode parentOfStartNode = null;

   public SimplePlanarRegionBipedalAnytimeFootstepPlanner(BipedalFootstepPlannerParameters parameters, YoVariableRegistry parentRegistry)
   {
      super(parameters, parentRegistry);
      maxNumberOfNodesBeforeSleeping.set(5000);
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
      parentOfStartNode = null;
      
      stack.clear();
      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      stack.push(startNode);
      closestNodeToGoal = null;
      mapToAllExploredNodes.clear();
      bestPlanYet.set(null);
   }

   private void initializeForNewPlanarRegions()
   {
      stack.clear();
      stack.push(startNode);
      mapToAllExploredNodes.clear();

      closestNodeToGoal = null;
      bestPlanYet.set(null);
//      trimBestPlanToNewPlanarRegions();
   }

   private void trimBestPlanToNewPlanarRegions()
   {
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
   }

   @Override
   public void executingFootstep(SimpleFootstep footstep)
   {
      latestExecutedFootstepReference.set(footstep);
   }

   private void replaceStartNode()
   {
//      BipedalFootstepPlannerNode newStartNode = newStartNodeReference.getAndSet(null);
//      if(newStartNode != null)
//      {
//         ArrayList<BipedalFootstepPlannerNode> childrenOfStartNode = new ArrayList<>();
//         startNode.getChildren(childrenOfStartNode);
//         PrintTools.info("clearing children of start node");
//
//         for (BipedalFootstepPlannerNode node : childrenOfStartNode)
//         {
//            if (!node.equals(newStartNode))
//            {
//               recursivelyMarkAsDead(node);
//            }
//            else
//            {
//               PrintTools.info("found new start node");
//               startNode = node;
//            }
//         }
//
//         initialSide = startNode.getRobotSide();
//         startNode.getSoleTransform(initialFootPose);
//      }

      SimpleFootstep footstep = latestExecutedFootstepReference.getAndSet(null);
      if (footstep != null)
      {
         FramePose tempPose = new FramePose();
         initialSide = footstep.getRobotSide();
         footstep.getSoleFramePose(tempPose);
         tempPose.getRigidBodyTransform(initialFootPose);
         
         parentOfStartNode = startNode;
         startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      }
   }

   private void recursivelyMarkAsDead(BipedalFootstepPlannerNode node)
   {
      node.setToDead();
      ArrayList<BipedalFootstepPlannerNode> childrenNodes = new ArrayList<>();
      node.getChildren(childrenNodes);

      for(BipedalFootstepPlannerNode childNode : childrenNodes)
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
      return plan(true);
   }

   public FootstepPlanningResult plan(boolean stopAndReturnWhenGoalIsFound)
   {
      initialize();
      goalNode = null;
      footstepPlan = null;

      planarRegionPotentialNextStepCalculator.setStartNode(startNode);

      while (!stopRequested)
      {
         stackSize.set(stack.size());
         replaceStartNode();
         checkForNewPlanarRegionsList();

         if(stackSize.getIntegerValue() > maxNumberOfNodesBeforeSleeping.getIntegerValue())
         {
            ThreadTools.sleep(100);
            continue;
         }

         if (stack.isEmpty())
         {
            if (stopAndReturnWhenGoalIsFound)
            {
               notifyListenerSolutionWasNotFound();
               return FootstepPlanningResult.NO_PATH_EXISTS;
            }

            ThreadTools.sleep(100);
            continue;
         }

         BipedalFootstepPlannerNode nodeToExpand = stack.pop();

         if(nodeToExpand.isDead())
            continue;

         // If stepping in place on first step, don't...
         if (parentOfStartNode != null)
         {
            if (parentOfStartNode.epsilonEquals(nodeToExpand, 0.01))
               continue;
         }

         boolean nearbyNodeAlreadyExists = checkIfNearbyNodeAlreadyExistsAndStoreIfNot(nodeToExpand) != null;
         if (nearbyNodeAlreadyExists)
            continue;

         setNodesCostsAndRememberIfClosestYet(nodeToExpand);
         notifyListenerNodeIsBeingExpanded(nodeToExpand);

         if (nodeToExpand.isAtGoal())
         {
            if ((nodeToExpand.getParentNode() != null) && (nodeToExpand.getParentNode().isAtGoal()) && stopAndReturnWhenGoalIsFound)
            {
               goalNode = nodeToExpand;

               notifyListenerSolutionWasFound(getPlan());
               return FootstepPlanningResult.OPTIMAL_SOLUTION;
            }
         }

         expandChildrenAndAddNodes(stack, nodeToExpand);
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
      Point3d currentPosition = nodeToSetCostOf.getSolePosition();
      Point3d goalPosition = planarRegionPotentialNextStepCalculator.getGoalPosition(nodeToSetCostOf.getRobotSide());
      Vector3d currentToGoalVector = new Vector3d();
      currentToGoalVector.sub(goalPosition, currentPosition);

      double euclideanDistanceToGoal = currentToGoalVector.length();
      nodeToSetCostOf.setEstimatedCostToGoal(euclideanDistanceToGoal);

      if (closestNodeToGoal == null || euclideanDistanceToGoal < closestNodeToGoal.getEstimatedCostToGoal())
      {
         closestNodeToGoal = nodeToSetCostOf;
         FootstepPlan newBestPlan = FootstepPlanningUtils.createFootstepPlanFromEndNode(closestNodeToGoal);
         bestPlanYet.set(newBestPlan);
      }
   }

   public void requestStop()
   {
      stopRequested = true;
   }

   @Override
   public void run()
   {
      stopRequested = false;
      plan(false);
   }
}
