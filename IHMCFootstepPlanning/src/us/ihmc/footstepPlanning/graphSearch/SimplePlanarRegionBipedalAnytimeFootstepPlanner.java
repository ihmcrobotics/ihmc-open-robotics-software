package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class SimplePlanarRegionBipedalAnytimeFootstepPlanner extends PlanarRegionBipedalFootstepPlanner implements AnytimeFootstepPlanner, Runnable
{
   private BipedalFootstepPlannerNode closestNodeToGoal = null;
   private final AtomicReference<FootstepPlan> bestPlanYet = new AtomicReference<>(null);
   private boolean stopRequested = false;
   private boolean isBestPlanYetOptimal = false;
   private final AtomicReference<PlanarRegionsList> planarRegionsListReference = new AtomicReference<>(null);
   private final AtomicReference<BipedalFootstepPlannerNode> newStartNodeReference = new AtomicReference<>(null);
   private final IntegerYoVariable maxNumberOfNodesBeforeSleeping = new IntegerYoVariable("maxNumberOfNodesBeforeSleeping", registry);

   private final FramePose tempFramePose = new FramePose();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public SimplePlanarRegionBipedalAnytimeFootstepPlanner(YoVariableRegistry parentRegistry)
   {
      super(parentRegistry);
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

      trimBestPlanToNewPlanarRegions();
   }

   private void trimBestPlanToNewPlanarRegions()
   {
      List<BipedalFootstepPlannerNode> listOfNodes = FootstepPlanningUtils.createListOfNodesFromEndNode(closestNodeToGoal);
      PrintTools.info("trimming plan to new planar regions");

      for (int i = 0; i < listOfNodes.size(); i++)
      {
         BipedalFootstepPlannerNode node = listOfNodes.get(i);
         boolean stillIsValid = planarRegionPotentialNextStepCalculator.snapToPlanarRegionAndCheckIfGoodSnap(node);
         PrintTools.info("trying to snap node with position " + node.getSolePosition());

         if(!stillIsValid)
         {
            closestNodeToGoal = node.getParentNode();
            if(i < listOfNodes.size())
            {
               FootstepPlan trimmedBestPlan = FootstepPlanningUtils.createFootstepPlanFromEndNode(closestNodeToGoal);
               bestPlanYet.set(trimmedBestPlan);
            }

            PrintTools.info("valid up to node " + i + " of " + listOfNodes.size());
            checkIfNearbyNodeAlreadyExistsAndStoreIfNot(node);
            break;
         }
         else
         {
            checkIfNearbyNodeAlreadyExistsAndStoreIfNot(node);
            PrintTools.info("valid node");
         }
      }
   }

   @Override
   public void executingFootstep(SimpleFootstep footstep)
   {
      footstep.getSoleFramePose(tempFramePose);
      tempFramePose.getRigidBodyTransform(tempTransform);
      BipedalFootstepPlannerNode newStartNode = new BipedalFootstepPlannerNode(footstep.getRobotSide(), tempTransform);
      newStartNodeReference.set(newStartNode);

      PrintTools.info("executing footstep");
   }

   private void replaceStartNode()
   {
      BipedalFootstepPlannerNode newStartNode = newStartNodeReference.getAndSet(null);
      if(newStartNode != null)
      {
         ArrayList<BipedalFootstepPlannerNode> childrenOfStartNode = new ArrayList<>();
         startNode.getChildren(childrenOfStartNode);
         PrintTools.info("clearing children of start node");

         for(BipedalFootstepPlannerNode node : childrenOfStartNode)
         {
            if(!node.equals(newStartNode))
            {
               recursivelyMarkAsDead(node);
            }
         }

         startNode = newStartNode;
         initialSide = startNode.getRobotSide();
         startNode.getSoleTransform(initialFootPose);
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
         replaceStartNode();
         checkForNewPlanarRegionsList();

         if(stack.size() > maxNumberOfNodesBeforeSleeping.getIntegerValue())
            ThreadTools.sleep(100);

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

         boolean nearbyNodeAlreadyExists = checkIfNearbyNodeAlreadyExistsAndStoreIfNot(nodeToExpand);
         if (nearbyNodeAlreadyExists)
            continue;

         setNodesCostsAndRememberIfClosestYet(nodeToExpand);
         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);

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

      double costFromParent = 1.0;
      double costToHereFromStart;
      if (nodeToSetCostOf.getParentNode() == null)
      {
         costToHereFromStart = 0.0;
      }
      else
      {
         costToHereFromStart = nodeToSetCostOf.getParentNode().getCostToHereFromStart() + costFromParent;
      }
      nodeToSetCostOf.setCostFromParent(costFromParent);
      nodeToSetCostOf.setCostToHereFromStart(costToHereFromStart);

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
