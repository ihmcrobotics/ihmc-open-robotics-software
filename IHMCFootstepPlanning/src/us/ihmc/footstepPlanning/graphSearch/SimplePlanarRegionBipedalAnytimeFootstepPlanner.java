package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

public class SimplePlanarRegionBipedalAnytimeFootstepPlanner extends PlanarRegionBipedalFootstepPlanner implements AnytimeFootstepPlanner, Runnable
{
   private final Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();
   private BipedalFootstepPlannerNode closestNodeToGoal = null;
   private final AtomicReference<FootstepPlan> bestPlanYet = new AtomicReference<>(null);
   private boolean stopRequested = false;
   private final HashMap<Integer, List<BipedalFootstepPlannerNode>> mapToAllExploredNodes = new HashMap<>();
   private boolean isBestPlanYetOptimal = false;
   private boolean alreadySetPlanarRegions = false;
   private final AtomicReference<PlanarRegionsList> planarRegionsListReference = new AtomicReference<>(null);
   private boolean requestInitialize = false;

   public SimplePlanarRegionBipedalAnytimeFootstepPlanner(YoVariableRegistry parentRegistry)
   {
      super(parentRegistry);
   }

   /**
    * @return The FootstepPlan that ends the closest to the goal of all explored yet
    */
   @Override
   public FootstepPlan getBestPlanYet()
   {
      return bestPlanYet.get();
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
//      if(alreadySetPlanarRegions)
//      {
//         return;
//      }
//      else
//      {
         planarRegionsListReference.set(planarRegionsList);
//         alreadySetPlanarRegions = true;
//      }
   }

   private void initialize()
   {
      stack.clear();
      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      stack.push(startNode);
      closestNodeToGoal = null;
      mapToAllExploredNodes.clear();
   }

   @Override
   public void executingFootstep(SimpleFootstep footstep)
   {
      FramePose newStartPose = new FramePose();
      footstep.getSoleFramePose(newStartPose);
      RobotSide newInitialSide = footstep.getRobotSide();

      super.setInitialStanceFoot(newStartPose, newInitialSide);
      requestInitialize = true;
   }

   @Override
   public boolean isBestPlanYetOptimal()
   {
      return isBestPlanYetOptimal;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      goalNode = null;

      while(planarRegionsListReference.get() == null)
      {
         ThreadTools.sleep(100);
      }

      checkForNewPlanarRegionsList();

      while (!stopRequested)
      {
         if(requestInitialize)
         {
            initialize();
            requestInitialize = false;
         }

         checkForNewPlanarRegionsList();

         if(stack.isEmpty())
         {
            ThreadTools.sleep(100);
            continue;
         }

         BipedalFootstepPlannerNode nodeToExpand = stack.pop();
         notifyListenerNodeSelectedForExpansion(nodeToExpand);

         if(nodeToExpand != startNode)
         {
            // Make sure popped node is a good one and can be expanded...
            boolean snapSucceded = snapToPlanarRegionAndCheckIfGoodSnap(nodeToExpand);
            if (!snapSucceded)
               continue;

            boolean goodFootstep = checkIfGoodFootstep(nodeToExpand);
            if (!goodFootstep)
               continue;

            boolean differentFromParent = checkIfDifferentFromGrandParent(nodeToExpand);
            {
               if (!differentFromParent)
                  continue;
            }

            boolean nearbyNodeAlreadyExists = checkIfNearbyNodeAlreadyExistsAndStoreIfNot(nodeToExpand);
            if (nearbyNodeAlreadyExists)
               continue;
         }

         setNodesCostsAndRememberIfClosestYet(nodeToExpand);
         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);
         numberOfNodesExpanded.increment();


         if (nodeToExpand.isAtGoal())
         {
            if ((nodeToExpand.getParentNode() != null) && (nodeToExpand.getParentNode().isAtGoal()))
            {
               goalNode = nodeToExpand;
               closestNodeToGoal = goalNode;
               notifyListenerSolutionWasFound(getPlan());
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
         super.setPlanarRegions(planarRegionsList);
         initialize();
      }
   }

   private void setNodesCostsAndRememberIfClosestYet(BipedalFootstepPlannerNode nodeToSetCostOf)
   {
      Point3d currentPosition = nodeToSetCostOf.getSolePosition();
      Point3d goalPosition = goalPositions.get(nodeToSetCostOf.getRobotSide());
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
         FootstepPlan newBestPlan = new FootstepPlan(closestNodeToGoal);
         bestPlanYet.set(newBestPlan);
      }
   }

   private boolean checkIfNearbyNodeAlreadyExistsAndStoreIfNot(BipedalFootstepPlannerNode nodeToExpand)
   {
      int hashCode = nodeToExpand.hashCode();
      boolean containsHashCode = mapToAllExploredNodes.containsKey(hashCode);
      if (!containsHashCode)
      {
         List<BipedalFootstepPlannerNode> nodesWithThisHash = new ArrayList<>();
         nodesWithThisHash.add(nodeToExpand);
         mapToAllExploredNodes.put(hashCode, nodesWithThisHash);

         return false;
      }
      else
      {
         List<BipedalFootstepPlannerNode> nodesWithThisHash = mapToAllExploredNodes.get(hashCode);

         for (int i = 0; i < nodesWithThisHash.size(); i++)
         {
            BipedalFootstepPlannerNode nodeWithSameHash = nodesWithThisHash.get(i);

            if (nodeToExpand.equals(nodeWithSameHash))
            {
               return true;
            }
         }

         return false;
      }
   }

   public void requestStop()
   {
      stopRequested = true;
   }

   @Override
   public void run()
   {
      plan();
   }
}
