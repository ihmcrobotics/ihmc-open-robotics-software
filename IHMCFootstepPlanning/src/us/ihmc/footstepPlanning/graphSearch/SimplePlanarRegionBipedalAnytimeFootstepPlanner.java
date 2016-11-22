package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.*;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.*;

public class SimplePlanarRegionBipedalAnytimeFootstepPlanner extends PlanarRegionBipedalFootstepPlanner implements AnytimeFootstepPlanner
{
   private final Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();
   private BipedalFootstepPlannerNode closestNodeToGoal = null;
   private boolean stopRequested = false;
   private final HashMap<Integer, List<BipedalFootstepPlannerNode>> mapToAllExploredNodes = new HashMap<>();

   /**
    * @return The FootstepPlan that ends the closest to the goal of all explored yet
    */
   @Override
   public synchronized FootstepPlan getBestPlanYet()
   {
      FootstepPlan bestPlanYet = new FootstepPlan(closestNodeToGoal);
      return bestPlanYet;
   }

   @Override
   public void executingFootstep(SimpleFootstep footstep)
   {
      stack.clear();
      FramePose newStartPose = new FramePose();
      footstep.getSoleFramePose(newStartPose);
      RobotSide newInitialSide = footstep.getRobotSide();

      super.setInitialStanceFoot(newStartPose, newInitialSide);
   }

   @Override
   public FootstepPlanningResult plan()
   {
      goalNode = null;

      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      stack.push(startNode);

      while (!stack.isEmpty() && !stopRequested)
      {
         BipedalFootstepPlannerNode nodeToExpand = stack.pop();
         notifyListenerNodeSelectedForExpansion(nodeToExpand);

         // Make sure popped node is a good one and can be expanded...
         boolean snapSucceded = snapToPlanarRegionAndCheckIfGoodSnap(nodeToExpand);
         if (!snapSucceded)
            continue;

         boolean goodFootstep = checkIfGoodFootstep(nodeToExpand);
         if (!goodFootstep)
            continue;

         boolean nearbyNodeAlreadyExists = checkIfNearbyNodeAlreadyExistsAndStoreIfNot(nodeToExpand);
         if (nearbyNodeAlreadyExists)
            continue;

         setNodesCostsAndRememberIfClosestYet(nodeToExpand);
         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);

         if (nodeToExpand.isAtGoal())
         {
            if ((nodeToExpand.getParentNode() != null) && (nodeToExpand.getParentNode().isAtGoal()))
            {
               goalNode = nodeToExpand;
               notifyListenerSolutionWasFound(goalNode);
               return FootstepPlanningResult.OPTIMAL_SOLUTION;
            }
         }

         RigidBodyTransform soleZUpTransform = new RigidBodyTransform();
         nodeToExpand.getSoleTransform(soleZUpTransform);
         setTransformZUpPreserveX(soleZUpTransform);

         // Check if goal is reachable:
         boolean goalIsReachable = addGoalNodeIfGoalIsReachable(nodeToExpand, soleZUpTransform, stack);
         if (goalIsReachable)
            continue;

         expandChildrenAndAddToQueue(stack, soleZUpTransform, nodeToExpand);
      }

      notifyListenerSolutionWasNotFound();
      return FootstepPlanningResult.NO_PATH_EXISTS;
   }

   private synchronized void setNodesCostsAndRememberIfClosestYet(BipedalFootstepPlannerNode nodeToSetCostOf)
   {
      Point3d currentPosition = nodeToSetCostOf.getSolePosition();
      Point3d goalPosition = goalPositions.get(nodeToSetCostOf.getRobotSide());
      Vector3d currentToGoalVector = new Vector3d();
      currentToGoalVector.sub(goalPosition, currentPosition);

      double costFromParent = 1.0;
      double costToHereFromStart;
      if(nodeToSetCostOf.getParentNode() == null)
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

      if(closestNodeToGoal == null || euclideanDistanceToGoal < closestNodeToGoal.getEstimatedCostToGoal())
      {
         closestNodeToGoal = nodeToSetCostOf;
      }
   }

   private boolean checkIfNearbyNodeAlreadyExistsAndStoreIfNot(BipedalFootstepPlannerNode nodeToExpand)
   {
      int hashCode = nodeToExpand.hashCode();
      boolean containsHashCode = mapToAllExploredNodes.containsKey(hashCode);
      if(!containsHashCode)
      {
         List<BipedalFootstepPlannerNode> nodesWithThisHash = new ArrayList<>();
         nodesWithThisHash.add(nodeToExpand);
         mapToAllExploredNodes.put(hashCode, nodesWithThisHash);

         return false;
      }
      else
      {
         List<BipedalFootstepPlannerNode> nodesWithThisHash = mapToAllExploredNodes.get(hashCode);

         for(int i = 0; i < nodesWithThisHash.size(); i++)
         {
            BipedalFootstepPlannerNode nodeWithSameHash = nodesWithThisHash.get(i);

            if(nodeToExpand.equals(nodeWithSameHash))
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
}
