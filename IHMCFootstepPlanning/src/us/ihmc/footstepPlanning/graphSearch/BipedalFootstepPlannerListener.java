package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface BipedalFootstepPlannerListener
{
   public abstract void goalWasSet(RigidBodyTransform goalLeftFootPose, RigidBodyTransform goalRightFootPose);
   public abstract void startNodeWasAdded(BipedalFootstepPlannerNode startNode);
   public abstract void planarRegionsListSet(PlanarRegionsList planarRegionsList);
   
   public abstract void nodeIsBeingExpanded(BipedalFootstepPlannerNode nodeToExpand);

   public abstract void nodeUnderConsideration(BipedalFootstepPlannerNode nodeToExpand);
   public abstract void nodeUnderConsiderationWasRejected(BipedalFootstepPlannerNode rejectedNode, BipedalFootstepPlannerNodeRejectionReason reason);
   public abstract void nodeUnderConsiderationWasSuccessful(BipedalFootstepPlannerNode node);

   public abstract void solutionWasFound(FootstepPlan footstepPlan);
   public abstract void solutionWasNotFound();
}
