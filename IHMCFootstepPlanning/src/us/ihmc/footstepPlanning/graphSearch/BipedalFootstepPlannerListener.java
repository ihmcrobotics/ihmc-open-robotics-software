package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface BipedalFootstepPlannerListener
{
   public abstract void nodeSelectedForExpansion(BipedalFootstepPlannerNode nodeToExpand);
   public abstract void nodeForExpansionWasAccepted(BipedalFootstepPlannerNode acceptedNode);
   public abstract void nodeForExpansionWasRejected(BipedalFootstepPlannerNode rejectedNode);
   public abstract void notifyListenerSolutionWasFound();
   public abstract void notifyListenerSolutionWasNotFound();
   public abstract void planarRegionsListSet(PlanarRegionsList planarRegionsList);
   public abstract void goalWasSet(RigidBodyTransform goalLeftFootPose, RigidBodyTransform goalRightFootPose);
}
