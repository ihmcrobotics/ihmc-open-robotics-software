package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface BipedalFootstepPlannerListener
{
   public abstract void nodeSelectedForExpansion(BipedalFootstepPlannerNode nodeToExpand);
   public abstract void nodeForExpansionWasAccepted(BipedalFootstepPlannerNode nodeToExpand);
   public abstract void nodeForExpansionWasRejected(BipedalFootstepPlannerNode nodeToExpand);
   public abstract void notifyListenerSolutionWasFound();
   public abstract void notifyListenerSolutionWasNotFound();
   public abstract void planarRegionsListSet(PlanarRegionsList planarRegionsList);
}
