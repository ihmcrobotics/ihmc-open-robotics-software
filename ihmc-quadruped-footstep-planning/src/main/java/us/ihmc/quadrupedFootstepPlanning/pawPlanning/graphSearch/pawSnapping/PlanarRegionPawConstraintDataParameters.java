package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

public class PlanarRegionPawConstraintDataParameters
{
   public boolean projectInsideUsingConvexHull;
   public double projectionInsideDelta;
   public double minimumProjectionInsideDelta = -1.0;
   public boolean enforceTranslationLessThanGridCell;
}
