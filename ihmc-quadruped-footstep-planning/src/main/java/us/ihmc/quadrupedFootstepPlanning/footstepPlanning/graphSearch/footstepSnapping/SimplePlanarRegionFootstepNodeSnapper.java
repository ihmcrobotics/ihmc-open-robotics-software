package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SimplePlanarRegionFootstepNodeSnapper extends FootstepNodeSnapper
{
   private final Point2D footPosition = new Point2D();

   private final DoubleProvider projectionInsideDelta;
   private final PlanarRegionConstraintDataHolder constraintDataHolder = new PlanarRegionConstraintDataHolder();
   private final PlanarRegionConstraintDataParameters constraintDataParameters = new PlanarRegionConstraintDataParameters();

   public SimplePlanarRegionFootstepNodeSnapper(FootstepPlannerParameters parameters, DoubleProvider projectionInsideDelta,
                                                boolean enforceTranslationLessThanGridCell)
   {
      super(parameters);

      this.projectionInsideDelta = projectionInsideDelta;

      constraintDataParameters.enforceTranslationLessThanGridCell = enforceTranslationLessThanGridCell;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      super.setPlanarRegions(planarRegionsList);
      constraintDataHolder.clear();
      constraintDataParameters.projectionInsideDelta = projectionInsideDelta.getValue();
      constraintDataParameters.projectInsideUsingConvexHull = parameters.getProjectInsideUsingConvexHull();
   }

   @Override
   public FootstepNodeSnapData snapInternal(int xIndex, int yIndex)
   {
      FootstepNodeTools.getFootPosition(xIndex, yIndex, footPosition);
      Vector2D projectionTranslation = new Vector2D();
      PlanarRegion highestRegion = PlanarRegionSnapTools
            .findHighestRegionWithProjection(footPosition, projectionTranslation, constraintDataHolder, planarRegionsList.getPlanarRegionsAsList(),
                                             constraintDataParameters);

      if (highestRegion == null || projectionTranslation.containsNaN() || isTranslationBiggerThanGridCell(projectionTranslation))
      {
         return FootstepNodeSnapData.emptyData();
      }
      else
      {
         double x = xIndex * FootstepNode.gridSizeXY + projectionTranslation.getX();
         double xTranslated = x + projectionTranslation.getX();
         double y = yIndex * FootstepNode.gridSizeXY + projectionTranslation.getY();
         double yTranslated = y + projectionTranslation.getY();
         double z = highestRegion.getPlaneZGivenXY(xTranslated, yTranslated);

         Vector3D surfaceNormal = new Vector3D();
         highestRegion.getNormal(surfaceNormal);

         RigidBodyTransform snapTransform = PlanarRegionSnapTools.createTransformToMatchSurfaceNormalPreserveX(surfaceNormal);
         PlanarRegionSnapTools.setTranslationSettingZAndPreservingXAndY(x, y, xTranslated, yTranslated, z, snapTransform);

         return new FootstepNodeSnapData(snapTransform);
      }
   }

   private boolean isTranslationBiggerThanGridCell(Vector2D translation)
   {
      double maximumTranslationPerAxis = 0.5 * FootstepNode.gridSizeXY;
      return Math.abs(translation.getX()) > maximumTranslationPerAxis || Math.abs(translation.getY()) > maximumTranslationPerAxis;
   }

}