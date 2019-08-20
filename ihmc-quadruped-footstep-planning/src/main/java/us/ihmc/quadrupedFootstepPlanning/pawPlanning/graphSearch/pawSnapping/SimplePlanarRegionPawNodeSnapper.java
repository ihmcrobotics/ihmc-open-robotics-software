package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SimplePlanarRegionPawNodeSnapper extends PawNodeSnapper
{
   private final Point2D pawPosition = new Point2D();

   private final DoubleProvider projectionInsideDelta;
   private final BooleanProvider projectInsideUsingConvexHull;
   private final PlanarRegionPawConstraintDataHolder constraintDataHolder = new PlanarRegionPawConstraintDataHolder();
   private final PlanarRegionPawConstraintDataParameters constraintDataParameters = new PlanarRegionPawConstraintDataParameters();

   public SimplePlanarRegionPawNodeSnapper(PawPlannerParameters parameters, DoubleProvider projectionInsideDelta,
                                           BooleanProvider projectInsideUsingConvexHull, boolean enforceTranslationLessThanGridCell)
   {
      super(parameters);

      this.projectionInsideDelta = projectionInsideDelta;
      this.projectInsideUsingConvexHull = projectInsideUsingConvexHull;

      constraintDataParameters.enforceTranslationLessThanGridCell = enforceTranslationLessThanGridCell;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      super.setPlanarRegions(planarRegionsList);
      constraintDataHolder.clear();
      constraintDataParameters.projectionInsideDelta = projectionInsideDelta.getValue();
      constraintDataParameters.projectInsideUsingConvexHull = projectInsideUsingConvexHull.getValue();
   }

   @Override
   public PawNodeSnapData snapInternal(int xIndex, int yIndex)
   {
      PawNodeTools.getPawPosition(xIndex, yIndex, pawPosition);
      Vector2D projectionTranslation = new Vector2D();
      PlanarRegion highestRegion = PlanarRegionPawSnapTools
            .findHighestRegionWithProjection(pawPosition, projectionTranslation, constraintDataHolder, planarRegionsList.getPlanarRegionsAsList(),
                                             constraintDataParameters);

      if (highestRegion == null || projectionTranslation.containsNaN() || isTranslationBiggerThanGridCell(projectionTranslation))
      {
         return PawNodeSnapData.emptyData();
      }
      else
      {
         double x = xIndex * PawNode.gridSizeXY + projectionTranslation.getX();
         double xTranslated = x + projectionTranslation.getX();
         double y = yIndex * PawNode.gridSizeXY + projectionTranslation.getY();
         double yTranslated = y + projectionTranslation.getY();
         double z = highestRegion.getPlaneZGivenXY(xTranslated, yTranslated);

         Vector3D surfaceNormal = new Vector3D();
         highestRegion.getNormal(surfaceNormal);

         RigidBodyTransform snapTransform = PlanarRegionPawSnapTools.createTransformToMatchSurfaceNormalPreserveX(surfaceNormal);
         PlanarRegionPawSnapTools.setTranslationSettingZAndPreservingXAndY(x, y, xTranslated, yTranslated, z, snapTransform);

         return new PawNodeSnapData(snapTransform);
      }
   }

   private boolean isTranslationBiggerThanGridCell(Vector2D translation)
   {
      if (!constraintDataParameters.enforceTranslationLessThanGridCell)
         return false;

      double maximumTranslationPerAxis = 0.5 * PawNode.gridSizeXY;
      return Math.abs(translation.getX()) > maximumTranslationPerAxis || Math.abs(translation.getY()) > maximumTranslationPerAxis;
   }

}