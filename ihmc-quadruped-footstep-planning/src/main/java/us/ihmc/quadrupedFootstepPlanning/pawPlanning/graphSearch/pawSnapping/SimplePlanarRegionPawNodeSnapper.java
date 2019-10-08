package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SimplePlanarRegionPawNodeSnapper extends PawNodeSnapper
{
   protected final Point2D pawPosition = new Point2D();

   private final DoubleProvider projectionInsideDelta;
   private final DoubleProvider minimumProjectionInsideDelta;
   private final BooleanProvider projectInsideUsingConvexHull;
   protected final PlanarRegionPawConstraintDataHolder constraintDataHolder = new PlanarRegionPawConstraintDataHolder();
   protected final PlanarRegionPawConstraintDataParameters constraintDataParameters = new PlanarRegionPawConstraintDataParameters();

   public SimplePlanarRegionPawNodeSnapper(PawStepPlannerParametersReadOnly parameters, boolean enforceTranslationLessThanGridCell)
   {
      this(parameters,
           parameters::getProjectInsideDistance,
           parameters::getMinimumProjectInsideDistance,
           parameters::getProjectInsideUsingConvexHull,
           enforceTranslationLessThanGridCell);
   }

   public SimplePlanarRegionPawNodeSnapper(PawStepPlannerParametersReadOnly parameters, DoubleProvider projectionInsideDelta,
                                           DoubleProvider minimumProjectionInsideDelta, BooleanProvider projectInsideUsingConvexHull, boolean enforceTranslationLessThanGridCell)
   {
      super(parameters);

      this.projectionInsideDelta = projectionInsideDelta;
      this.minimumProjectionInsideDelta = minimumProjectionInsideDelta;
      this.projectInsideUsingConvexHull = projectInsideUsingConvexHull;

      constraintDataParameters.enforceTranslationLessThanGridCell = enforceTranslationLessThanGridCell;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      super.setPlanarRegions(planarRegionsList);
      constraintDataHolder.clear();
      constraintDataParameters.projectionInsideDelta = projectionInsideDelta.getValue();
      constraintDataParameters.minimumProjectionInsideDelta = minimumProjectionInsideDelta.getValue();
      constraintDataParameters.projectInsideUsingConvexHull = projectInsideUsingConvexHull.getValue();
   }

   @Override
   public PawNodeSnapData snapInternal(RobotQuadrant robotQuadrant, int xIndex, int yIndex, double yaw)
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
         RigidBodyTransform snapTransform = getSnapTransformIncludingTranslation(pawPosition, projectionTranslation, highestRegion);
         return new PawNodeSnapData(snapTransform);
      }
   }

   protected RigidBodyTransform getSnapTransformIncludingTranslation(Point2DReadOnly position, Vector2DReadOnly translation, PlanarRegion containingRegion)
   {
      double x = position.getX();
      double xTranslated = x + translation.getX();
      double y = position.getY();
      double yTranslated = y + translation.getY();
      double z = containingRegion.getPlaneZGivenXY(xTranslated, yTranslated);

      Vector3D surfaceNormal = new Vector3D();
      containingRegion.getNormal(surfaceNormal);

      RigidBodyTransform snapTransform = PlanarRegionPawSnapTools.createTransformToMatchSurfaceNormalPreserveX(surfaceNormal);
      PlanarRegionPawSnapTools.setTranslationSettingZAndPreservingXAndY(x, y, xTranslated, yTranslated, z, snapTransform);

      return snapTransform;
   }

   protected boolean isTranslationBiggerThanGridCell(Vector2D translation)
   {
      if (!constraintDataParameters.enforceTranslationLessThanGridCell)
         return false;

      double maximumTranslationPerAxis = 0.5 * PawNode.gridSizeXY;
      return Math.abs(translation.getX()) > maximumTranslationPerAxis || Math.abs(translation.getY()) > maximumTranslationPerAxis;
   }

}