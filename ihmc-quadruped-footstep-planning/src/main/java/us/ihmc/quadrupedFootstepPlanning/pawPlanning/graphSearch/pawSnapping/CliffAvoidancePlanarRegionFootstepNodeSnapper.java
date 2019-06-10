package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawCliffDetectionTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class CliffAvoidancePlanarRegionFootstepNodeSnapper extends SimplePlanarRegionPawNodeSnapper
{
   private final double shortCliffHeightToAvoid;
   private final double tallCliffHeightToAvoid;

   private final double distanceToAvoidShortCliffs;
   private final double distanceToAvoidTallCliffs;


   public CliffAvoidancePlanarRegionFootstepNodeSnapper(PawStepPlannerParametersReadOnly parameters, DoubleProvider projectionInsideDelta,
                                                        BooleanProvider projectInsideUsingConvexHull, boolean enforceTranslationLessThanGridCell)
   {
      super(parameters, projectionInsideDelta, projectInsideUsingConvexHull, enforceTranslationLessThanGridCell);

      tallCliffHeightToAvoid = parameters.getCliffHeightToAvoid();
      shortCliffHeightToAvoid = 0.5 * tallCliffHeightToAvoid;

      distanceToAvoidTallCliffs = 0.1;
      distanceToAvoidShortCliffs = 0.05;
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
         snapTransform = pushAwayFromCliffs(robotQuadrant, pawPosition, yaw, snapTransform, projectionTranslation, highestRegion);

         if (snapTransform == null)
            return PawNodeSnapData.emptyData();

         return new PawNodeSnapData(snapTransform);
      }
   }

   private RigidBodyTransform pushAwayFromCliffs(RobotQuadrant movingQuadrant, Point2DReadOnly footPosition, double footYaw,
                                                 RigidBodyTransform snapTransform, Vector2DReadOnly projectionTranslation,
                                                 PlanarRegion highestRegion)
   {
      Point3D snappedFoot = new Point3D(footPosition);
      Point3D highestNearbyPoint = new Point3D();

      snapTransform.transform(snappedFoot);

      double forward = movingQuadrant.isQuadrantInFront() ?
            parameters.getMinimumFrontEndForwardDistanceFromCliffBottoms() :
            parameters.getMinimumHindEndForwardDistanceFromCliffBottoms();
      double backward = movingQuadrant.isQuadrantInFront() ?
            -parameters.getMinimumFrontEndBackwardDistanceFromCliffBottoms() :
            -parameters.getMinimumHindEndBackwardDistanceFromCliffBottoms();
      double left = parameters.getMinimumLateralDistanceFromCliffBottoms();
      double right = -parameters.getMinimumLateralDistanceFromCliffBottoms();

      PawCliffDetectionTools.findHighestNearbyPoint(planarRegionsList, snappedFoot, footYaw, highestNearbyPoint, forward, backward, left, right);

      double cliffHeight = highestNearbyPoint.getZ() - snappedFoot.getZ();
      if (cliffHeight < shortCliffHeightToAvoid)
         return snapTransform;

      double interpolationAlpha = (cliffHeight - shortCliffHeightToAvoid) / (tallCliffHeightToAvoid - shortCliffHeightToAvoid);
      double distanceToAvoidBy = InterpolationTools.linearInterpolate(distanceToAvoidShortCliffs, distanceToAvoidTallCliffs, interpolationAlpha);

      double distanceToCliff = snappedFoot.distance(highestNearbyPoint);
      if (distanceToCliff < distanceToAvoidBy)
      {
         Vector2D avoidanceTranslation = new Vector2D(snappedFoot.getX() - highestNearbyPoint.getX(), snappedFoot.getY() - highestNearbyPoint.getY());
         avoidanceTranslation.scale(distanceToAvoidBy - distanceToCliff / avoidanceTranslation.length());
         avoidanceTranslation.add(projectionTranslation);

         if (isTranslationBiggerThanGridCell(avoidanceTranslation))
            return null;

         return getSnapTransformIncludingTranslation(footPosition, avoidanceTranslation, highestRegion);
      }

      return snapTransform;
   }


}