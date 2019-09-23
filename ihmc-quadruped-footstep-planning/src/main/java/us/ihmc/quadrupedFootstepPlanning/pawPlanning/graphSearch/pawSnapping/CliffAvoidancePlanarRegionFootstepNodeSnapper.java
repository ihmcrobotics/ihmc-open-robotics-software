package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
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
   private final double cliffHeightToAvoid;

   public CliffAvoidancePlanarRegionFootstepNodeSnapper(PawStepPlannerParametersReadOnly parameters, boolean enforceTranslationLessThanGridCell)
   {
      super(parameters, enforceTranslationLessThanGridCell);

      cliffHeightToAvoid = parameters.getCliffHeightToAvoid();
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

      RigidBodyTransform transformToRegion = new RigidBodyTransform();
      transformToRegion.setRotationYaw(footYaw);

      ConvexPolygon2D avoidanceRegion = new ConvexPolygon2D();
      avoidanceRegion.addVertex(forward, left);
      avoidanceRegion.addVertex(forward, right);
      avoidanceRegion.addVertex(backward, left);
      avoidanceRegion.addVertex(backward, right);
      avoidanceRegion.update();
      avoidanceRegion.applyTransform(transformToRegion);
      avoidanceRegion.translate(snappedFoot.getX(), snappedFoot.getY());

      double cliffHeight = PawCliffDetectionTools.findHighestNearbyPoint(planarRegionsList, snappedFoot, highestNearbyPoint, avoidanceRegion);

      if (highestNearbyPoint.distance(snappedFoot) < 1e-2)
         return snapTransform;

      Vector2D avoidanceDirection = new Vector2D(highestNearbyPoint);
      avoidanceDirection.sub(snappedFoot.getX(), snappedFoot.getY());

      Point2DReadOnly intersection = EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(footPosition, avoidanceDirection, avoidanceRegion.getVertexBufferView(), avoidanceRegion.getNumberOfVertices(), true)[0];

      // clean this up.
      double nominalDistanceToAvoidBy = snappedFoot.distanceXY(intersection);
      double scaledDistanceToAvoidBy = InterpolationTools.linearInterpolate(0.0, nominalDistanceToAvoidBy, Math.min(cliffHeight / cliffHeightToAvoid, 1.0));

      double distanceToCliff = snappedFoot.distanceXY(highestNearbyPoint);
      if (distanceToCliff < scaledDistanceToAvoidBy)
      {
         avoidanceDirection.negate();
         avoidanceDirection.scale(scaledDistanceToAvoidBy - distanceToCliff / avoidanceDirection.length());
         avoidanceDirection.add(projectionTranslation);

         if (isTranslationBiggerThanGridCell(avoidanceDirection))
            return null;

         return getSnapTransformIncludingTranslation(footPosition, avoidanceDirection, highestRegion);
      }

      return snapTransform;
   }


}