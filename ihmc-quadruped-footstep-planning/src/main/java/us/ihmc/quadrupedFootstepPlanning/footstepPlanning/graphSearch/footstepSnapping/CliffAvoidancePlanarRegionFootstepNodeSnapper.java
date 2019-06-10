package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.CliffDetectionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class CliffAvoidancePlanarRegionFootstepNodeSnapper extends SimplePlanarRegionFootstepNodeSnapper
{
   private final double shortCliffHeightToAvoid;
   private final double tallCliffHeightToAvoid;

   private final double distanceToAvoidShortCliffs;
   private final double distanceToAvoidTallCliffs;


   public CliffAvoidancePlanarRegionFootstepNodeSnapper(FootstepPlannerParameters parameters, DoubleProvider projectionInsideDelta,
                                                        BooleanProvider projectInsideUsingConvexHull, boolean enforceTranslationLessThanGridCell)
   {
      super(parameters, projectionInsideDelta, projectInsideUsingConvexHull, enforceTranslationLessThanGridCell);

      tallCliffHeightToAvoid = parameters.getCliffHeightToAvoid();
      shortCliffHeightToAvoid = 0.5 * tallCliffHeightToAvoid;

      distanceToAvoidTallCliffs = 0.1;
      distanceToAvoidShortCliffs = 0.05;
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
         RigidBodyTransform snapTransform = getSnapTransformIncludingTranslation(footPosition, projectionTranslation, highestRegion);
         snapTransform = pushAwayFromCliffs(footPosition, snapTransform, projectionTranslation, highestRegion);

         if (snapTransform == null)
            return FootstepNodeSnapData.emptyData();

         return new FootstepNodeSnapData(snapTransform);
      }
   }

   private RigidBodyTransform pushAwayFromCliffs(Point2DReadOnly footPosition, RigidBodyTransform snapTransform, Vector2DReadOnly projectionTranslation,
                                                 PlanarRegion highestRegion)
   {
      Point3D snappedFoot = new Point3D(footPosition);
      Point3D highestNearbyPoint = new Point3D();

      snapTransform.transform(snappedFoot);
      CliffDetectionTools.findHighestNearbyRegion(planarRegionsList, snappedFoot, highestNearbyPoint, distanceToAvoidTallCliffs);

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