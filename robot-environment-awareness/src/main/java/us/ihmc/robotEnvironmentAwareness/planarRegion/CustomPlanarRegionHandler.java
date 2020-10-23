package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.REAGeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class CustomPlanarRegionHandler
{
   public static void performConvexDecompositionIfNeeded(PlanarRegion customRegion)
   {
      if (customRegion.getNumberOfConvexPolygons() > 0)
         return;
      if (customRegion.getConcaveHull().isEmpty())
         throw new IllegalArgumentException("Invalid planar region: missing the concave hull information.");

      List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
      double depthThreshold = 0.01;
      ConcaveHullDecomposition.recursiveApproximateDecomposition(customRegion.getConcaveHull(), depthThreshold, decomposedPolygons);
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      customRegion.getTransformToWorld(transformToWorld);
      customRegion.set(transformToWorld, decomposedPolygons);
   }

   /**
    * Attempts for each custom region to find any estimated region it can be merged to.
    * <p>
    * When merging a custom region to an estimated region, the edges are simply added as
    * intersections which are used in the polygonization.
    * </p>
    * 
    * @param customRegions the planar regions provided from outside REA. Not modified.
    * @param estimatedRegions the current segmentation data obtained from the sensor(s). Modified:
    *           any custom region may be merged to these estimated regions.
    * @param parameters this method recycle the segmentation parameters to identify whether a custom
    *           region should be merged to an estimated region.
    * @return the list of custom regions that were not merged.
    */
   public static List<PlanarRegion> mergeCustomRegionsToEstimatedRegions(Collection<PlanarRegion> customRegions,
                                                                         List<PlanarRegionSegmentationRawData> estimatedRegions,
                                                                         CustomRegionMergeParameters parameters)
   {
      boolean atLeastOneRegionWasMerged = true;

      List<PlanarRegion> previousUnmergedRegions = new ArrayList<>(customRegions);
      List<PlanarRegion> currentUnmergedRegions = Collections.emptyList();

      while (atLeastOneRegionWasMerged)
      {
         currentUnmergedRegions = previousUnmergedRegions.stream()
                                                         .filter(customRegion -> mergeCustomRegionToEstimatedRegions(customRegion, estimatedRegions,
                                                                                                                     parameters) == null)
                                                         .collect(Collectors.toList());

         atLeastOneRegionWasMerged = currentUnmergedRegions.size() < previousUnmergedRegions.size();
         previousUnmergedRegions = currentUnmergedRegions;
      }

      return currentUnmergedRegions;
   }

   public static List<PlanarRegionSegmentationRawData> mergeCustomRegionToEstimatedRegions(PlanarRegion customRegion,
                                                                                           List<PlanarRegionSegmentationRawData> estimatedRegions,
                                                                                           CustomRegionMergeParameters parameters)
   {
      List<PlanarRegionSegmentationRawData> modifiedEstimatedRegions = new ArrayList<>();

      for (PlanarRegionSegmentationRawData estimatedRegion : estimatedRegions)
      {
         if (!isCustomRegionMergeableToEstimatedRegion(customRegion, estimatedRegion, parameters))
            continue;

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         customRegion.getTransformToWorld(transformToWorld);

         List<Point3D> vertices = customRegion.getConcaveHull().stream().map(Point3D::new).peek(transformToWorld::transform).collect(Collectors.toList());

         Point3D previousVertex = vertices.get(vertices.size() - 1);

         for (Point3D vertex : vertices)
         {
            estimatedRegion.addIntersection(new LineSegment3D(previousVertex, vertex));
            previousVertex = vertex;
         }

         modifiedEstimatedRegions.add(estimatedRegion);
      }

      return modifiedEstimatedRegions.isEmpty() ? null : modifiedEstimatedRegions;
   }

   private static boolean isCustomRegionMergeableToEstimatedRegion(PlanarRegion customRegion, PlanarRegionSegmentationRawData estimatedRegion,
                                                                   CustomRegionMergeParameters parameters)
   {
      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();

      double distanceFromPlane = EuclidGeometryTools.distanceFromPoint3DToPlane3D(customRegion.getPlane().getPoint(), estimatedRegion.getOrigin(),
                                                                                  estimatedRegion.getNormal());
      if (distanceFromPlane > maxDistanceFromPlane)
         return false;

      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());

      double normalDotProduct = Math.abs(estimatedRegion.getNormal().dot(customRegion.getNormal()));
      if (normalDotProduct < dotThreshold)
         return false;

      double searchRadius = parameters.getSearchRadius();
      double searchRadiusSquared = searchRadius * searchRadius;

      BoundingBox3D customRegionBBX = customRegion.getBoundingBox3dInWorld();

      double bbxDistance = REAGeometryTools.distanceSquaredBetweenTwoBoundingBox3Ds(estimatedRegion.getBoundingBoxInWorld().getMinPoint(),
                                                                                    estimatedRegion.getBoundingBoxInWorld().getMaxPoint(),
                                                                                    customRegionBBX.getMinPoint(), customRegionBBX.getMaxPoint());
      if (bbxDistance > searchRadiusSquared)
         return false;

      return estimatedRegion.getPointCloudInWorld().parallelStream().map(Point3D::new).peek(customRegion::transformFromWorldToLocal).map(Point2D::new)
                            .anyMatch(point -> customRegion.distanceToPoint(point) < searchRadius);
   }
}
