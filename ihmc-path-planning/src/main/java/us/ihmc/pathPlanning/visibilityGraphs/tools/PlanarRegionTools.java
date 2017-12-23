package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.PlanarRegionFilter;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.lists.ListWrappingIndexTools;

public class PlanarRegionTools
{
   /**
    * Finds and returns the closest point the the provided point on the planar regions.
    */
   public static Point3D projectPointToPlanes(Point3DReadOnly point, PlanarRegionsList regions)
   {
      double smallestDistance = Double.POSITIVE_INFINITY;
      Point3D closestPoint = null;

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         Point3D intersection = closestPointOnPlane(point, region);
         double distance = intersection.distance(point);

         if (closestPoint == null || distance < smallestDistance)
         {
            smallestDistance = distance;
            closestPoint = intersection;
         }
      }

      return closestPoint;
   }

   /**
    * Projects the given point onto the planar region, returning the closest point on the region to
    * the provided point.
    */
   public static Point3D closestPointOnPlane(Point3DReadOnly point, PlanarRegion region)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      region.getTransformToWorld(regionToWorld);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);
      planeNormal.applyTransform(regionToWorld);

      Point3D pointOnPlane = new Point3D();
      pointOnPlane.set(region.getConvexPolygon(0).getVertex(0));
      pointOnPlane.applyTransform(regionToWorld);

      Point3D intersectionWithPlane = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, point, planeNormal);
      if (intersectionWithPlane == null)
      {
         return null;
      }

      Point3D intersectionInPlaneFrame = new Point3D(intersectionWithPlane);
      intersectionInPlaneFrame.applyInverseTransform(regionToWorld);
      Point2D intersectionInPlaneFrame2D = new Point2D(intersectionInPlaneFrame);

      // checking convex hull here - might be better to check all polygons to avoid false positive
      ConvexPolygon2D convexHull = region.getConvexHull();
      if (!convexHull.isPointInside(intersectionInPlaneFrame2D))
      {
         convexHull.orthogonalProjection(intersectionInPlaneFrame2D);
         intersectionWithPlane.setToZero();
         intersectionWithPlane.set(intersectionInPlaneFrame2D);
         intersectionWithPlane.applyTransform(regionToWorld);
      }
      return intersectionWithPlane;
   }

   /**
    * Projects the given point onto a planar region from the list. The projection is done along the
    * z axis in world frame and if there is multiple regions that the point can be projected onto,
    * the highest intersection point will be returned.
    * <p>
    * Will return null if the is no planar region above or below the point.
    */
   public static Point3D projectPointToPlanesVertically(Point3DReadOnly point, PlanarRegionsList regions)
   {
      return projectPointToPlanesVertically(point, regions.getPlanarRegionsAsList());
   }

   /**
    * Projects the given point onto a planar region from the list. The projection is done along the
    * z axis in world frame and if there is multiple regions that the point can be projected onto,
    * the highest intersection point will be returned.
    * <p>
    * Will return null if the is no planar region above or below the point.
    */
   public static Point3D projectPointToPlanesVertically(Point3DReadOnly point, List<PlanarRegion> regions)
   {
      Line3D projectionLine = new Line3D(point, new Vector3D(0.0, 0.0, 1.0));
      Point3D highestIntersection = null;

      for (PlanarRegion region : regions)
      {
         Point3D intersection = intersectRegionWithLine(region, projectionLine);

         if (intersection == null)
         {
            continue;
         }

         if (highestIntersection == null || highestIntersection.getZ() < intersection.getZ())
         {
            highestIntersection = intersection;
         }
      }

      return highestIntersection;
   }

   /**
    * Will return the intersection point between a line and a single planar region. If the line does
    * not intersect the region this method will return null.
    */
   public static Point3D intersectRegionWithLine(PlanarRegion region, Line3D projectionLine)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      region.getTransformToWorld(regionToWorld);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);
      planeNormal.applyTransform(regionToWorld);

      Point3D pointOnPlane = new Point3D();
      pointOnPlane.set(region.getConvexPolygon(0).getVertex(0));
      pointOnPlane.applyTransform(regionToWorld);

      Point3DReadOnly pointOnLine = projectionLine.getPoint();
      Vector3DReadOnly directionOfLine = projectionLine.getDirection();
      Point3D intersectionWithPlane = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, directionOfLine);
      if (intersectionWithPlane == null)
      {
         return null;
      }

      // checking convex hull here - might be better to check all polygons to avoid false positive
      if (isPointInWorldInsidePlanarRegion(region, intersectionWithPlane))
      {
         return intersectionWithPlane;
      }

      return null;
   }

   public static Point3D intersectRegionsWithRay(PlanarRegionsList regions, Point3D rayStart, Vector3D rayDirection)
   {
      double smallestDistance = Double.POSITIVE_INFINITY;
      Point3D closestIntersection = null;

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         Point3D intersection = intersectRegionWithRay(region, rayStart, rayDirection);
         if (intersection == null)
         {
            continue;
         }
         double distance = intersection.distance(rayStart);
         if (distance < smallestDistance)
         {
            smallestDistance = distance;
            closestIntersection = intersection;
         }
      }

      return closestIntersection;
   }

   public static Point3D intersectRegionWithRay(PlanarRegion region, Point3D rayStart, Vector3D rayDirection)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      region.getTransformToWorld(regionToWorld);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);
      planeNormal.applyTransform(regionToWorld);

      Point3D pointOnPlane = new Point3D();
      pointOnPlane.set(region.getConvexPolygon(0).getVertex(0));
      pointOnPlane.applyTransform(regionToWorld);

      Point3D intersectionWithPlane = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, rayStart, rayDirection);
      if (intersectionWithPlane == null)
      {
         return null;
      }

      Point3D intersectionInPlaneFrame = new Point3D(intersectionWithPlane);
      intersectionInPlaneFrame.applyInverseTransform(regionToWorld);
      // checking convex hull here - might be better to check all polygons to avoid false positive
      if (!region.getConvexHull().isPointInside(intersectionInPlaneFrame.getX(), intersectionInPlaneFrame.getY()))
      {
         return null;
      }

      Vector3D rayToIntersection = new Vector3D();
      rayToIntersection.sub(intersectionWithPlane, rayStart);
      if (rayToIntersection.dot(rayDirection) < 0.0)
      {
         return null;
      }

      return intersectionWithPlane;
   }

   public static boolean isPointOnRegion(PlanarRegion region, Point3D point, double epsilon)
   {
      Point3D closestPoint = closestPointOnPlane(point, region);
      return closestPoint.epsilonEquals(point, epsilon);
   }

   public static NavigableRegion getNavigableRegionContainingThisPoint(Point3DReadOnly point, List<NavigableRegion> navigableRegions)
   {
      return getNavigableRegionContainingThisPoint(point, navigableRegions, 0.0);
   }

   public static NavigableRegion getNavigableRegionContainingThisPoint(Point3DReadOnly point, List<NavigableRegion> navigableRegions, double epsilon)
   {
      List<NavigableRegion> containers = new ArrayList<>();

      for (NavigableRegion navigableRegion : navigableRegions)
      {
         if (isPointInWorldInsidePlanarRegion(navigableRegion.getHomeRegion(), point, epsilon))
         {
            containers.add(navigableRegion);
         }
      }

      if (containers.isEmpty())
         return null;
      if (containers.size() == 1)
         return containers.get(0);

      Point3D pointOnRegion = new Point3D();
      Vector3D regionNormal = new Vector3D();

      NavigableRegion closestContainer = containers.get(0);
      closestContainer.getHomeRegion().getNormal(regionNormal);
      closestContainer.getHomeRegion().getPointInRegion(pointOnRegion);
      double minDistance = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnRegion, regionNormal);

      for (int i = 1; i < containers.size(); i++)
      {
         NavigableRegion candidate = containers.get(i);
         candidate.getHomeRegion().getNormal(regionNormal);
         candidate.getHomeRegion().getPointInRegion(pointOnRegion);
         double distance = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnRegion, regionNormal);
         if (distance < minDistance)
         {
            closestContainer = candidate;
            minDistance = distance;
         }
      }

      return closestContainer;
   }

   public static boolean isPointInWorldInsidePlanarRegion(PlanarRegion planarRegion, Point3DReadOnly pointInWorldToCheck)
   {
      return isPointInLocalInsidePlanarRegion(planarRegion, pointInWorldToCheck, 0.0);
   }

   public static boolean isPointInWorldInsidePlanarRegion(PlanarRegion planarRegion, Point3DReadOnly pointInWorldToCheck, double epsilon)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);
      Point3D pointInLocalToCheck = new Point3D();
      transformToWorld.inverseTransform(pointInWorldToCheck, pointInLocalToCheck);
      return isPointInLocalInsidePlanarRegion(planarRegion, pointInLocalToCheck, epsilon);
   }

   public static boolean isPointInLocalInsidePlanarRegion(PlanarRegion planarRegion, Point3DReadOnly pointInLocalToCheck)
   {
      return isPointInLocalInsidePlanarRegion(planarRegion, new Point2D(pointInLocalToCheck));
   }

   public static boolean isPointInLocalInsidePlanarRegion(PlanarRegion planarRegion, Point3DReadOnly pointInLocalToCheck, double epsilon)
   {
      return isPointInLocalInsidePlanarRegion(planarRegion, new Point2D(pointInLocalToCheck), epsilon);
   }

   public static boolean isPointInLocalInsidePlanarRegion(PlanarRegion planarRegion, Point2DReadOnly pointInLocalToCheck)
   {
      return isPointInLocalInsidePlanarRegion(planarRegion, pointInLocalToCheck, 0.0);
   }

   public static boolean isPointInLocalInsidePlanarRegion(PlanarRegion planarRegion, Point2DReadOnly pointInLocalToCheck, double epsilon)
   {
      ConvexPolygon2D convexHull = planarRegion.getConvexHull();
      BoundingBox2D boundingBox = convexHull.getBoundingBox();

      if (!boundingBox.isInsideEpsilon(pointInLocalToCheck, epsilon))
         return false;
      if (!convexHull.isPointInside(pointInLocalToCheck, epsilon))
         return false;
      if (MathTools.epsilonEquals(0.0, epsilon, 1.0e-10))
      {
         return isPointInsidePolygon(planarRegion.getConcaveHull(), pointInLocalToCheck);
      }
      else
      {
         double[] epsilons = new double[planarRegion.getConcaveHullSize()];
         Arrays.fill(epsilons, epsilon);
         List<Point2D> concaveHull = ClusterTools.extrudePolygon(true, Arrays.asList(planarRegion.getConcaveHull()), epsilons);
         return isPointInsidePolygon(concaveHull, pointInLocalToCheck);
      }
   }

   public static boolean isPointInsidePolygon(Point2DReadOnly[] polygon, Point2DReadOnly pointToCheck)
   {
      return isPointInsidePolygon(Arrays.asList(polygon), pointToCheck);
   }

   public static boolean isPointInsidePolygon(List<? extends Point2DReadOnly> polygon, Point2DReadOnly pointToCheck)
   {
      if (polygon.size() < 3)
      {
         return false;
      }

      Point2DReadOnly rayOrigin = pointToCheck;
      Vector2D rayDirection = new Vector2D();

      Point2D pointOnArbitraryEdge = new Point2D();

      for (int i = 0; i < polygon.size(); i++)
      { // Picking an edge that is not parallel to the ray.
         Point2DReadOnly edgeStart = polygon.get(i);
         Point2DReadOnly edgeEnd = ListWrappingIndexTools.getNext(i, polygon);
         Vector2D edgeDirection = new Vector2D();
         edgeDirection.sub(edgeEnd, edgeStart);

         pointOnArbitraryEdge.interpolate(edgeStart, edgeEnd, 0.5);
         rayDirection.sub(pointOnArbitraryEdge, rayOrigin);

         double cross = edgeDirection.cross(rayDirection);

         if (Math.abs(cross) > 1.0e-3)
            break;
      }

      int numberOfIntersections = 0;

      Point2D previousIntersection = null;
      Point2D currentIntersection = null;

      for (int i = 0; i < polygon.size(); i++)
      {
         Point2DReadOnly edgeStart = polygon.get(i);
         Point2DReadOnly edgeEnd = ListWrappingIndexTools.getNext(i, polygon);

         currentIntersection = VisibilityGraphsGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, edgeStart, edgeEnd);

         if (currentIntersection != null)
         { // There is an intersection
            if (previousIntersection == null || !currentIntersection.epsilonEquals(previousIntersection, 1.0e-10))
            { // Because the intersection is different from the previous, the intersection is not on a vertex.
               numberOfIntersections++;
            }
         }

         previousIntersection = currentIntersection;
      }

      if (numberOfIntersections == 0)
      {
         //Could be both outside or inside
         return false;
      }

      // If the number of intersections is odd, the point is inside.
      return numberOfIntersections % 2 != 0;
   }

   public static boolean areBothPointsInsidePlanarRegion(Point2DReadOnly point1, Point2DReadOnly point2, PlanarRegion homeRegion)
   {
      if (!PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, point1))
         return false;
      if (!PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, point2))
         return false;

      return true;
   }

   /**
    * Check if the projection of at least one vertex of {@code regionA} is inside {@code regionB}.
    * <p>
    * The sign of {@code epsilon} is equivalent to performing the test against {@code regionB}
    * shrunk by {@code Math.abs(epsilon)} if {@code epsilon < 0.0}, or against the {@code regionB}
    * enlarged by {@code epsilon} if {@code epsilon > 0.0}.
    * </p>
    * 
    * @param regionA the query. Not modified.
    * @param regionB the reference. Not modified.
    * @param epsilon the tolerance to use during the test.
    * @return {@code true} if region A is at least partially above or below region B, {@code false}
    *         otherwise.
    */
   public static boolean isRegionAOverlapingWithRegionB(PlanarRegion regionA, PlanarRegion regionB, double epsilon)
   {
      RigidBodyTransform transformFromBToWorld = new RigidBodyTransform();
      regionB.getTransformToWorld(transformFromBToWorld);
      RigidBodyTransform transformFromAToB = new RigidBodyTransform();
      regionA.getTransformToWorld(transformFromAToB);
      transformFromAToB.preMultiplyInvertOther(transformFromBToWorld);

      ConvexPolygon2D convexHullB = regionB.getConvexHull();

      for (int i = 0; i < regionA.getConvexHull().getNumberOfVertices(); i++)
      {
         Point3D vertexA3D = new Point3D(regionA.getConvexHull().getVertex(i));
         transformFromAToB.transform(vertexA3D);
         Point2D vertexA2D = new Point2D(vertexA3D);
         if (convexHullB.getBoundingBox().isInsideEpsilon(vertexA2D, epsilon) && convexHullB.isPointInside(vertexA2D, epsilon))
            return true;
      }

      return false;
   }

   /**
    * From the local coordinates of the {@code regionB}, this method computes and return the minimum
    * z-coordinate among the vertices of {@code regionA}'s concave hull.
    * 
    * @param regionA the query. Not modified.
    * @param regionB the reference. Not modified.
    * @return the height of the lowest vertex of {@code regionA} above {@code regionB}. The returned
    *         value is negative if the lowest vertex is below {@code regionB}.
    */
   public static double computeMinHeightOfRegionAAboveRegionB(PlanarRegion regionA, PlanarRegion regionB)
   {
      RigidBodyTransform transformFromBToWorld = new RigidBodyTransform();
      regionB.getTransformToWorld(transformFromBToWorld);
      RigidBodyTransform transformFromAToB = new RigidBodyTransform();
      regionA.getTransformToWorld(transformFromAToB);
      transformFromAToB.preMultiplyInvertOther(transformFromBToWorld);

      double minZ = Double.POSITIVE_INFINITY;

      for (int i = 0; i < regionA.getConvexHull().getNumberOfVertices(); i++)
      {
         Point3D vertexA3D = new Point3D(regionA.getConcaveHull()[i]);
         transformFromAToB.transform(vertexA3D);
         minZ = Math.min(minZ, vertexA3D.getZ());
      }

      return minZ;
   }

   public static List<PlanarRegion> ensureClockwiseOrder(List<PlanarRegion> planarRegions)
   {
      List<PlanarRegion> copies = new ArrayList<>(planarRegions.size());

      for (PlanarRegion planarRegion : planarRegions)
      {
         PlanarRegion copy = planarRegion.copy();
         List<Point2DReadOnly> concaveHullVertices = Arrays.asList(copy.getConcaveHull());
         ConcaveHullTools.ensureClockwiseOrdering(concaveHullVertices);
         copies.add(copy);
      }

      return copies;
   }

   public static List<PlanarRegion> filterPlanarRegionsByHullSize(int minNumberOfVertices, List<PlanarRegion> planarRegions)
   {
      if (minNumberOfVertices <= 0)
         return planarRegions;

      return planarRegions.stream().filter(region -> region.getConcaveHull().length >= minNumberOfVertices).collect(Collectors.toList());
   }

   public static List<PlanarRegion> filterPlanarRegionsByArea(double minArea, List<PlanarRegion> planarRegions)
   {
      if (!Double.isFinite(minArea) || minArea <= 0.0)
         return planarRegions;

      return planarRegions.stream().filter(region -> computePlanarRegionArea(region) >= minArea).collect(Collectors.toList());
   }

   public static List<PlanarRegion> filterPlanarRegionsWithBoundingCapsule(Point3DReadOnly capsuleStart, Point3DReadOnly capsuleEnd, double capsuleRadius,
                                                                           List<PlanarRegion> planarRegions)
   {
      return filterPlanarRegionsWithBoundingCapsule(new LineSegment3D(capsuleStart, capsuleEnd), capsuleRadius, planarRegions);
   }

   public static List<PlanarRegion> filterPlanarRegionsWithBoundingCapsule(LineSegment3D capsuleSegment, double capsuleRadius, List<PlanarRegion> planarRegions)
   {
      if (!Double.isFinite(capsuleRadius) || capsuleRadius < 0.0)
         return planarRegions;

      return planarRegions.stream().filter(region -> isPlanarRegionIntersectingWithCapsule(capsuleSegment, capsuleRadius, region)).collect(Collectors.toList());
   }

   public static double computePlanarRegionArea(PlanarRegion planarRegion)
   {
      double area = 0.0;
      for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
      {
         area += planarRegion.getConvexPolygon(i).getArea();
      }
      return area;
   }

   public static boolean isPlanarRegionIntersectingWithCapsule(LineSegment3D capsuleSegment, double capsuleRadius, PlanarRegion query)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      query.getTransformToWorld(transformToWorld);

      return Arrays.stream(query.getConcaveHull()).map(vertex -> applyTransform(transformToWorld, vertex))
                   .anyMatch(vertex -> capsuleSegment.distance(vertex) <= capsuleRadius);
   }

   private static Point3D applyTransform(RigidBodyTransform transform, Point2D point2D)
   {
      Point3D point3D = new Point3D(point2D);
      transform.transform(point3D);
      return point3D;
   }

   public static List<PlanarRegion> filterRegionsByTruncatingVerticesBeneathHomeRegion(List<PlanarRegion> regionsToCheck, PlanarRegion homeRegion,
                                                                                       double depthThresholdForConvexDecomposition, PlanarRegionFilter filter)
   {
      List<PlanarRegion> filteredList = new ArrayList<>();
      Point3D pointOnPlane = new Point3D();
      Vector3D planeNormal = new Vector3D();

      homeRegion.getPointInRegion(pointOnPlane);
      homeRegion.getNormal(planeNormal);

      for (PlanarRegion regionToCheck : regionsToCheck)
      {
         PlanarRegion truncatedPlanarRegion = truncatePlanarRegionIfIntersectingWithPlane(pointOnPlane, planeNormal, regionToCheck,
                                                                                          depthThresholdForConvexDecomposition, filter);
         if (truncatedPlanarRegion != null)
            filteredList.add(truncatedPlanarRegion);
      }

      return filteredList;
   }

   /**
    * Truncate the given planar region {@code planarRegionToTuncate} with the plane such that only
    * the part that is <b>above</b> the plane remains.
    * 
    * @param pointOnPlane a point on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @param planarRegionToTruncate the original planar region to be truncated. Not modified.
    * @param depthThresholdForConvexDecomposition used to recompute the convex decomposition of the
    *           planar region when it has been truncated.
    * @param filter the filter used to determine if the truncated region is to be created.
    * @return the truncated planar region which is completely above the plane, or {@code null} if
    *         the given planar region is completely underneath the plane or if it is too small
    *         according to {@code minTruncatedSize} and {@code minTruncatedArea}.
    */
   public static PlanarRegion truncatePlanarRegionIfIntersectingWithPlane(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal,
                                                                          PlanarRegion planarRegionToTruncate, double depthThresholdForConvexDecomposition,
                                                                          PlanarRegionFilter filter)
   {
      Point3D pointOnRegion = new Point3D();
      Vector3D regionNormal = new Vector3D();
      planarRegionToTruncate.getPointInRegion(pointOnRegion);
      planarRegionToTruncate.getNormal(regionNormal);

      if (EuclidGeometryTools.areVector3DsParallel(planeNormal, regionNormal, Math.toRadians(3.0)))
      { // The region and the plane are parallel, check which one is above the other.
         double signedDistance = signedDistanceFromPoint3DToPlane3D(pointOnRegion, pointOnPlane, planeNormal);
         if (signedDistance < 0.0)
            return null; // The region is underneath
         else
            return planarRegionToTruncate; // The region is above
      }

      Point3D pointOnPlaneInRegionFrame = new Point3D(pointOnPlane);
      Vector3D planeNormalInRegionFrame = new Vector3D(planeNormal);

      RigidBodyTransform transformFromRegionToWorld = new RigidBodyTransform();
      planarRegionToTruncate.getTransformToWorld(transformFromRegionToWorld);
      pointOnPlaneInRegionFrame.applyInverseTransform(transformFromRegionToWorld);
      planeNormalInRegionFrame.applyInverseTransform(transformFromRegionToWorld);

      Point2DReadOnly vertex2D = planarRegionToTruncate.getConcaveHullVertex(planarRegionToTruncate.getConcaveHullSize() - 1);
      Point3D vertex3D = new Point3D(vertex2D);
      double previousSignedDistance = signedDistanceFromPoint3DToPlane3D(vertex3D, pointOnPlaneInRegionFrame, planeNormalInRegionFrame);

      Point3D previousVertex3D = vertex3D;

      List<Point2D> truncatedConcaveHullVertices = new ArrayList<>();

      boolean isRegionEntirelyAbove = true;
      double epsilonDistance = 1.0e-10;

      for (int i = 0; i < planarRegionToTruncate.getConcaveHullSize(); i++)
      {
         vertex2D = planarRegionToTruncate.getConcaveHullVertex(i);
         vertex3D = new Point3D(vertex2D);

         double signedDistance = signedDistanceFromPoint3DToPlane3D(vertex3D, pointOnPlaneInRegionFrame, planeNormalInRegionFrame);
         isRegionEntirelyAbove &= signedDistance >= -epsilonDistance;

         if (signedDistance * previousSignedDistance < 0.0)
         {
            if (Math.abs(signedDistance) <= epsilonDistance)
            {
               truncatedConcaveHullVertices.add(new Point2D(vertex2D));
            }
            else if (Math.abs(previousSignedDistance) > epsilonDistance)
            {
               Vector3D edgeDirection = new Vector3D();
               edgeDirection.sub(vertex3D, previousVertex3D);
               Point3D intersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlaneInRegionFrame, planeNormalInRegionFrame,
                                                                                                     vertex3D, previousVertex3D);

               truncatedConcaveHullVertices.add(new Point2D(intersection));
            }
         }

         if (signedDistance >= -epsilonDistance)
         {
            truncatedConcaveHullVertices.add(new Point2D(vertex2D));
         }

         previousVertex3D = vertex3D;
         previousSignedDistance = signedDistance;
      }

      if (isRegionEntirelyAbove)
         return planarRegionToTruncate;

      if (truncatedConcaveHullVertices.isEmpty())
         return null; // The region is completely underneath

      List<ConvexPolygon2D> truncatedConvexPolygons = new ArrayList<>();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(new ArrayList<>(truncatedConcaveHullVertices), depthThresholdForConvexDecomposition,
                                                                 truncatedConvexPolygons);

      Point2D[] concaveHullVertices = new Point2D[truncatedConcaveHullVertices.size()];
      truncatedConcaveHullVertices.toArray(concaveHullVertices);
      PlanarRegion truncatedRegion = new PlanarRegion(transformFromRegionToWorld, concaveHullVertices, truncatedConvexPolygons);
      truncatedRegion.setRegionId(planarRegionToTruncate.getRegionId());
      if (filter == null || filter.isPlanarRegionRelevant(truncatedRegion))
         return truncatedRegion;
      else
         return null;
   }
}
