package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionTools
{
   private static final boolean debug = false;

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
    * Projects the given point onto the planar region, returning the closest point on
    * the region to the provided point.
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
    * Projects the given point onto a planar region from the list. The projection is done along
    * the z axis in world frame and if there is multiple regions that the point can be
    * projected onto, the highest intersection point will be returned.
    * <p>
    * Will return null if the is no planar region above or below the point.
    */
   public static Point3D projectPointToPlanesVertically(Point3DReadOnly point, PlanarRegionsList regions)
   {
      Line3D projectionLine = new Line3D(point, new Vector3D(0.0, 0.0, 1.0));
      Point3D highestIntersection = null;
      PlanarRegion highestRegion = null;

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         Point3D intersection = intersectRegionWithLine(region, projectionLine);

         if (intersection == null)
         {
            continue;
         }

         if (highestIntersection == null || highestIntersection.getZ() < intersection.getZ())
         {
            highestIntersection = intersection;
            highestRegion = region;
         }
      }

      return highestIntersection;
   }

   /**
    * Will return the intersection point between a line and a single planar region. If
    * the line does not intersect the region this method will return null.
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
      if (isPointInWorldInsideARegion(region, intersectionWithPlane))
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

   public static boolean isPointInsideAnyRegion(List<PlanarRegion> regions, Point3DReadOnly pointToCheck)
   {
      for (PlanarRegion region : regions)
      {
         if (isPointInWorldInsideARegion(region, pointToCheck))
         {
            return true;
         }
      }

      return false;
   }

   public static boolean isPointInWorldInsideARegion(PlanarRegion region, Point3DReadOnly pointInWorldToCheck)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      region.getTransformToWorld(transformToWorld);
      Point3D pointInLocalToCheck = new Point3D();
      transformToWorld.inverseTransform(pointInWorldToCheck, pointInLocalToCheck);
      return isPointInLocalInsideARegion(region, pointInLocalToCheck);
   }

   public static boolean isPointInLocalInsideARegion(PlanarRegion region, Point3DReadOnly pointInLocalToCheck)
   {
      Point2D[] pointsInConcaveHull = region.getConcaveHull();

      Point2D centroid = EuclidGeometryTools.averagePoint2Ds(Arrays.asList(pointsInConcaveHull));

      Vector2D directionToCentroid = new Vector2D(centroid.getX() - pointInLocalToCheck.getX(), centroid.getY() - pointInLocalToCheck.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      Point2D endPoint = new Point2D(pointInLocalToCheck.getX() + directionToCentroid.getX(), pointInLocalToCheck.getY() + directionToCentroid.getY());

      boolean pointIsInside = isPointInsidePolygon(pointsInConcaveHull, new Point2D(pointInLocalToCheck), endPoint);

      if (pointIsInside)
      {
         return true;
      }

      return false;
   }

   // FIXME This is flaky when the line (pointToCheck, lineEnd) goes through vertices of the polygon.
   public static boolean isPointInsidePolygon(Point2D[] polygon, Point2D pointToCheck, Point2D lineEnd)
   {
      int index = 0;

      Point2D[] points = new Point2D[polygon.length + 1];

      for (int i = 0; i < polygon.length; i++)
      {
         points[i] = polygon[i];
      }
      points[points.length - 1] = points[0];

      for (int i = 1; i < points.length; i++)
      {
         Point2D point1 = points[i - 1];
         Point2D point2 = points[i];

         if (EuclidGeometryTools.doLineSegment2DsIntersect(point1, point2, pointToCheck, lineEnd))
         {
            index++;
         }
      }

      //      System.out.println("INDEX: " + index);

      if (index == 0)
      {
         //Could be both outside or inside
         return false;
      }

      if (index % 2 == 0)
      {
         return false;
      }
      else
      {
         return true;
      }
   }

   public static boolean areBothPointsInsidePolygon(Point2D point1, Point2D point2, PlanarRegion homeRegion)
   {
      ArrayList<Point2D> points = new ArrayList<>();
      for (int i = 1; i < homeRegion.getConcaveHullSize(); i++)
      {
         Point2D point = homeRegion.getConcaveHull()[i];
         points.add(point);
      }

      Point2D centroid = EuclidGeometryTools.averagePoint2Ds(points);

      Vector2D directionToCentroid = new Vector2D(centroid.getX() - point1.getX(), centroid.getY() - point1.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      Point2D endPoint = new Point2D(point1.getX() + directionToCentroid.getX(), point1.getY() + directionToCentroid.getY());

      boolean startIsInside = PlanarRegionTools.isPointInsidePolygon(homeRegion.getConcaveHull(), point1, endPoint);

      directionToCentroid = new Vector2D(centroid.getX() - point2.getX(), centroid.getY() - point2.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      endPoint = new Point2D(point2.getX() + directionToCentroid.getX(), point2.getY() + directionToCentroid.getY());

      boolean goalIsInside = PlanarRegionTools.isPointInsidePolygon(homeRegion.getConcaveHull(), point2, endPoint);

      if (startIsInside && goalIsInside)
      {
         return true;
      }
      return false;
   }

   public static boolean areBothPointsInsidePolygon(Point2D point1, Point2D point2, List<Point2D> pointsInPolygon)
   {
      Point2D centroid = EuclidGeometryTools.averagePoint2Ds(pointsInPolygon);

      Vector2D directionToCentroid = new Vector2D(centroid.getX() - point1.getX(), centroid.getY() - point1.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      Point2D endPoint = new Point2D(point1.getX() + directionToCentroid.getX(), point1.getY() + directionToCentroid.getY());

      Point2D[] pointsArr = pointsInPolygon.toArray(new Point2D[pointsInPolygon.size()]);
      boolean startIsInside = PlanarRegionTools.isPointInsidePolygon(pointsArr, point1, endPoint);

      directionToCentroid = new Vector2D(centroid.getX() - point2.getX(), centroid.getY() - point2.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      endPoint = new Point2D(point2.getX() + directionToCentroid.getX(), point2.getY() + directionToCentroid.getY());

      boolean goalIsInside = PlanarRegionTools.isPointInsidePolygon(pointsArr, point2, endPoint);

      if (startIsInside && goalIsInside)
      {
         return true;
      }
      return false;
   }

   public static boolean isPartOfTheRegionInside(PlanarRegion regionToCheck, PlanarRegion containingRegion)
   {
      ArrayList<Point3D> pointsToCalculateCentroid = new ArrayList<>();
      Point2D[] homePointsArr = new Point2D[containingRegion.getConvexHull().getNumberOfVertices()];
      for (int i = 0; i < containingRegion.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) containingRegion.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         containingRegion.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);
         Point3D transformedPt = fpt.getPoint();

         homePointsArr[i] = new Point2D(transformedPt.getX(), transformedPt.getY());
         pointsToCalculateCentroid.add(new Point3D(transformedPt.getX(), transformedPt.getY(), transformedPt.getZ()));
      }

      ConvexPolygon2D homeConvexPol = new ConvexPolygon2D(homePointsArr);
      homeConvexPol.update();

      Point3D centroidOfHomeRegion = PointCloudTools.getCentroid(pointsToCalculateCentroid);

      Vector3D normal = calculateNormal(containingRegion);

      for (int i = 0; i < regionToCheck.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) regionToCheck.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         regionToCheck.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);

         Point3D pointToProject = fpt.getPoint();
         Point3D projectedPointFromOtherRegion = new Point3D();

         //                     System.out.println(pointToProject + "  " + point3D + "  " + normal + "  " + projectedPointFromOtherRegion);
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPointFromOtherRegion);

         if (homeConvexPol.isPointInside(new Point2D(projectedPointFromOtherRegion.getX(), projectedPointFromOtherRegion.getY())))
         {
            return true;
         }
         //         }
      }

      return false;
   }

   public static Vector3D calculateNormal(PlanarRegion region)
   {
      Vector3D normal = new Vector3D();
      region.getNormal(normal);
      return normal;
   }

   public static ArrayList<PlanarRegion> determineWhichRegionsAreInside(PlanarRegion containingRegion, List<PlanarRegion> otherRegionsEx)
   {
      ArrayList<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();

      for (PlanarRegion otherRegion : otherRegionsEx)
      {
         if (PlanarRegionTools.isPartOfTheRegionInside(otherRegion, containingRegion))
         {
            regionsInsideHomeRegion.add(otherRegion);
         }
      }

      return regionsInsideHomeRegion;
   }

   public static boolean areAllPointsBelowTheRegion(PlanarRegion regionToCheck, PlanarRegion homeRegion)
   {
      for (int i = 0; i < homeRegion.getConcaveHull().length; i++)
      {
         Point2D point2D = (Point2D) homeRegion.getConcaveHull()[i];
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D homeRegionPoint = new FramePoint3D();
         homeRegionPoint.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         homeRegion.getTransformToWorld(transToWorld);
         homeRegionPoint.applyTransform(transToWorld);

         for (int j = 0; j < regionToCheck.getConcaveHull().length; j++)
         {
            Point2D point2D1 = (Point2D) regionToCheck.getConcaveHull()[j];
            Point3D point3D1 = new Point3D(point2D1.getX(), point2D1.getY(), 0);
            FramePoint3D otherRegionPoint = new FramePoint3D();
            otherRegionPoint.set(point3D1);
            RigidBodyTransform transToWorld1 = new RigidBodyTransform();
            regionToCheck.getTransformToWorld(transToWorld1);
            otherRegionPoint.applyTransform(transToWorld1);

            System.out.println(homeRegionPoint.getZ() + "   " + homeRegionPoint.getZ());
            if (homeRegionPoint.getZ() + 0.1 < otherRegionPoint.getZ())
            {
               return false;
            }
         }
      }

      return true;
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

   public static List<PlanarRegion> filterPlanarRegionsWithBoundingCapsule(Point3D capsuleStart, Point3D capsuleEnd, double capsuleRadius,
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

   public static Point3D projectPointToPlane(Point3DReadOnly pointToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = calculateNormal(regionToProjectTo);
      Point2D point2D = (Point2D) regionToProjectTo.getConvexHull().getVertex(0);
      Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);

      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal);
   }

   public static void classifyRegions(List<PlanarRegion> regionsToClassify, double zNormalThreshold, List<PlanarRegion> obstacleRegionsToPack,
                                       List<PlanarRegion> accessibleRegionsToPack)
   {
      Vector3D normal = new Vector3D();

      for (PlanarRegion region : regionsToClassify)
      {
         if (!region.isEmpty())
         {
            region.getNormal(normal);
            if (Math.abs(normal.getZ()) < zNormalThreshold)
            {
               obstacleRegionsToPack.add(region);
            }
            else
            {
               accessibleRegionsToPack.add(region);
            }
         }
      }
   }
   
   public static List<PlanarRegion> filterRegionsThatAreAboveHomeRegion(List<PlanarRegion> regionsToCheck, PlanarRegion homeRegion)
   {
      List<PlanarRegion> filteredList = new ArrayList<>();
      for (PlanarRegion region : regionsToCheck)
      {
         if (isRegionAboveHomeRegion(region, homeRegion))
         {
            filteredList.add(region);
         }
      }

      return filteredList;
   }
   
   public static boolean isRegionAboveHomeRegion(PlanarRegion regionToCheck, PlanarRegion homeRegion)
   {
      for (int i = 0; i < homeRegion.getConcaveHull().length; i++)
      {
         Point2D point2D = (Point2D) homeRegion.getConcaveHull()[i];
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D homeRegionPoint = new FramePoint3D();
         homeRegionPoint.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         homeRegion.getTransformToWorld(transToWorld);
         homeRegionPoint.applyTransform(transToWorld);

         for (int j = 0; j < regionToCheck.getConcaveHull().length; j++)
         {
            Point2D point2D1 = (Point2D) regionToCheck.getConcaveHull()[j];
            Point3D point3D1 = new Point3D(point2D1.getX(), point2D1.getY(), 0);
            FramePoint3D otherRegionPoint = new FramePoint3D();
            otherRegionPoint.set(point3D1);
            RigidBodyTransform transToWorld1 = new RigidBodyTransform();
            regionToCheck.getTransformToWorld(transToWorld1);
            otherRegionPoint.applyTransform(transToWorld1);

            if (homeRegionPoint.getZ() > otherRegionPoint.getZ())
            {
               //               System.out.println("Region is below home");
               return false;
            }
         }
      }
      //      System.out.println("Region is above home");
      return true;
   }
   
   public static boolean isRegionTooHighToStep(PlanarRegion regionToProject, PlanarRegion regionToProjectTo, double tooHighToStepThreshold)
   {
      Vector3D normal = PlanarRegionTools.calculateNormal(regionToProjectTo);

      for (int i = 0; i < regionToProject.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) regionToProject.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         regionToProject.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);

         Point3D pointToProject = fpt.getPoint();
         Point3D projectedPoint = new Point3D();
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPoint);

         if (pointToProject.distance(projectedPoint) >= tooHighToStepThreshold)
         {
            return true;
         }
      }

      return false;
   }

   public static boolean doRay2DAndLineSegment2DIntersect(Point2DReadOnly rayOrigin, Vector2D rayDirection, Point2DReadOnly lineSegmentStart,
                                                          Point2DReadOnly lineSegmentEnd)
   {
      return doRay2DAndLineSegment2DIntersect(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(), lineSegmentStart.getX(),
                                              lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY());
   }

   public static boolean doRay2DAndLineSegment2DIntersect(double rayOriginX, double rayOriginY, double rayDirectionX, double rayDirectionY,
                                                          double lineSegmentStartX, double lineSegmentStartY, double lineSegmentEndX, double lineSegmentEndY)
   {
      double epsilon = 1.0e-7;

      double lineSegmentDirectionX = lineSegmentEndX - lineSegmentStartX;
      double lineSegmentDirectionY = lineSegmentEndY - lineSegmentStartY;

      double determinant = -rayDirectionX * lineSegmentDirectionY + rayDirectionY * lineSegmentDirectionX;

      double dx = lineSegmentStartX - rayOriginX;
      double dy = lineSegmentStartY - rayOriginY;

      if (Math.abs(determinant) < epsilon)
      { // The ray and line segment are parallel
        // Check if they are collinear
         double cross = dx * rayDirectionY - dy * rayDirectionX;
         if (Math.abs(cross) < epsilon)
         {
            if (EuclidGeometryTools.isPoint2DInFrontOfRay2D(lineSegmentStartX, lineSegmentStartY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY))
               return true;
            if (EuclidGeometryTools.isPoint2DInFrontOfRay2D(lineSegmentEndX, lineSegmentEndY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY))
               return true;
            return false;
         }
         // The line segments are parallel but are not collinear, they do not intersect
         else
         {
            return false;
         }
      }

      double oneOverDeterminant = 1.0 / determinant;
      double AInverse00 = -lineSegmentDirectionY;
      double AInverse01 = lineSegmentDirectionX;
      double AInverse10 = -rayDirectionY;
      double AInverse11 = rayDirectionX;

      double alpha = oneOverDeterminant * (AInverse00 * dx + AInverse01 * dy);
      double beta = oneOverDeterminant * (AInverse10 * dx + AInverse11 * dy);

      return alpha > 0.0 - epsilon && 0.0 - epsilon < beta && beta < 1.0 + epsilon;
   }

   public static Point2D intersectionBetweenRay2DAndLineSegment2D(Point2DReadOnly rayOrigin, Vector2D rayDirection, Point2DReadOnly lineSegmentStart,
                                                                  Point2DReadOnly lineSegmentEnd)
   {
      Point2D intersection = new Point2D();
      boolean success = intersectionBetweenRay2DAndLineSegment2D(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(),
                                                                 lineSegmentStart.getX(), lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY(),
                                                                 intersection);
      if (success)
         return intersection;
      else
         return null;
   }

   public static boolean intersectionBetweenRay2DAndLineSegment2D(Point2DReadOnly rayOrigin, Vector2D rayDirection, Point2DReadOnly lineSegmentStart,
                                                                  Point2DReadOnly lineSegmentEnd, Point2DBasics intersectionToPack)
   {
      return intersectionBetweenRay2DAndLineSegment2D(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(), lineSegmentStart.getX(),
                                                      lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY(), intersectionToPack);
   }

   public static boolean intersectionBetweenRay2DAndLineSegment2D(double rayOriginX, double rayOriginY, double rayDirectionX, double rayDirectionY,
                                                                  double lineSegmentStartX, double lineSegmentStartY, double lineSegmentEndX,
                                                                  double lineSegmentEndY, Point2DBasics intersectionToPack)
   {
      if (!doRay2DAndLineSegment2DIntersect(rayOriginX, rayOriginY, rayDirectionX, rayDirectionY, lineSegmentStartX, lineSegmentStartY, lineSegmentEndX,
                                            lineSegmentEndY))
      {
         if (intersectionToPack != null)
            intersectionToPack.setToNaN();
         return false;
      }

      double epsilon = 1.0e-10;

      double lineSegmentDirectionx = lineSegmentEndX - lineSegmentStartX;
      double lineSegmentDirectiony = lineSegmentEndY - lineSegmentStartY;

      if (Math.abs(-rayDirectionX * lineSegmentDirectiony + rayDirectionY * lineSegmentDirectionx) > epsilon)
      { // The ray and line segment are not parallel and are intersecting, same as finding the intersection of two lines.
         double pointOnLine1x = rayOriginX;
         double pointOnLine1y = rayOriginY;
         double pointOnLine2x = lineSegmentStartX;
         double pointOnLine2y = lineSegmentStartY;
         return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x, pointOnLine1y, rayDirectionX, rayDirectionY, pointOnLine2x, pointOnLine2y,
                                                                  lineSegmentDirectionx, lineSegmentDirectiony, intersectionToPack);
      }
      else
      { // The ray and line segment are parallel and intersecting, they must be overlapping.
        // Let's first check for a common endpoint
        // Let's find the first endpoint that is inside the other line segment and return it.
         if (EuclidGeometryTools.isPoint2DInFrontOfRay2D(lineSegmentStartX, lineSegmentStartY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY))
         {
            intersectionToPack.set(lineSegmentStartX, lineSegmentStartY);
            return true;
         }

         if (EuclidGeometryTools.isPoint2DInFrontOfRay2D(lineSegmentEndX, lineSegmentEndY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY))
         {
            intersectionToPack.set(lineSegmentEndX, lineSegmentEndY);
            return true;
         }

         // There is some inconsistency between doRay2DAndLineSegment2DIntersect and this method, crashing.
         throw new RuntimeException("Unexpected state.");
      }
   }
}
