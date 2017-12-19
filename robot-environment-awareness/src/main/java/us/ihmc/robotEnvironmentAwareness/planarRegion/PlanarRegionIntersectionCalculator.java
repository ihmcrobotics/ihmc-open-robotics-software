package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment1D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class PlanarRegionIntersectionCalculator
{
   public static List<LineSegment3D> computeIntersections(List<PlanarRegionSegmentationRawData> rawData, IntersectionEstimationParameters parameters)
   {
      List<LineSegment3D> allIntersections = new ArrayList<>();
      Map<List<LineSegment3D>, Pair<PlanarRegionSegmentationRawData, PlanarRegionSegmentationRawData>> intersectionsToRegionsMap = new HashMap<>();

      for (int i = 0; i < rawData.size(); i++)
      {
         PlanarRegionSegmentationRawData currentRegion = rawData.get(i);

         if (currentRegion.size() < parameters.getMinRegionSize())
            continue;

         for (int j = i + 1; j < rawData.size(); j++)
         {
            PlanarRegionSegmentationRawData currentNeighbor = rawData.get(j);

            if (currentNeighbor.size() < parameters.getMinRegionSize())
               continue;

            double minRegionAngleDifference = parameters.getMinRegionAngleDifference();

            Line3D intersectionLine = computeIntersectionLine3d(currentRegion, currentNeighbor, minRegionAngleDifference);
            if (intersectionLine == null)
               continue;

            double maxDistanceToRegion = parameters.getMaxDistanceToRegion();
            double minIntersectionLength = parameters.getMinIntersectionLength();

            List<LineSegment3D> intersectionList = findIntersectionEndPoints(currentRegion, currentNeighbor, maxDistanceToRegion, minIntersectionLength,
                                                                             intersectionLine);

            if (intersectionList != null)
            {
               allIntersections.addAll(intersectionList);
               intersectionsToRegionsMap.put(intersectionList, ImmutablePair.of(currentRegion, currentNeighbor));
            }
         }
      }

      extendLinesToIntersection(allIntersections);

      if (parameters.isAddIntersectionsToRegions())
      {
         for (Entry<List<LineSegment3D>, Pair<PlanarRegionSegmentationRawData, PlanarRegionSegmentationRawData>> entry : intersectionsToRegionsMap.entrySet())
         {
            List<LineSegment3D> intersections = entry.getKey();
            PlanarRegionSegmentationRawData region1 = entry.getValue().getLeft();
            PlanarRegionSegmentationRawData region2 = entry.getValue().getRight();

            List<LineSegment2D> intersectionsForRegion1 = PolygonizerTools.toLineSegmentsInPlane(intersections, region1.getOrigin(), region1.getNormal());
            region1.addIntersections(intersectionsForRegion1);
            List<LineSegment2D> intersectionsForRegion2 = PolygonizerTools.toLineSegmentsInPlane(intersections, region2.getOrigin(), region2.getNormal());
            region2.addIntersections(intersectionsForRegion2);
         }
      }

      return allIntersections;
   }

   public static void extendLinesToIntersection(List<LineSegment3D> allIntersections)
   {
      Point3D closestPointOnCurrentLine = new Point3D();
      Point3D closestPointOnOtherLine = new Point3D();

      for (int i = 0; i < allIntersections.size(); i++)
      {
         LineSegment3D currentIntersectionSegment = allIntersections.get(i);
         Line3D currentIntersectionLine = new Line3D(currentIntersectionSegment.getLine());

         for (int j = i + 1; j < allIntersections.size(); j++)
         {
            LineSegment3D otherIntersectionSegment = allIntersections.get(j);
            Line3D otherIntersectionLine = new Line3D(otherIntersectionSegment.getLine());

            double distanceBetweenLines = currentIntersectionLine.closestPointsWith(otherIntersectionLine, closestPointOnCurrentLine, closestPointOnOtherLine);

            if (distanceBetweenLines > 0.05)
               continue;

            if (currentIntersectionSegment.distance(closestPointOnCurrentLine) <= 0.05 && otherIntersectionSegment.distance(closestPointOnOtherLine) <= 0.05)
            {
               double alphaCurrent = currentIntersectionSegment.percentageAlongLineSegment(closestPointOnCurrentLine);
               if (alphaCurrent < 0.0)
                  currentIntersectionSegment.setFirstEndpoint(closestPointOnCurrentLine);
               else if (alphaCurrent > 1.0)
                  currentIntersectionSegment.setSecondEndpoint(closestPointOnCurrentLine);

               double alphaOther = otherIntersectionSegment.percentageAlongLineSegment(closestPointOnOtherLine);
               if (alphaOther < 0.0)
                  otherIntersectionSegment.setFirstEndpoint(closestPointOnOtherLine);
               else if (alphaOther > 1.0)
                  otherIntersectionSegment.setSecondEndpoint(closestPointOnOtherLine);
            }
         }
      }
   }

   private static Line3D computeIntersectionLine3d(PlanarRegionSegmentationRawData currentRegion, PlanarRegionSegmentationRawData currentNeighbor,
                                                   double minRegionAngleDifference)
   {
      Point3D origin1 = currentRegion.getOrigin();
      Vector3D normal1 = currentRegion.getNormal();
      Point3D origin2 = currentNeighbor.getOrigin();
      Vector3D normal2 = currentNeighbor.getNormal();

      Point3D pointOnIntersection = new Point3D();
      Vector3D intersectionDirection = new Vector3D();
      boolean success = EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(origin1, normal1, origin2, normal2, minRegionAngleDifference, pointOnIntersection,
                                                                           intersectionDirection);
      if (!success)
         return null;
      else
         return new Line3D(pointOnIntersection, intersectionDirection);
   }

   private static List<LineSegment3D> findIntersectionEndPoints(PlanarRegionSegmentationRawData currentRegion, PlanarRegionSegmentationRawData currentNeighbor,
                                                                double maxDistance, double minIntersectionLength, Line3D intersectionLine)
   {

      List<LineSegment1D> intersectionsFromRegion1 = findIntersectionLineSegments(currentRegion, maxDistance, minIntersectionLength, intersectionLine);
      if (intersectionsFromRegion1 == null || intersectionsFromRegion1.isEmpty())
         return null;
      List<LineSegment1D> intersectionsFromRegion2 = findIntersectionLineSegments(currentNeighbor, maxDistance, minIntersectionLength, intersectionLine);
      if (intersectionsFromRegion2 == null || intersectionsFromRegion2.isEmpty())
         return null;

      List<LineSegment3D> intersections = new ArrayList<>();

      for (LineSegment1D intersectionFromRegion1 : intersectionsFromRegion1)
      {
         for (LineSegment1D intersectionFromRegion2 : intersectionsFromRegion2)
         {
            LineSegment1D overlap = intersectionFromRegion1.computeOverlap(intersectionFromRegion2);
            if (overlap != null && overlap.length() > minIntersectionLength)
               intersections.add(overlap.toLineSegment3d(intersectionLine));
         }
      }

      return intersections.isEmpty() ? null : intersections;
   }

   /**
    * 
    * @param currentRegion
    * @param maxDistance
    * @param intersectionPoint
    * @param intersectionDirection
    * @return
    */
   private static List<LineSegment1D> findIntersectionLineSegments(PlanarRegionSegmentationRawData currentRegion, double maxDistance,
                                                                   double minIntersectionLength, Line3D intersectionLine)
   {
      Vector3D perpendicularToDirection = new Vector3D();
      perpendicularToDirection.cross(currentRegion.getNormal(), intersectionLine.getDirection());
      perpendicularToDirection.normalize();

      Vector3D distance = new Vector3D();
      Point3D regionPoint = new Point3D();

      // 1-D Coordinates along the intersection direction of all the region points that are close enough to the intersection.
      // By using a PriorityQueue the coordinates are sorted.
      PriorityQueue<Double> points1D = new PriorityQueue<>();

      for (int i = 0; i < currentRegion.size(); i++)
      {
         currentRegion.getPoint(i, regionPoint);
         distance.sub(regionPoint, intersectionLine.getPoint());

         double orthogonalDistanceFromLine = Math.abs(distance.dot(perpendicularToDirection));
         if (orthogonalDistanceFromLine <= maxDistance)
            points1D.add(distance.dot(intersectionLine.getDirection()));
      }

      if (points1D.size() < 2)
         return null;

      List<LineSegment1D> intersectionSegments = new ArrayList<>();

      double firstEndpoint = points1D.poll();
      double secondEndpoint = Double.NaN;

      double lastPoint1D = firstEndpoint;

      while (!points1D.isEmpty())
      {
         double currentPoint1D = points1D.poll();

         if (Math.abs(currentPoint1D - lastPoint1D) < maxDistance)
         { // The current point is close enough to the previous, we extend the current segment
            secondEndpoint = currentPoint1D;
            if (points1D.isEmpty() && Math.abs(secondEndpoint - firstEndpoint) >= minIntersectionLength)
               intersectionSegments.add(new LineSegment1D(firstEndpoint, secondEndpoint));
         }
         else
         { // The current point is too far from the previous, end of the current segment
              // If there is not secondEndpoint, that means the firstEndpoint is isolated => not an intersection.
            if (!Double.isNaN(secondEndpoint) || Math.abs(secondEndpoint - firstEndpoint) >= minIntersectionLength)
               intersectionSegments.add(new LineSegment1D(firstEndpoint, secondEndpoint));

            if (points1D.size() < 2)
               break;

            // Beginning of a new segment
            firstEndpoint = currentPoint1D;
            secondEndpoint = Double.NaN; // Serve to detect isolated point.
         }
         lastPoint1D = currentPoint1D;
      }

      return intersectionSegments.isEmpty() ? null : intersectionSegments;
   }
}
