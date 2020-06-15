package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ClusterType;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.robotics.geometry.ConvexPolygonConstructorFromInteriorOfRays;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.tools.lists.PairList;

public class ClusterTools
{
   //TODO: +++JerryPratt: The case of vertical PlanarRegions needs a lot of work. The outside sides will get extruded based on their height, even if close by
   // there are high vertices. Need some example cases for this and need to fix it up.
   private static final double HALF_PI = 0.5 * Math.PI;
   private static final double POPPING_POLYGON_POINTS_THRESHOLD = 0.0; //MathTools.square(0.025);
   private static final double POPPING_MULTILINE_POINTS_THRESHOLD = MathTools.square(0.10);
   private static final double NAV_TO_NON_NAV_DISTANCE = 0.001;

   public static List<ExtrusionHull> extrudePolygonInward(List<ConvexPolygon2D> polygons, ObstacleExtrusionDistanceCalculator calculator)
   {
      List<ExtrusionHull> listOfExtrusions = new ArrayList<>();
      for (ConvexPolygon2DReadOnly polygon : polygons)
      {
         double[] extrusionDistances = polygon.getPolygonVerticesView().stream().mapToDouble(rawPoint -> calculator.computeExtrusionDistance(new Point2D(rawPoint), 0.0)).toArray();
         ConvexPolygon2D scaledPolygon = new ConvexPolygon2D();
         extrudeConvexPolygonInward(polygon, extrusionDistances, scaledPolygon);

         listOfExtrusions.add(new ExtrusionHull(scaledPolygon.getPolygonVerticesView()));
      }

      return listOfExtrusions;
   }

   /**
    * Grows or shrinks the size of the polygon, If distance is positive it shrinks the polygon in by the distance in meters,
    * If the distance is negative it grows the polygon. If polygonQ is a line and the distance is negative, a 6 point polygon is returned around the line. If
    * polygonQ is a point, a square is returned around the point. polygonQ is not changed.
    */
   public static boolean extrudeConvexPolygonInward(ConvexPolygon2DReadOnly polygonQ, double[] distances, ConvexPolygon2DBasics polygonToPack)
   {
      if (distances.length != polygonQ.getNumberOfVertices())
         throw new IllegalArgumentException("Not a valid number of distances.");

      boolean allAreZero = true;
      for (double distance : distances)
      {
         if (distance < 0.0)
            throw new IllegalArgumentException("Not a valid distance " + distance + ", must be positive.");

         if (distance > 1.0e-10)
         {
            allAreZero = false;
            break;
         }
      }
      if (allAreZero)
      {
         polygonToPack.set(polygonQ);
         return true;
      }

      List<? extends Point2DReadOnly> vertices = polygonQ.getPolygonVerticesView();

      if (polygonQ.getNumberOfVertices() == 2)
      {
         Point2DReadOnly vertex0 = vertices.get(0);
         Point2DReadOnly vertex1 = vertices.get(1);

         if (vertex0.distance(vertex1) < distances[0] + distances[1])
         {
            Point2D midPoint = new Point2D(vertex0);
            midPoint.add(vertex1);
            midPoint.scale(0.5);

            polygonToPack.clear();
            polygonToPack.addVertex(midPoint);
            polygonToPack.update();
            return false;
         }

         double edgeLength = vertex0.distance(vertex1);
         double percentageAlongSegment0 = distances[0] / edgeLength;
         double percentageAlongSegment1 = distances[1] / edgeLength;

         Point2D newVertex0 = new Point2D();
         Point2D newVertex1 = new Point2D();
         newVertex0.interpolate(vertex0, vertex1, percentageAlongSegment0);
         newVertex1.interpolate(vertex1, vertex0, percentageAlongSegment1);

         polygonToPack.clear();
         polygonToPack.addVertex(newVertex0);
         polygonToPack.addVertex(newVertex1);
         polygonToPack.update();

         return true;
      }

      if (polygonQ.getNumberOfVertices() == 1)
      {
         polygonToPack.set(polygonQ);
         return false;
      }

      ArrayList<Line2D> rays = new ArrayList<>();

      int leftMostIndexOnPolygonQ = EuclidGeometryPolygonTools
            .findVertexIndex(polygonQ, true, Bound.MIN, Bound.MIN);
      Point2DReadOnly vertexQ = polygonQ.getVertex(leftMostIndexOnPolygonQ);
      int nextVertexQIndex = polygonQ.getNextVertexIndex(leftMostIndexOnPolygonQ);
      Point2DReadOnly nextVertexQ = polygonQ.getVertex(nextVertexQIndex);
      Point2DReadOnly nextNextVertexQ = polygonQ.getNextVertex(nextVertexQIndex);

      Vector2D normalizedVector = new Vector2D();
      Vector2D nextNormalizedVector = new Vector2D();
      Point2D referencePoint = new Point2D();

      for (int i = 0; i < polygonQ.getNumberOfVertices(); i++)
      {
         normalizedVector.sub(nextVertexQ, vertexQ);
         normalizedVector.normalize();
         nextNormalizedVector.sub(nextNextVertexQ, nextVertexQ);
         nextNormalizedVector.normalize();

         // don't include collinear points
         if (normalizedVector.dot(nextNormalizedVector) < 1.0 - 1e-6)
         {
            Vector2DBasics vectorPerpendicularToEdgeOnQ = EuclidGeometryTools.perpendicularVector2D(normalizedVector);
            vectorPerpendicularToEdgeOnQ.negate();

            referencePoint.scaleAdd(distances[i], vectorPerpendicularToEdgeOnQ, vertexQ);

            rays.add(new Line2D(referencePoint, normalizedVector));
         }

         nextVertexQIndex = polygonQ.getNextVertexIndex(nextVertexQIndex);

         vertexQ = nextVertexQ;
         nextVertexQ = polygonQ.getVertex(nextVertexQIndex);
         nextNextVertexQ = polygonQ.getNextVertex(nextVertexQIndex);
      }


      ConvexPolygonConstructorFromInteriorOfRays convexPolygonConstructorFromInteriorOfRays = new ConvexPolygonConstructorFromInteriorOfRays();

      boolean foundSolution = convexPolygonConstructorFromInteriorOfRays.constructFromInteriorOfRays(rays, polygonToPack);
      if (!foundSolution)
      {
         polygonToPack.clear();
         polygonToPack.addVertex(polygonQ.getCentroid());
         polygonToPack.update();
      }

      return foundSolution;
   }

   public static ExtrusionHull extrudePolygon(boolean extrudeToTheLeft, Cluster cluster, ObstacleExtrusionDistanceCalculator calculator)
   {
      return new ExtrusionHull(extrudePolygon(extrudeToTheLeft, cluster.getRawPointsInLocal3D(), calculator));
   }

   public static List<Point2DReadOnly> extrudePolygon(boolean extrudeToTheLeft, List<Point3DReadOnly> rawPoints, ObstacleExtrusionDistanceCalculator calculator)
   {
      return extrudePolygon(extrudeToTheLeft, rawPoints, calculator, false);
   }

   public static List<Point2DReadOnly> extrudePolygon(boolean extrudeToTheLeft, List<Point3DReadOnly> rawPoints, ObstacleExtrusionDistanceCalculator calculator, boolean uniformExtrusionDistance)
   {
      double[] extrusionDistances = new double[rawPoints.size()];
      List<Point2DReadOnly> rawPoints2D = new ArrayList<>();
      double maxValue = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < rawPoints.size(); i++)
      {
         Point3DReadOnly rawPoint = rawPoints.get(i);
         Point2DReadOnly rawPoint2D = new Point2D(rawPoint);
         rawPoints2D.add(rawPoint2D);
         extrusionDistances[i] = calculator.computeExtrusionDistance(rawPoint2D, rawPoint.getZ());
         maxValue = Math.max(maxValue, extrusionDistances[i]);
      }
      if (uniformExtrusionDistance)
      {
         for (int i = 0; i < rawPoints.size(); i++)
            extrusionDistances[i] = maxValue;
      }
      return extrudePolygon(extrudeToTheLeft, rawPoints2D, extrusionDistances);
   }

   public static List<Point2DReadOnly> extrudePolygon(boolean extrudeToTheLeft, List<Point2DReadOnly> pointsToExtrude, double[] extrusionDistances)
   {
      if (pointsToExtrude.size() == 2)
      {
         return extrudeMultiLine(pointsToExtrude, extrusionDistances, 5);
      }

      List<Point2DReadOnly> extrusions = new ArrayList<>();

      // gets all the edges, where edge i is the edge that ends at point i.
      List<LineSegment2DReadOnly> edges = getAllEdges(pointsToExtrude);

      for (int i = 0; i < pointsToExtrude.size(); i++)
      {
         Point2DReadOnly previousPoint = ListWrappingIndexTools.getPrevious(i, pointsToExtrude);
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(i);

         if (pointToExtrude.distanceSquared(previousPoint) < POPPING_POLYGON_POINTS_THRESHOLD)
            continue;

         double extrusionDistance = extrusionDistances[i];

         LineSegment2DReadOnly edgePrev = edges.get(i);
         LineSegment2DReadOnly edgeNext = ListWrappingIndexTools.getNext(i, edges);

         boolean shouldExtrudeCorner;

         //TODO: +++JerryPratt: Think about half_pi limits here. Do they make the most sense? Just a little over and you still might want to round the corner...
         double cornerAngle = edgePrev.direction(false).angle(edgeNext.direction(false));
         if (extrudeToTheLeft)
            shouldExtrudeCorner = cornerAngle <= -HALF_PI;
         else
            shouldExtrudeCorner = cornerAngle >= HALF_PI;

         if (shouldExtrudeCorner)
         {
            int numberOfExtrusionsAtEndpoints = 3;
            extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, extrudeToTheLeft, numberOfExtrusionsAtEndpoints,
                                                                   extrusionDistance));
         }
         else
         {
            extrusions.add(extrudeSinglePointAtInsideCorner(pointToExtrude, edgePrev, edgeNext, extrudeToTheLeft, extrusionDistance));
         }
      }

      return extrusions;
   }

   private static List<LineSegment2DReadOnly> getAllEdges(List<Point2DReadOnly> points)
   {
      List<LineSegment2DReadOnly> edges = new ArrayList<>();

      for (int i = 0; i < points.size(); i++)
      {
         Point2DReadOnly previousPoint = ListWrappingIndexTools.getPrevious(i, points);
         Point2DReadOnly pointToExtrude = points.get(i);

         edges.add(new LineSegment2D(previousPoint, pointToExtrude));
      }

      return edges;
   }

   public static List<Point2DReadOnly> extrudeMultiLine(List<Point3DReadOnly> rawPoints, ObstacleExtrusionDistanceCalculator calculator, int numberOfExtrusionsAtEndpoints)
   {
      double[] extrusionDistances = new double[rawPoints.size()];
      List<Point2DReadOnly> rawPoints2D = new ArrayList<>();
      for (int i = 0; i < rawPoints.size(); i++)
      {
         Point3DReadOnly rawPoint = rawPoints.get(i);
         Point2DReadOnly rawPoint2D = new Point2D(rawPoint);
         rawPoints2D.add(rawPoint2D);
         extrusionDistances[i] = calculator.computeExtrusionDistance(rawPoint2D, rawPoint.getZ());
      }
      return extrudeMultiLine(rawPoints2D, extrusionDistances, numberOfExtrusionsAtEndpoints);
   }

   /**
    * Enlarges the area around a multi-point line segment to create a closed polygon and returns the polygon as a list of points.
    * Resulting polygon is in clockwise ordering.
    */
   public static List<Point2DReadOnly> extrudeMultiLine(List<Point2DReadOnly> pointsToExtrude, double[] extrusionDistances, int numberOfExtrusionsAtEndpoints)
   {
      List<Point2DReadOnly> extrusions = new ArrayList<>();

      if (pointsToExtrude.size() >= 2)
      {
         // Start
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(0);
         double extrusionDistance = extrusionDistances[0];

         LineSegment2D edgePrev = new LineSegment2D(pointsToExtrude.get(1), pointToExtrude);
         LineSegment2D edgeNext = new LineSegment2D(pointToExtrude, pointsToExtrude.get(1));
         extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = 1; i < pointsToExtrude.size() - 1; i++)
      {
         // Go from start to end
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(i);
         double extrusionDistance = extrusionDistances[i];

         LineSegment2D edgePrev = new LineSegment2D(pointsToExtrude.get(i - 1), pointToExtrude);
         LineSegment2D edgeNext = new LineSegment2D(pointToExtrude, pointsToExtrude.get(i + 1));

         boolean shouldExtrudeCorner = edgePrev.direction(false).angle(edgeNext.direction(false)) <= -HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            extrusions.add(extrudeSinglePointAtInsideCorner(pointToExtrude, edgePrev, edgeNext, true, extrusionDistance));
         }
      }

      if (pointsToExtrude.size() >= 2)
      {
         // End
         int lastIndex = pointsToExtrude.size() - 1;
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(lastIndex);
         double extrusionDistance = extrusionDistances[lastIndex];

         LineSegment2D edgePrev = new LineSegment2D(pointsToExtrude.get(lastIndex - 1), pointToExtrude);
         LineSegment2D edgeNext = new LineSegment2D(pointToExtrude, pointsToExtrude.get(lastIndex - 1));
         extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = pointsToExtrude.size() - 2; i >= 1; i--)
      {
         // Go from end back to start
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(i);
         double extrusionDistance = extrusionDistances[i];

         LineSegment2D edgePrev = new LineSegment2D(pointsToExtrude.get(i + 1), pointToExtrude);
         LineSegment2D edgeNext = new LineSegment2D(pointToExtrude, pointsToExtrude.get(i - 1));

         boolean shouldExtrudeCorner = edgePrev.direction(false).angle(edgeNext.direction(false)) <= -HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            extrusions.add(extrudeSinglePointAtInsideCorner(pointToExtrude, edgePrev, edgeNext, true, extrusionDistance));
         }
      }

      return extrusions;
   }

   /**
    * Extrudes a single point at the extrusionDistance. If this is to the inside of a corner (angle is less than 180),
    * then the two new lines will be moved by the extrusionDistance.
    * If it is to the outside, then you should use extrudeMultiplePointsAtOutsideCorner() instead.
    */
   public static Point2DReadOnly extrudeSinglePointAtInsideCorner(Point2DReadOnly pointToExtrude, LineSegment2DReadOnly edgePrev,
                                                                  LineSegment2DReadOnly edgeNext,  boolean extrudeToTheLeft, double extrusionDistance)
   {
      Vector2DBasics previousEdgeDirection = edgePrev.direction(true);
      Vector2DBasics nextEdgeDirection = edgeNext.direction(true);

      Vector2D extrusionDirection = new Vector2D();
      extrusionDirection.interpolate(previousEdgeDirection, nextEdgeDirection, 0.5);
      extrusionDirection.normalize();

      double cosTheta = -previousEdgeDirection.dot(nextEdgeDirection);
      double oneMinusCosThetaOverTwo = (1.0 - cosTheta) / 2.0;

      // Just in case. This should never happen, but with roundoff errors, sometimes it does.
      if (oneMinusCosThetaOverTwo < Double.MIN_VALUE)
      {
         oneMinusCosThetaOverTwo = Double.MIN_VALUE;
      }

      double sinThetaOverTwo = Math.sqrt(oneMinusCosThetaOverTwo);
      double extrusionMultiplier = 1.0 / sinThetaOverTwo;

      //TODO: Hackish here. Maybe pass in magic number as a parameter.
      // But without this, could blow up to near infinity.
      if (extrusionMultiplier > 3.0)
         extrusionMultiplier = 3.0;

      extrusionDistance = extrusionDistance * extrusionMultiplier;

      extrusionDirection = EuclidGeometryTools.perpendicularVector2D(extrusionDirection);
      if (!extrudeToTheLeft)
      {
         extrusionDirection.negate();
      }

      Point2D extrusion = new Point2D();
      extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);

      return extrusion;
   }

   public static List<Point2DReadOnly> extrudeMultiplePointsAtOutsideCorner(Point2DReadOnly cornerPointToExtrude, LineSegment2DReadOnly previousEdge,
                                                                            LineSegment2DReadOnly nextEdge, boolean extrudeToTheLeft, int numberOfExtrusions,
                                                                            double extrusionDistance)
   {
      List<Point2DReadOnly> extrusions = new ArrayList<>();

      Vector2D firstExtrusionDirection = EuclidGeometryTools.perpendicularVector2D(previousEdge.direction(true));
      if (!extrudeToTheLeft)
         firstExtrusionDirection.negate();
      Point2D firstExtrusion = new Point2D();
      firstExtrusion.scaleAdd(extrusionDistance, firstExtrusionDirection, cornerPointToExtrude);
      extrusions.add(firstExtrusion);

      Vector2D lastExtrusionDirection = EuclidGeometryTools.perpendicularVector2D(nextEdge.direction(true));
      if (!extrudeToTheLeft)
         lastExtrusionDirection.negate();
      Point2D lastExtrusion = new Point2D();
      lastExtrusion.scaleAdd(extrusionDistance, lastExtrusionDirection, cornerPointToExtrude);

      if (numberOfExtrusions > 2)
      {
         double openingAngle = firstExtrusionDirection.angle(lastExtrusionDirection);
         if (MathTools.epsilonEquals(Math.PI, Math.abs(openingAngle), 1.0e-7))
            openingAngle = extrudeToTheLeft ? -Math.PI : Math.PI;

         Vector2D extrusionDirection = new Vector2D();

         for (int i = 1; i < numberOfExtrusions - 1; i++)
         {
            double alpha = i / (numberOfExtrusions - 1.0);
            RotationMatrixTools.applyYawRotation(alpha * openingAngle, firstExtrusionDirection, extrusionDirection);
            Point2D extrusion = new Point2D();
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, cornerPointToExtrude);
            extrusions.add(extrusion);
         }
      }

      extrusions.add(lastExtrusion);

      return extrusions;
   }

   public static Cluster getTheClosestCluster(Point3DReadOnly pointToSortFrom, List<Cluster> clusters)
   {
      double minDistance = Double.MAX_VALUE;
      Cluster closestCluster = null;

      for (Cluster cluster : clusters)
      {
         double distOfPoint = Double.MAX_VALUE;
         Point3DReadOnly closestPointInCluster = null;

         for (Point3DReadOnly point : cluster.getNonNavigableExtrusionsInWorld())
         {
            double currentDistance = point.distanceSquared(pointToSortFrom);
            if (currentDistance < distOfPoint)
            {
               distOfPoint = currentDistance;
               closestPointInCluster = point;
            }
         }

         double currentDistance = closestPointInCluster.distanceSquared(pointToSortFrom);

         if (currentDistance < minDistance)
         {
            minDistance = currentDistance;
            closestCluster = cluster;
         }
      }

      return closestCluster;
   }

   public static Point3D getTheClosestVisibleExtrusionPoint(Point3DReadOnly pointToSortFrom, List<Point3D> extrusionPoints)
   {
      double minDistance = Double.MAX_VALUE;
      Point3D closestPoint = null;

      for (Point3D point : extrusionPoints)
      {
         double currentDistance = point.distanceSquared(pointToSortFrom);
         if (currentDistance < minDistance)
         {
            minDistance = currentDistance;
            closestPoint = point;
         }
      }

      return closestPoint;
   }

   public static Point3D getTheClosestVisibleExtrusionPoint(double alpha, Point3DReadOnly start, Point3DReadOnly goal,
                                                            List<? extends Point3DReadOnly> extrusionPoints, PlanarRegion region)
   {
      double minWeight = Double.MAX_VALUE;
      Point3DReadOnly closestPoint = null;

      for (Point3DReadOnly point : extrusionPoints)
      {
         if (PlanarRegionTools.isPointInWorldInsidePlanarRegion(region, point))
         {
            double weight = alpha * goal.distance(point) + (1 - alpha) * start.distance(point);

            if (weight < minWeight)
            {
               minWeight = weight;
               closestPoint = point;
            }
         }
      }

      return new Point3D(closestPoint);
   }

   public static Cluster createHomeRegionCluster(PlanarRegion homeRegion, NavigableExtrusionDistanceCalculator preferredCalculator,
                                                 NavigableExtrusionDistanceCalculator calculator, boolean includePreferredExtrusions)
   {
      Cluster homeRegionCluster = new Cluster(ExtrusionSide.INSIDE, ClusterType.POLYGON);
      homeRegionCluster.setTransformToWorld(homeRegion.getTransformToWorld());
      homeRegionCluster.addRawPointsInLocal2D(homeRegion.getConcaveHull());

      double preferredExtrusionDistance = preferredCalculator.computeNavigableExtrusionDistance(homeRegion);
      double extrusionDistance = calculator.computeNavigableExtrusionDistance(homeRegion);

      ObstacleExtrusionDistanceCalculator preferredNonNavigableCalculator = (p, h) -> preferredExtrusionDistance - NAV_TO_NON_NAV_DISTANCE;
      ObstacleExtrusionDistanceCalculator preferredNavigableCalculator = (p, h) -> preferredExtrusionDistance;
      ObstacleExtrusionDistanceCalculator nonNavigableCalculator = (p, h) -> extrusionDistance - NAV_TO_NON_NAV_DISTANCE;
      ObstacleExtrusionDistanceCalculator navigableCalculator = (p, h) -> extrusionDistance;

      boolean extrudeToTheLeft = homeRegionCluster.getExtrusionSide() != ExtrusionSide.INSIDE;

      //TODO: JEP+++: Why do we add a NonNavigableExtrusion to a home region cluster?
      // I guess it's for inner region connections that cross over empty space.
      // Need to make sure they don't. But then also need to make sure these
      // NonNavigable regions are not treated as boundaries when making
      // inter region connections...
      if (includePreferredExtrusions)
      {
         List<ConvexPolygon2D> polygons = homeRegion.getConvexPolygons();
         homeRegionCluster.addPreferredNonNavigableExtrusionsInLocal(extrudePolygonInward(polygons, preferredNonNavigableCalculator));
         homeRegionCluster.addPreferredNavigableExtrusionsInLocal(extrudePolygonInward(polygons, preferredNavigableCalculator));
      }
      homeRegionCluster.addNonNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, homeRegionCluster, nonNavigableCalculator));
      homeRegionCluster.addNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, homeRegionCluster, navigableCalculator));
      return homeRegionCluster;
   }

   public static PairList<Cluster, PlanarRegion> createObstacleClusters(PlanarRegion homeRegion, List<PlanarRegion> obstacleRegions, double orthogonalAngle,
                                                                        ObstacleExtrusionDistanceCalculator preferredExtrusionDistanceCalculator,
                                                                        ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator,
                                                                        boolean includePreferredExtrusions)
   {
      PairList<Cluster, PlanarRegion> obstacleClusters = new PairList<>();

      double zThresholdBeforeOrthogonal = Math.cos(orthogonalAngle);

      for (PlanarRegion obstacleRegion : obstacleRegions)
      {
         Cluster obstacleCluster = createObstacleCluster(homeRegion, preferredExtrusionDistanceCalculator, extrusionDistanceCalculator,
                                                         homeRegion.getTransformToWorld(), zThresholdBeforeOrthogonal, obstacleRegion,
                                                         includePreferredExtrusions);
         obstacleClusters.add(obstacleCluster, obstacleRegion);
      }

      return obstacleClusters;
   }

   private static Cluster createObstacleCluster(PlanarRegion homeRegion, ObstacleExtrusionDistanceCalculator preferredExtrusionDistanceCalculator,
                                                ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator,
                                                RigidBodyTransformReadOnly transformFromHomeRegionToWorld, double zThresholdBeforeOrthogonal,
                                                PlanarRegion obstacleRegion, boolean includePreferredExtrusions)
   {
      List<Point2D> concaveHull = obstacleRegion.getConcaveHull();

      RigidBodyTransformReadOnly transformFromObstacleToWorld = obstacleRegion.getTransformToWorld();

      List<Point3DReadOnly> obstacleConcaveHullInWorld = new ArrayList<>();
      List<Point3DReadOnly> obstacleClusterPointsWithZeroZ = new ArrayList<>();
      // Transform the obstacle to world and also Project the obstacle to z = 0
      calculatePointsInWorldAtRegionHeight(concaveHull, transformFromObstacleToWorld, homeRegion, obstacleConcaveHullInWorld, obstacleClusterPointsWithZeroZ);

      Vector3DReadOnly obstacleNormal = obstacleRegion.getNormal();
      boolean verticalObstacle = Math.abs(obstacleNormal.getZ()) < zThresholdBeforeOrthogonal;

      ClusterType obstacleClusterType = getClusterType(verticalObstacle);
      //TODO: +++JerryPratt: Rethink vertical obstacles and redo how they are done. Lots of potential issues with things like single region doorways, regions that start and end at a low height, etc.
      if (verticalObstacle)
         obstacleClusterPointsWithZeroZ = filterVerticalPolygonForMultiLineExtrusion(obstacleClusterPointsWithZeroZ, POPPING_MULTILINE_POINTS_THRESHOLD);

      // actually extrude the points
      List<? extends Point2DReadOnly> navigableExtrusionsInFlatWorld = computeObstacleNavigableExtrusionsInLocal(obstacleClusterType, obstacleClusterPointsWithZeroZ,
                                                                                                                 extrusionDistanceCalculator);
      List<? extends Point2DReadOnly> nonNavigableExtrusionsInFlatWorld = computeObstacleNonNavigableExtrusionsInLocal(obstacleClusterType, obstacleClusterPointsWithZeroZ,
                                                                                                                       extrusionDistanceCalculator);

      // Project the points back up to the home region.
      RigidBodyTransform transformFromWorldToHome = new RigidBodyTransform(transformFromHomeRegionToWorld);
      transformFromWorldToHome.invert();
      ExtrusionHull navigableExtrusionsInHomeRegionLocal = projectPointsVerticallyToPlanarRegionLocal(homeRegion, navigableExtrusionsInFlatWorld,
                                                                                                      transformFromWorldToHome);
      ExtrusionHull nonNavigableExtrusionsInHomeRegionLocal = projectPointsVerticallyToPlanarRegionLocal(homeRegion,
                                                                                                         nonNavigableExtrusionsInFlatWorld,
                                                                                                         transformFromWorldToHome);

      List<ExtrusionHull> preferredNavigableExtrusionsInHomeRegionLocal = null;
      List<ExtrusionHull> preferredNonNavigableExtrusionsInHomeRegionLocal = null;
      if (includePreferredExtrusions)
      {
         // actually extrude the points
         List<? extends Point2DReadOnly> preferredNavigableExtrusionsInFlatWorld = computeObstacleNavigableExtrusionsInLocal(obstacleClusterType,
                                                                                                                             obstacleClusterPointsWithZeroZ,
                                                                                                                             preferredExtrusionDistanceCalculator);

         List<? extends Point2DReadOnly> preferredNonNavigableExtrusionsInFlatWorld = computeObstacleNonNavigableExtrusionsInLocal(obstacleClusterType,
                                                                                                                                   obstacleClusterPointsWithZeroZ,
                                                                                                                                   preferredExtrusionDistanceCalculator);

         // Project the points back to the the home region.
         preferredNavigableExtrusionsInHomeRegionLocal = new ArrayList<>();
         preferredNavigableExtrusionsInHomeRegionLocal.add(projectPointsVerticallyToPlanarRegionLocal(homeRegion, preferredNavigableExtrusionsInFlatWorld,
                                                                                                      transformFromWorldToHome));

         preferredNonNavigableExtrusionsInHomeRegionLocal = new ArrayList<>();
         preferredNonNavigableExtrusionsInHomeRegionLocal.add(projectPointsVerticallyToPlanarRegionLocal(homeRegion, preferredNonNavigableExtrusionsInFlatWorld,
                                                                                                         transformFromWorldToHome));
      }

      Cluster cluster = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      cluster.setTransformToWorld(transformFromHomeRegionToWorld);
      cluster.addRawPointsInWorld(obstacleConcaveHullInWorld);
      if (includePreferredExtrusions)
      {
         cluster.setPreferredNavigableExtrusionsInLocal(preferredNavigableExtrusionsInHomeRegionLocal);
         cluster.setPreferredNonNavigableExtrusionsInLocal(preferredNonNavigableExtrusionsInHomeRegionLocal);
      }
      cluster.setNavigableExtrusionsInLocal(navigableExtrusionsInHomeRegionLocal);
      cluster.setNonNavigableExtrusionsInLocal(nonNavigableExtrusionsInHomeRegionLocal);

      return cluster;
   }

   public static void calculatePointsInWorldAtRegionHeight(List<? extends Point2DReadOnly> points,
                                                            RigidBodyTransformReadOnly transformToWorld,
                                                            PlanarRegion region,
                                                            List<Point3DReadOnly> pointsInWorldToPack,
                                                            List<Point3DReadOnly> pointsOnRegionInWorldToPack)
   {
      if (pointsInWorldToPack != null)
         pointsInWorldToPack.clear();
      pointsOnRegionInWorldToPack.clear();

      for (int i = 0; i < points.size(); i++)
      {
         Point2DReadOnly point = points.get(i);
         Point3D pointInWorld = new Point3D(point);
         pointInWorld.applyTransform(transformToWorld);

         if (pointsInWorldToPack != null)
            pointsInWorldToPack.add(pointInWorld);

         double zInHomeRegion = region.getPlaneZGivenXY(pointInWorld.getX(), pointInWorld.getY());

         double obstacleHeight = pointInWorld.getZ() - zInHomeRegion;
         Point3D tempPoint = new Point3D(pointInWorld);
         tempPoint.setZ(obstacleHeight);

         pointsOnRegionInWorldToPack.add(tempPoint);
      }
   }

   private static ClusterType getClusterType(boolean verticalExtrusion)
   {
      return verticalExtrusion ? ClusterType.MULTI_LINE : ClusterType.POLYGON;
   }

   public static ExtrusionHull projectPointsVerticallyToPlanarRegionLocal(PlanarRegion planarRegionToProjectOnto,
                                                                                   List<? extends Point2DReadOnly> pointsToProjectInWorld,
                                                                                   RigidBodyTransformReadOnly transformFromWorldToPlanarRegion)
   {
      ExtrusionHull navigableExtrusionsInHomeRegionLocal = new ExtrusionHull();
      for (int i = 0; i < pointsToProjectInWorld.size(); i++)
      {
         Point2DReadOnly navigableExtrusionInFlatWorld = pointsToProjectInWorld.get(i);
         Point3D navigableExtrusionInFlatWorld3D = new Point3D(navigableExtrusionInFlatWorld);

         Point3D extrudedPointOnHomeRegion = PlanarRegionTools.projectInZToPlanarRegion(navigableExtrusionInFlatWorld3D, planarRegionToProjectOnto);

         transformFromWorldToPlanarRegion.transform(extrudedPointOnHomeRegion);
         navigableExtrusionsInHomeRegionLocal.addPoint(extrudedPointOnHomeRegion);
      }

      return navigableExtrusionsInHomeRegionLocal;
   }

   public static List<? extends Point2DReadOnly> computeObstacleNonNavigableExtrusionsInLocal(ClusterType type, List<Point3DReadOnly> rawClusterPoints,
                                                                                               ObstacleExtrusionDistanceCalculator calculator)
   {
      int numberOfExtrusionsAtEndpoints = 5;
      ObstacleExtrusionDistanceCalculator nonNavigableCalculator = (p, h) -> calculator.computeExtrusionDistance(p, h) - NAV_TO_NON_NAV_DISTANCE;

      switch (type)
      {
      case MULTI_LINE:
         return extrudeMultiLine(rawClusterPoints, nonNavigableCalculator, numberOfExtrusionsAtEndpoints);
      case POLYGON:
         return extrudePolygon(true, rawClusterPoints, nonNavigableCalculator);
      default:
         throw new RuntimeException("Unhandled cluster type: " + type);
      }
   }

   public static List<? extends Point2DReadOnly> computeObstacleNavigableExtrusionsInLocal(ClusterType type, List<Point3DReadOnly> rawClusterPoints,
                                                                                           ObstacleExtrusionDistanceCalculator calculator)
   {
      return computeObstacleNavigableExtrusionsInLocal(type, rawClusterPoints, calculator, false);
   }

   public static List<? extends Point2DReadOnly> computeObstacleNavigableExtrusionsInLocal(ClusterType type, List<Point3DReadOnly> rawClusterPoints,
                                                                                            ObstacleExtrusionDistanceCalculator calculator,
                                                                                           boolean useUniformExtrusionDistance)
   {
      int numberOfExtrusionsAtEndpoints = 5;

      switch (type)
      {
      case MULTI_LINE:
         return extrudeMultiLine(rawClusterPoints, calculator, numberOfExtrusionsAtEndpoints);
      case POLYGON:
         return extrudePolygon(true, rawClusterPoints, calculator, useUniformExtrusionDistance);
      default:
         throw new RuntimeException("Unhandled cluster type: " + type);
      }
   }


   /**
    * @param verticalPolygonVertices
    * @param poppingPointsDistanceSquaredThreshold
    * @return
    */
   public static List<Point3DReadOnly> filterVerticalPolygonForMultiLineExtrusion(List<? extends Point3DReadOnly> verticalPolygonVertices,
                                                                   double poppingPointsDistanceSquaredThreshold)
   {
      if (verticalPolygonVertices.size() <= 2)
         return verticalPolygonVertices.stream().map(Point3D::new).collect(Collectors.toList());

      // Making a deep copy
      List<Point3D> filteredPoints = verticalPolygonVertices.stream().map(Point3D::new).collect(Collectors.toList());

      Point3D mean = new Point3D();
      Vector3D principalVector = new Vector3D();
      PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

      pca.clear();
      filteredPoints.forEach(p -> pca.addPoint(p.getX(), p.getY(), 0.0));
      pca.compute();
      pca.getMean(mean);
      pca.getPrincipalVector(principalVector);
      Line2D line = new Line2D(new Point2D(mean), new Vector2D(principalVector));
      // Projecting the points in the 2D plane described by the z-axis and the line direction
      List<Point2D> projectedPoints = new ArrayList<>();
      for (Point3D point : filteredPoints)
      {
         double x = line.parameterGivenPointOnLine(new Point2D(point), Double.POSITIVE_INFINITY);
         double z = point.getZ();
         projectedPoints.add(new Point2D(x, z));
      }

      for (int pointIndex = 0; pointIndex < projectedPoints.size(); pointIndex++)
      {
         Point3D point = filteredPoints.get(pointIndex);
         Point2D projectedPoint = projectedPoints.get(pointIndex);

         for (int edgeIndex = 0; edgeIndex < projectedPoints.size(); edgeIndex++)
         {
            Point2D edgeStart = projectedPoints.get(edgeIndex);
            Point2D edgeEnd = ListWrappingIndexTools.getNext(edgeIndex, projectedPoints);

            // Check if the point is between start and end
            double signedDistanceToStart = edgeStart.getX() - projectedPoint.getX();
            double signedDistanceToEnd = edgeEnd.getX() - projectedPoint.getX();
            if (signedDistanceToStart * signedDistanceToEnd > 0.0)
               continue; // If same sign, the edge is not above/below the point, keep going.
            // The edge is above or below the point, let's compute the edge height at the point x-coordinate
            double alpha = EuclidGeometryTools.percentageAlongLineSegment2D(projectedPoint, edgeStart, edgeEnd);
            double height = EuclidCoreTools.interpolate(edgeStart.getY(), edgeEnd.getY(), alpha);

            point.setZ(Math.max(height, point.getZ()));
         }

         // Adjust the XY-coordinates to be on the line
         point.set(line.pointOnLineGivenParameter(projectedPoint.getX()));
      }

      // Sort the points given their position on the line.
      Collections.sort(filteredPoints, (p1, p2) -> {
         double t1 = line.parameterGivenPointOnLine(new Point2D(p1), Double.POSITIVE_INFINITY);
         double t2 = line.parameterGivenPointOnLine(new Point2D(p2), Double.POSITIVE_INFINITY);
         return t1 >= t2 ? 1 : -1;
      });

      // FIXME Problem with the obstacle height.
      if (filteredPoints.get(0).distanceXYSquared(filteredPoints.get(filteredPoints.size() - 1)) <= poppingPointsDistanceSquaredThreshold)
      {
         double maxHeight = filteredPoints.stream().map(Point3D::getZ).max((d1, d2) -> Double.compare(d1, d2)).get();
         List<Point3DReadOnly> endpoints = new ArrayList<>();
         Point3D endPoint0 = filteredPoints.get(0);
         Point3D endPoint1 = filteredPoints.get(filteredPoints.size() - 1);
         endPoint0.setZ(maxHeight);
         endPoint1.setZ(maxHeight);
         endpoints.add(endPoint0);
         endpoints.add(endPoint1);
         return endpoints;
      }
      else
      {
         filteredPoints = filterPointsWithSameXYCoordinatesKeepingHighest(filteredPoints, poppingPointsDistanceSquaredThreshold);
         return new ArrayList<>(filteredPoints);
      }
   }

   /**
    * Goes through a list of points and removes duplicates next to each other whose xy distance squared is less than poppingPointsDistanceSquaredThreshold.
    * It assumes the list is ordered in such a way that the only possible duplicates are next to each other.
    * In removing a duplicate, it keeps the one with the larger z value.
    * At the end, all points are guaranteed to be greater than the threshold distance apart.
    *
    * @param pointsToFilter List of points to be filtered.
    * @param poppingPointsDistanceSquaredThreshold Threshold for the squared distance for considering points to be duplicates.
    */
   public static List<Point3D> filterPointsWithSameXYCoordinatesKeepingHighest(List<Point3D> pointsToFilter, double poppingPointsDistanceSquaredThreshold)
   {
      List<Point3D> filteredPointsToReturn = new ArrayList<>();
      if (pointsToFilter.isEmpty())
         return filteredPointsToReturn;

      if (pointsToFilter.size() == 1)
      {
         filteredPointsToReturn.add(pointsToFilter.get(0));
         return filteredPointsToReturn;
      }

      boolean pointWasFiltered = false;

      for (int i = 0; i < pointsToFilter.size(); i++)
      {
         if (i==pointsToFilter.size() - 1)
         {
            filteredPointsToReturn.add(pointsToFilter.get(i));
            continue;
         }

         Point3D currentPoint = pointsToFilter.get(i);
         Point3D nextPoint = pointsToFilter.get(i + 1);

         if (currentPoint.distanceXYSquared(nextPoint) < poppingPointsDistanceSquaredThreshold)
         {
            if (currentPoint.getZ() > nextPoint.getZ())
            {
               filteredPointsToReturn.add(currentPoint);
            }
            else
            {
               filteredPointsToReturn.add(nextPoint);
            }
            pointWasFiltered = true;
            i++;
         }
         else
         {
            filteredPointsToReturn.add(currentPoint);
         }
      }

      if (pointWasFiltered)
      {
         return filterPointsWithSameXYCoordinatesKeepingHighest(filteredPointsToReturn, poppingPointsDistanceSquaredThreshold);
      }

      return filteredPointsToReturn;

   }
}
