package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class ClusterTools
{
   private static final double HALF_PI = 0.5 * Math.PI;
   private static final double POPPING_POLYGON_POINTS_THRESHOLD = 0.0; //MathTools.square(0.025);
   private static final double POPPING_MULTILINE_POINTS_THRESHOLD = MathTools.square(0.20);
   private static final double NAV_TO_NON_NAV_DISTANCE = 0.001;

   public static List<Point2D> extrudePolygon(boolean extrudeToTheLeft, Cluster cluster, ObstacleExtrusionDistanceCalculator calculator)
   {
      List<Point2DReadOnly> rawPoints = cluster.getRawPointsInLocal2D();
      double[] extrusionDistances = cluster.getRawPointsInLocal3D().stream()
                                           .mapToDouble(rawPoint -> calculator.computeExtrusionDistance(new Point2D(rawPoint), rawPoint.getZ())).toArray();

      return extrudePolygon(extrudeToTheLeft, rawPoints, extrusionDistances);
   }

   public static List<Point2D> extrudePolygon(boolean extrudeToTheLeft, List<Point2DReadOnly> pointsToExtrude, double[] extrusionDistances)
   {
      if (pointsToExtrude.size() == 2)
      {
         return extrudeMultiLine(pointsToExtrude, extrusionDistances, 5);
      }

      List<Point2D> extrusions = new ArrayList<>();

      for (int i = 0; i < pointsToExtrude.size(); i++)
      {
         Point2DReadOnly previousPoint = ListWrappingIndexTools.getPrevious(i, pointsToExtrude);
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(i);

         if (pointToExtrude.distanceSquared(previousPoint) < POPPING_POLYGON_POINTS_THRESHOLD)
            continue;

         Point2DReadOnly nextPoint = ListWrappingIndexTools.getNext(i, pointsToExtrude);

         double extrusionDistance = extrusionDistances[i];

         Line2D edgePrev = new Line2D(previousPoint, pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, nextPoint);

         boolean shouldExtrudeCorner;

         //TODO: Think about half_pi limits here. Do they make the most sense? Just a little over and you still might want to round the corner...
         if (extrudeToTheLeft)
            shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -HALF_PI;
         else
            shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) >= HALF_PI;

         if (shouldExtrudeCorner)
         {
            int numberOfExtrusionsAtEndpoints = 3;
            extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, extrudeToTheLeft, numberOfExtrusionsAtEndpoints,
                                                                   extrusionDistance));
         }
         else
         {
            extrudeSinglePointAtInsideCorner(extrusions, pointToExtrude, extrusionDistance, edgePrev, edgeNext, extrudeToTheLeft);
         }
      }

      return extrusions;
   }

   public static List<Point2D> extrudeMultiLine(Cluster cluster, ObstacleExtrusionDistanceCalculator calculator, int numberOfExtrusionsAtEndpoints)
   {
      List<Point2DReadOnly> rawPoints = cluster.getRawPointsInLocal2D();
      double[] extrusionDistances = cluster.getRawPointsInLocal3D().stream()
                                           .mapToDouble(rawPoint -> calculator.computeExtrusionDistance(new Point2D(rawPoint), rawPoint.getZ())).toArray();

      return extrudeMultiLine(rawPoints, extrusionDistances, numberOfExtrusionsAtEndpoints);
   }

   /**
    * Enlarges the area around a multi-point line segment to create a closed polygon and returns the polygon as a list of points.
    * Resulting polygon is in clockwise ordering.
    */
   public static List<Point2D> extrudeMultiLine(List<Point2DReadOnly> pointsToExtrude, double[] extrusionDistances, int numberOfExtrusionsAtEndpoints)
   {
      List<Point2D> extrusions = new ArrayList<>();

      if (pointsToExtrude.size() >= 2)
      {
         // Start
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(0);
         double extrusionDistance = extrusionDistances[0];

         Line2D edgePrev = new Line2D(pointsToExtrude.get(1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, pointsToExtrude.get(1));
         extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = 1; i < pointsToExtrude.size() - 1; i++)
      {
         // Go from start to end
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(i);
         double extrusionDistance = extrusionDistances[i];

         Line2D edgePrev = new Line2D(pointsToExtrude.get(i - 1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, pointsToExtrude.get(i + 1));

         boolean shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            extrudeSinglePointAtInsideCorner(extrusions, pointToExtrude, extrusionDistance, edgePrev, edgeNext, true);
         }
      }

      if (pointsToExtrude.size() >= 2)
      {
         // End
         int lastIndex = pointsToExtrude.size() - 1;
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(lastIndex);
         double extrusionDistance = extrusionDistances[lastIndex];

         Line2D edgePrev = new Line2D(pointsToExtrude.get(lastIndex - 1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, pointsToExtrude.get(lastIndex - 1));
         extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = pointsToExtrude.size() - 2; i >= 1; i--)
      {
         // Go from end back to start
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(i);
         double extrusionDistance = extrusionDistances[i];

         Line2D edgePrev = new Line2D(pointsToExtrude.get(i + 1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, pointsToExtrude.get(i - 1));

         boolean shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeMultiplePointsAtOutsideCorner(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            extrudeSinglePointAtInsideCorner(extrusions, pointToExtrude, extrusionDistance, edgePrev, edgeNext, true);
         }
      }

      return extrusions;
   }

   /**
    * Extrudes a single point at the extrusionDistance. If this is to the inside of a corner (angle is less than 180), 
    * then the two new lines will be moved by the extrusionDistance. 
    * If it is to the outside, then you should use extrudeMultiplePointsAtOutsideCorner() instead.
    */
   public static void extrudeSinglePointAtInsideCorner(List<Point2D> extrusions, Point2DReadOnly pointToExtrude, double extrusionDistance, Line2D edgePrev,
                                                       Line2D edgeNext, boolean extrudeToTheLeft)
   {
      Vector2DBasics previousEdgeDirection = edgePrev.getDirection();
      Vector2DBasics nextEdgeDirection = edgeNext.getDirection();

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
      extrusions.add(extrusion);
   }

   public static List<Point2D> extrudeLine(Point2DReadOnly endpoint1, Point2DReadOnly endpoint2, double extrusionDistance, int numberOfExtrusionsAtEndpoints)
   {
      return extrudeLine(endpoint1, extrusionDistance, endpoint2, extrusionDistance, numberOfExtrusionsAtEndpoints);
   }

   public static List<Point2D> extrudeLine(Point2DReadOnly endpoint1, double extrusionDistance1, Point2DReadOnly endpoint2, double extrusionDistance2,
                                           int numberOfExtrusionsAtEndpoints)
   {
      List<Point2D> extrusions = new ArrayList<>();
      Line2D edge1 = new Line2D(endpoint1, endpoint2);
      Line2D edge2 = new Line2D(endpoint2, endpoint1);

      List<Point2D> extrusions1 = extrudeMultiplePointsAtOutsideCorner(endpoint1, edge2, edge1, true, numberOfExtrusionsAtEndpoints, extrusionDistance1);
      List<Point2D> extrusions2 = extrudeMultiplePointsAtOutsideCorner(endpoint2, edge1, edge2, true, numberOfExtrusionsAtEndpoints, extrusionDistance2);
      extrusions.addAll(extrusions1);
      extrusions.addAll(extrusions2);

      return extrusions;
   }

   public static List<Point2D> extrudeMultiplePointsAtOutsideCorner(Point2DReadOnly cornerPointToExtrude, Line2D previousEdge, Line2D nextEdge,
                                                                    boolean extrudeToTheLeft, int numberOfExtrusions, double extrusionDistance)
   {
      List<Point2D> extrusions = new ArrayList<>();

      Vector2D firstExtrusionDirection = EuclidGeometryTools.perpendicularVector2D(previousEdge.getDirection());
      if (!extrudeToTheLeft)
         firstExtrusionDirection.negate();
      Point2D firstExtrusion = new Point2D();
      firstExtrusion.scaleAdd(extrusionDistance, firstExtrusionDirection, cornerPointToExtrude);
      extrusions.add(firstExtrusion);

      Vector2D lastExtrusionDirection = EuclidGeometryTools.perpendicularVector2D(nextEdge.getDirection());
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

   public static Cluster createHomeRegionCluster(PlanarRegion homeRegion, NavigableExtrusionDistanceCalculator calculator)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      homeRegion.getTransformToWorld(transformToWorld);

      Cluster homeRegionCluster = new Cluster(ExtrusionSide.INSIDE, ClusterType.POLYGON);
      homeRegionCluster.setTransformToWorld(transformToWorld);
      homeRegionCluster.addRawPointsInLocal2D(homeRegion.getConcaveHull());

      double extrusionDistance = calculator.computeExtrusionDistance(homeRegion);

      ObstacleExtrusionDistanceCalculator nonNavigableCalculator = (p, h) -> extrusionDistance - NAV_TO_NON_NAV_DISTANCE;
      ObstacleExtrusionDistanceCalculator navigableCalculator = (p, h) -> extrusionDistance;

      boolean extrudeToTheLeft = homeRegionCluster.getExtrusionSide() != ExtrusionSide.INSIDE;

      //TODO: JEP+++: Why do we add a NonNavigableExtrusion to a home region cluster?
      // I guess it's for inner region cionnections that cross over empty space.
      // Need to make sure they don't. But then also need to make sure these 
      // NonNavigable regions are not treated as boundaries when making 
      // inter region connections...

      homeRegionCluster.addNonNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, homeRegionCluster, nonNavigableCalculator));
      homeRegionCluster.addNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, homeRegionCluster, navigableCalculator));
      homeRegionCluster.updateBoundingBox();
      return homeRegionCluster;
   }

   public static List<Cluster> createObstacleClusters(PlanarRegion homeRegion, List<PlanarRegion> obstacleRegions, double orthogonalAngle,
                                                      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator)
   {
      List<Cluster> obstacleClusters = new ArrayList<>();

      RigidBodyTransform transformFromHomeToWorld = new RigidBodyTransform();
      homeRegion.getTransformToWorld(transformFromHomeToWorld);

      Vector3D homeRegionSurfaceNormal = homeRegion.getNormal();
      double zThresholdBeforeOrthogonal = Math.cos(orthogonalAngle);

      for (PlanarRegion obstacleRegion : obstacleRegions)
      {
         Cluster obstacleCluster = createObstacleCluster(homeRegion, extrusionDistanceCalculator, transformFromHomeToWorld, homeRegionSurfaceNormal,
                                                         zThresholdBeforeOrthogonal, obstacleRegion);
         obstacleClusters.add(obstacleCluster);
      }

      return obstacleClusters;
   }

   //TODO: +++ Make unnecessary and Delete me!
   private static Cluster createObstacleClusterOld(ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator,
                                                   RigidBodyTransform transformFromHomeRegionToWorld, Vector3D referenceNormal,
                                                   double zThresholdBeforeOrthogonal, PlanarRegion obstacleRegion)
   {
      Vector3D otherNormal = obstacleRegion.getNormal();

      Cluster cluster = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      cluster.setTransformToWorld(transformFromHomeRegionToWorld);

      List<Point3D> rawPointsInLocal = new ArrayList<>();
      RigidBodyTransform transformFromOtherToHome = new RigidBodyTransform();
      obstacleRegion.getTransformToWorld(transformFromOtherToHome);
      transformFromOtherToHome.preMultiplyInvertOther(transformFromHomeRegionToWorld);

      for (int i = 0; i < obstacleRegion.getConvexHull().getNumberOfVertices(); i++)
      {
         Point3D concaveHullVertexHome = new Point3D(obstacleRegion.getConvexHull().getVertex(i));
         concaveHullVertexHome.applyTransform(transformFromOtherToHome);
         rawPointsInLocal.add(concaveHullVertexHome);
      }

      //TODO: Check this. When should it be a multi-line and when should it be a polygon?
      if (Math.abs(otherNormal.dot(referenceNormal)) < zThresholdBeforeOrthogonal)
      {
         // Project region as a line
         cluster.setType(ClusterType.MULTI_LINE);
         cluster.addRawPointsInLocal3D(filterVerticalPolygonForMultiLineExtrusion(rawPointsInLocal, POPPING_MULTILINE_POINTS_THRESHOLD));
      }
      else
      {
         // Project region as a polygon
         cluster.setType(ClusterType.POLYGON);
         cluster.addRawPointsInLocal3D(rawPointsInLocal);
      }

      extrudeObstacleCluster(cluster, extrusionDistanceCalculator);

      return cluster;
   }

   //TODO: +++JEP Finish this for vertical regions, so we don't have to use createObstacleClusterOld.
   private static Cluster createObstacleCluster(PlanarRegion homeRegion, ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator,
                                                RigidBodyTransform transformFromHomeRegionToWorld, Vector3D referenceNormal, double zThresholdBeforeOrthogonal,
                                                PlanarRegion obstacleRegion)
   {
      Vector3D otherNormal = obstacleRegion.getNormal();

      //TODO: Check this. When should it be a multi-line and when should it be a polygon?
      //TODO: +++JEP: Clean up to not have to call the old method...
      if (Math.abs(otherNormal.dot(referenceNormal)) < zThresholdBeforeOrthogonal)
      {
         return createObstacleClusterOld(extrusionDistanceCalculator, transformFromHomeRegionToWorld, referenceNormal, zThresholdBeforeOrthogonal,
                                         obstacleRegion);
      }

      Point2D[] concaveHull = obstacleRegion.getConcaveHull();

      RigidBodyTransform transformFromObstacleToWorld = new RigidBodyTransform();
      obstacleRegion.getTransformToWorld(transformFromObstacleToWorld);

      // Transform the obstacle to world and also Project the obstacle to z = 0:
      List<Point3D> obstacleConcaveHullInWorld = new ArrayList<>();
      List<Point3D> obstacleConcaveHullProjectedToHomeRegion = new ArrayList<>();
      List<Point2D> obstacleConcaveHullProjectedToGround = new ArrayList<>();
      for (int i = 0; i < concaveHull.length; i++)
      {
         Point2DReadOnly obstacleConcaveHullVertexInLocal = concaveHull[i];
         Point3D obstacleConcaveHullVertexInWorld = new Point3D(obstacleConcaveHullVertexInLocal);
         obstacleConcaveHullVertexInWorld.applyTransform(transformFromObstacleToWorld);

         obstacleConcaveHullInWorld.add(obstacleConcaveHullVertexInWorld);

         Point2D obstacleConcaveHullVertexProjectedToGround2D = new Point2D(obstacleConcaveHullVertexInWorld);
         obstacleConcaveHullProjectedToGround.add(obstacleConcaveHullVertexProjectedToGround2D);

         Point3D obstacleConcaveHullVertexProjectedDownToHomeRegion = PlanarRegionTools.projectInZToPlanarRegion(obstacleConcaveHullVertexInWorld, homeRegion);
         obstacleConcaveHullProjectedToHomeRegion.add(obstacleConcaveHullVertexProjectedDownToHomeRegion);
      }

      // TODO: This is good but hackish. Need to clean up to not require this temporary Cluster...
      ArrayList<Point3D> temporaryClusterPoints = createTemporaryClusterPointsSettingZToHeightDifference(obstacleConcaveHullInWorld,
                                                                                                         obstacleConcaveHullProjectedToHomeRegion);
      Cluster tempFlatClusterToExtrude = createTemporaryClusterWithZEqualZeroAndExtrudeIt(extrusionDistanceCalculator, temporaryClusterPoints);

      List<Point2DReadOnly> navigableExtrusionsInFlatWorld = tempFlatClusterToExtrude.getNavigableExtrusionsInLocal();
      List<Point2DReadOnly> nonNavigableExtrusionsInFlatWorld = tempFlatClusterToExtrude.getNonNavigableExtrusionsInLocal();

      // Project the points back up to the home region...

      RigidBodyTransform transformFromWorldToHome = new RigidBodyTransform(transformFromHomeRegionToWorld);
      transformFromWorldToHome.invert();
      List<Point2DReadOnly> navigableExtrusionsInHomeRegionLocal = projectPointsVerticallyToPlanarRegionLocal(homeRegion, navigableExtrusionsInFlatWorld,
                                                                                                              transformFromWorldToHome);
      List<Point2DReadOnly> nonNavigableExtrusionsInHomeRegionLocal = projectPointsVerticallyToPlanarRegionLocal(homeRegion, nonNavigableExtrusionsInFlatWorld,
                                                                                                                 transformFromWorldToHome);

      Cluster cluster = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      cluster.setTransformToWorld(transformFromHomeRegionToWorld);
      cluster.addRawPointsInWorld(obstacleConcaveHullInWorld);
      cluster.setNavigableExtrusionsInLocal(navigableExtrusionsInHomeRegionLocal);
      cluster.setNonNavigableExtrusionsInLocal(nonNavigableExtrusionsInHomeRegionLocal);

      return cluster;
   }

   private static Cluster createTemporaryClusterWithZEqualZeroAndExtrudeIt(ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator,
                                                                           ArrayList<Point3D> temporaryClusterPoints)
   {
      Cluster tempFlatClusterToExtrude = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      tempFlatClusterToExtrude.addRawPointsInWorld(temporaryClusterPoints);
     
      extrudeObstacleCluster(tempFlatClusterToExtrude, extrusionDistanceCalculator);

      return tempFlatClusterToExtrude;
   }

   private static ArrayList<Point3D> createTemporaryClusterPointsSettingZToHeightDifference(List<Point3D> obstacleConcaveHullInWorld,
                                                                                            List<Point3D> obstacleConcaveHullProjectedToHomeRegion)
   {
      ArrayList<Point3D> temporaryClusterPoints = new ArrayList<>();
      for (int i = 0; i < obstacleConcaveHullInWorld.size(); i++)
      {
         Point3D obstaclePointInWorld = obstacleConcaveHullInWorld.get(i);
         Point3D obstacleProjectedToHomeRegionInWorld = obstacleConcaveHullProjectedToHomeRegion.get(i);

         //TODO: Delete this check after it all works.
         if (Math.abs(obstaclePointInWorld.getX() - obstacleProjectedToHomeRegionInWorld.getX()) > 1e-7)
            throw new RuntimeException();
         if (Math.abs(obstaclePointInWorld.getY() - obstacleProjectedToHomeRegionInWorld.getY()) > 1e-7)
            throw new RuntimeException();

         double obstacleHeight = obstaclePointInWorld.getZ() - obstacleProjectedToHomeRegionInWorld.getZ();

         temporaryClusterPoints.add(new Point3D(obstaclePointInWorld.getX(), obstaclePointInWorld.getY(), obstacleHeight));
      }
      return temporaryClusterPoints;
   }

   private static List<Point2DReadOnly> projectPointsVerticallyToPlanarRegionLocal(PlanarRegion planarRegionToProjectOnto, List<Point2DReadOnly> pointsToProjectInWorld,
                                                                                   RigidBodyTransform transformFromWorldToPlanarRegion)
   {
      List<Point2DReadOnly> navigableExtrusionsInHomeRegionLocal = new ArrayList<>();
      for (int i = 0; i < pointsToProjectInWorld.size(); i++)
      {
         Point2DReadOnly navigableExtrusionInFlatWorld = pointsToProjectInWorld.get(i);
         Point3D navigableExtrusionInFlatWorld3D = new Point3D(navigableExtrusionInFlatWorld);

         Point3D extrudedPointOnHomeRegion = PlanarRegionTools.projectInZToPlanarRegion(navigableExtrusionInFlatWorld3D, planarRegionToProjectOnto);

         transformFromWorldToPlanarRegion.transform(extrudedPointOnHomeRegion);

         //TODO: Verify z = 0 here...
         if (Math.abs(extrudedPointOnHomeRegion.getZ()) > 1e-7)
            throw new RuntimeException();

         Point2D navigableExtrusionInHomeRegionLocal = new Point2D(extrudedPointOnHomeRegion);

         navigableExtrusionsInHomeRegionLocal.add(navigableExtrusionInHomeRegionLocal);
      }

      return navigableExtrusionsInHomeRegionLocal;
   }

   public static void extrudeObstacleCluster(Cluster cluster, ObstacleExtrusionDistanceCalculator calculator)
   {
      ObstacleExtrusionDistanceCalculator nonNavigableCalculator = (p, h) -> calculator.computeExtrusionDistance(p, h) - NAV_TO_NON_NAV_DISTANCE;
      ObstacleExtrusionDistanceCalculator navigableCalculator = calculator;
      int numberOfExtrusionsAtEndpoints = 5;

      switch (cluster.getType())
      {
      case MULTI_LINE:
         cluster.addNonNavigableExtrusionsInLocal(extrudeMultiLine(cluster, nonNavigableCalculator, numberOfExtrusionsAtEndpoints));
         cluster.addNavigableExtrusionsInLocal(extrudeMultiLine(cluster, navigableCalculator, numberOfExtrusionsAtEndpoints));
         cluster.setType(ClusterType.POLYGON);
         break;
      case POLYGON:
         boolean extrudeToTheLeft = cluster.getExtrusionSide() != ExtrusionSide.INSIDE;
         cluster.addNonNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, cluster, nonNavigableCalculator));
         cluster.addNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, cluster, navigableCalculator));
         break;

      default:
         throw new RuntimeException("Unhandled cluster type: " + cluster.getType());
      }
      cluster.updateBoundingBox();
   }

   /**
    * 
    * 
    * @param verticalPolygonVertices
    * @param poppingPointsDistanceSquaredThreshold
    * @return
    */
   static List<Point3D> filterVerticalPolygonForMultiLineExtrusion(List<? extends Point3DReadOnly> verticalPolygonVertices,
                                                                   double poppingPointsDistanceSquaredThreshold)
   {
      // Making a deep copy
      List<Point3D> filteredPoints = verticalPolygonVertices.stream().map(Point3D::new).collect(Collectors.toList());

      if (verticalPolygonVertices.size() <= 2)
         return filteredPoints;

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
         List<Point3D> endpoints = new ArrayList<>();
         endpoints.add(filteredPoints.get(0));
         endpoints.add(filteredPoints.get(filteredPoints.size() - 1));
         endpoints.forEach(p -> p.setZ(maxHeight));
         return endpoints;
      }
      else
      {
         int index = 0;
         // Look for points with same XY-coordinates and only keep the highest one.
         while (index < filteredPoints.size() - 1)
         {
            Point3D pointCurr = filteredPoints.get(index);
            Point3D pointNext = filteredPoints.get(index + 1);

            if (pointCurr.distanceXYSquared(pointNext) < poppingPointsDistanceSquaredThreshold)
               filteredPoints.remove(pointCurr.getZ() <= pointNext.getZ() ? index : index + 1);
            else
               index++;
         }
         return filteredPoints;
      }
   }
}
