package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.robotics.lists.ListWrappingIndexTools;

public class ClusterTools
{
   private static final double POPPING_POLYGON_POINTS_THRESHOLD = 0.0; //MathTools.square(0.025);
   private static final double POPPING_MULTILINE_POINTS_THRESHOLD = MathTools.square(0.20);
   private static final double NAV_TO_NON_NAV_DISTANCE = 0.01;
   private static final boolean debug = false;

   public static List<Point2D> extrudePolygon(boolean extrudeToTheLeft, Cluster cluster, ExtrusionDistanceCalculator calculator)
   {
      if (cluster.getNumberOfRawPoints() == 2)
      {
         return extrudeMultiLine(cluster, calculator, 5);
      }

      List<Point2D> extrusions = new ArrayList<>();

      List<Point2D> rawPoints = cluster.getRawPointsInLocal2D();

      for (int i = 0; i < cluster.getNumberOfRawPoints(); i++)
      {
         Point2D previousPoint = ListWrappingIndexTools.getPrevious(i, rawPoints);
         Point2D pointToExtrude = rawPoints.get(i);

         if (pointToExtrude.distanceSquared(previousPoint) < POPPING_POLYGON_POINTS_THRESHOLD)
            continue;

         Point2D nextPoint = ListWrappingIndexTools.getNext(i, rawPoints);

         double obstacleHeight = cluster.getRawPointInLocal3D(i).getZ();

         Line2D edgePrev = new Line2D(previousPoint, pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, nextPoint);

         double extrusionDistance = calculator.computeExtrusionDistance(cluster, pointToExtrude, obstacleHeight);

         boolean shouldExtrudeCorner;

         if (extrudeToTheLeft)
            shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -0.5 * Math.PI;
         else
            shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) >= 0.5 * Math.PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, extrudeToTheLeft, 3, extrusionDistance));
         }
         else
         {
            Vector2D extrusionDirection = new Vector2D();
            extrusionDirection.interpolate(edgePrev.getDirection(), edgeNext.getDirection(), 0.5);
            extrusionDirection = EuclidGeometryTools.perpendicularVector2D(extrusionDirection);
            if (!extrudeToTheLeft)
               extrusionDirection.negate();

            Point2D extrusion = new Point2D();
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);
            extrusions.add(extrusion);
         }
      }

      if (!extrusions.isEmpty())
         extrusions.add(extrusions.get(0));

      return extrusions;
   }

   public static List<Point2D> extrudeMultiLine(Cluster cluster, ExtrusionDistanceCalculator calculator, int numberOfExtrusionsAtEndpoints)
   {
      List<Point2D> extrusions = new ArrayList<>();
      List<Point2D> rawPoints = cluster.getRawPointsInLocal2D();

      if (rawPoints.size() >= 2)
      { // Start
         Point2D pointToExtrude = rawPoints.get(0);
         double obstacleHeight = cluster.getRawPointInLocal3D(0).getZ();

         Line2D edgePrev = new Line2D(rawPoints.get(1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, rawPoints.get(1));
         double extrusionDistance = calculator.computeExtrusionDistance(cluster, pointToExtrude, obstacleHeight);
         extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = 1; i < rawPoints.size() - 1; i++)
      { // Go from start to end
         Point2D pointToExtrude = rawPoints.get(i);
         double obstacleHeight = cluster.getRawPointInLocal3D(i).getZ();

         Line2D edgePrev = new Line2D(rawPoints.get(i - 1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, rawPoints.get(i + 1));

         boolean shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -0.5 * Math.PI;
         double extrusionDistance = calculator.computeExtrusionDistance(cluster, pointToExtrude, obstacleHeight);

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            Vector2D extrusionDirection = new Vector2D();
            extrusionDirection.interpolate(edgePrev.getDirection(), edgeNext.getDirection(), 0.5);
            extrusionDirection.normalize();
            extrusionDirection = EuclidGeometryTools.perpendicularVector2D(extrusionDirection);

            Point2D extrusion = new Point2D();
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);
            extrusions.add(extrusion);
         }
      }

      if (rawPoints.size() >= 2)
      { // End
         int lastIndex = rawPoints.size() - 1;
         Point2D pointToExtrude = rawPoints.get(lastIndex);
         double obstacleHeight = cluster.getRawPointInLocal3D(lastIndex).getZ();

         Line2D edgePrev = new Line2D(rawPoints.get(lastIndex - 1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, rawPoints.get(lastIndex - 1));
         double extrusionDistance = calculator.computeExtrusionDistance(cluster, pointToExtrude, obstacleHeight);
         extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = rawPoints.size() - 2; i >= 1; i--)
      { // Go from end back to start
         Point2D pointToExtrude = rawPoints.get(i);
         double obstacleHeight = cluster.getRawPointInLocal3D(i).getZ();

         Line2D edgePrev = new Line2D(rawPoints.get(i + 1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, rawPoints.get(i - 1));

         boolean shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -0.5 * Math.PI;
         double extrusionDistance = calculator.computeExtrusionDistance(cluster, pointToExtrude, obstacleHeight);

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            Vector2D extrusionDirection = new Vector2D();
            extrusionDirection.interpolate(edgePrev.getDirection(), edgeNext.getDirection(), 0.5);
            extrusionDirection.normalize();
            extrusionDirection = EuclidGeometryTools.perpendicularVector2D(extrusionDirection);

            Point2D extrusion = new Point2D();
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);
            extrusions.add(extrusion);
         }
      }

      if (!extrusions.isEmpty())
         extrusions.add(extrusions.get(0));

      return extrusions;
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

      List<Point2D> extrusions1 = extrudeCorner(endpoint1, edge2, edge1, true, numberOfExtrusionsAtEndpoints, extrusionDistance1);
      List<Point2D> extrusions2 = extrudeCorner(endpoint2, edge1, edge2, true, numberOfExtrusionsAtEndpoints, extrusionDistance2);
      extrusions.addAll(extrusions1);
      extrusions.addAll(extrusions2);
      extrusions.add(extrusions1.get(0));

      return extrusions;
   }

   public static List<Point2D> extrudeCorner(Point2DReadOnly cornerPointToExtrude, Line2D previousEdge, Line2D nextEdge, boolean extrudeToTheLeft,
                                             int numberOfExtrusions, double extrusionDistance)
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

   public static void extrudeCluster(Cluster cluster, Point2DReadOnly observer, ExtrusionDistanceCalculator calculator, List<Cluster> listOfClusters)
   {
      ExtrusionDistanceCalculator nonNavigableCalculator = (c, p, h) -> calculator.computeExtrusionDistance(c, p, h) - NAV_TO_NON_NAV_DISTANCE;
      ExtrusionDistanceCalculator navigableCalculator = (c, p, h) -> calculator.computeExtrusionDistance(c, p, h);
      int numberOfExtrusionsAtEndpoints = 5;

      switch (cluster.getType())
      {
      case LINE:
      case MULTI_LINE:
         cluster.addNonNavigableExtrusionsInLocal2D(extrudeMultiLine(cluster, nonNavigableCalculator, numberOfExtrusionsAtEndpoints));
         cluster.addNavigableExtrusionsInLocal2D(extrudeMultiLine(cluster, navigableCalculator, numberOfExtrusionsAtEndpoints));
         break;
      case POLYGON:
         boolean extrudeToTheLeft = cluster.getExtrusionSide() != ExtrusionSide.INSIDE;
         cluster.addNonNavigableExtrusionsInLocal2D(extrudePolygon(extrudeToTheLeft, cluster, nonNavigableCalculator));
         cluster.addNavigableExtrusionsInLocal2D(extrudePolygon(extrudeToTheLeft, cluster, navigableCalculator));
         break;

      default:
         throw new RuntimeException("Unhandled cluster type: " + cluster.getType());
      }
   }

   public static void classifyExtrusions(List<PlanarRegion> regionsToProject, PlanarRegion regionToProjectTo, List<PlanarRegion> lineObstaclesToPack,
                                         List<PlanarRegion> polygonObstaclesToPack, double zNormalThreshold)
   {

      for (PlanarRegion regionToProject : regionsToProject)
      {
         Vector3D normal = PlanarRegionTools.calculateNormal(regionToProject);

         if (normal != null && regionToProject != regionToProjectTo)
         {
            if (Math.abs(normal.getZ()) < zNormalThreshold)
            {
               lineObstaclesToPack.add(regionToProject);
            }
            else
            {
               polygonObstaclesToPack.add(regionToProject);
            }
         }
      }
   }

   public static Cluster getTheClosestCluster(Point3DReadOnly pointToSortFrom, List<Cluster> clusters)
   {
      double minDistance = Double.MAX_VALUE;
      Cluster closestCluster = null;

      for (Cluster cluster : clusters)
      {
         double distOfPoint = Double.MAX_VALUE;
         Point3D closestPointInCluster = null;

         for (Point3D point : cluster.getNonNavigableExtrusionsInWorld3D())
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
         if (PlanarRegionTools.isPointInWorldInsideARegion(region, point))
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

   public static void createClusterForHomeRegion(List<Cluster> clusters, RigidBodyTransform transformToWorld, PlanarRegion homeRegion, double extrusionDistance)
   {
      Cluster cluster = new Cluster();
      clusters.add(cluster);
      cluster.setType(Type.POLYGON);
      cluster.setTransformToWorld(transformToWorld);
      cluster.setHomeRegion(true);
      cluster.addRawPointsInLocal2D(homeRegion.getConcaveHull());
      cluster.setExtrusionSide(ExtrusionSide.INSIDE);
   }

   public static void createClustersFromRegions(PlanarRegion homeRegion, List<PlanarRegion> regions, List<PlanarRegion> lineObstacleRegions,
                                                List<PlanarRegion> polygonObstacleRegions, List<Cluster> clusters, RigidBodyTransform transformFromHomeToWorld,
                                                VisibilityGraphsParameters visibilityGraphsParameters)
   {
      Point3D mean = new Point3D();
      Vector3D principalVector = new Vector3D();
      PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

      for (PlanarRegion region : lineObstacleRegions)
      {
         if (regions.contains(region))
         {
            Cluster cluster = new Cluster();
            clusters.add(cluster);
            cluster.setType(Type.MULTI_LINE);
            cluster.setExtrusionSide(ExtrusionSide.OUTSIDE);
            cluster.setTransformToWorld(transformFromHomeToWorld);

            List<Point3D> rawPoints = new ArrayList<>();
            RigidBodyTransform transformFromOtherToHome = new RigidBodyTransform();
            region.getTransformToWorld(transformFromOtherToHome);
            transformFromOtherToHome.preMultiplyInvertOther(transformFromHomeToWorld);

            for (int i = 0; i < region.getConvexHull().getNumberOfVertices(); i++)
            {
               Point3D concaveHullVertexHome = new Point3D(region.getConvexHull().getVertex(i));
               concaveHullVertexHome.applyTransform(transformFromOtherToHome);
               rawPoints.add(concaveHullVertexHome);
            }

            if (rawPoints.size() <= 2)
            {
               cluster.addRawPointsInLocal3D(rawPoints);
               continue;
            }

            pca.clear();
            rawPoints.forEach(p -> pca.addPoint(p.getX(), p.getY(), 0.0));
            pca.compute();
            pca.getMean(mean);
            pca.getPrincipalVector(principalVector);
            Line2D line = new Line2D(new Point2D(mean), new Vector2D(principalVector));
            // Adjust the XY-coordinates to be on the line
            rawPoints.stream().forEach(p -> p.set(line.orthogonalProjectionCopy(new Point2D(p))));
            // Sort the points given their position on the line.
            Collections.sort(rawPoints, (p1, p2) -> {
               double t1 = line.parameterGivenPointOnLine(new Point2D(p1), Double.POSITIVE_INFINITY);
               double t2 = line.parameterGivenPointOnLine(new Point2D(p2), Double.POSITIVE_INFINITY);
               return t1 >= t2 ? 1 : -1;
            });

            if (rawPoints.get(0).distanceXY(rawPoints.get(rawPoints.size() - 1)) <= POPPING_MULTILINE_POINTS_THRESHOLD)
            {
               cluster.addRawPointInLocal3D(rawPoints.get(0));
               cluster.addRawPointInLocal3D(rawPoints.get(rawPoints.size() - 1));
            }
            else
            {
               int index = 0;
               // Look for points with same XY-coordinates and only keep the highest one.
               while (index < rawPoints.size() - 1)
               {
                  Point3D pointCurr = rawPoints.get(index);
                  Point3D pointNext = rawPoints.get(index + 1);

                  if (pointCurr.distanceXYSquared(pointNext) < POPPING_MULTILINE_POINTS_THRESHOLD)
                     rawPoints.remove(pointCurr.getZ() <= pointNext.getZ() ? index : index + 1);
                  else
                     index++;
               }
               cluster.addRawPointsInLocal3D(rawPoints);
            }
         }
      }

      for (PlanarRegion region : polygonObstacleRegions)
      {
         if (regions.contains(region))
         {
            Cluster cluster = new Cluster();
            clusters.add(cluster);
            cluster.setType(Type.POLYGON);
            cluster.setExtrusionSide(ExtrusionSide.OUTSIDE);
            cluster.setTransformToWorld(transformFromHomeToWorld);

            RigidBodyTransform transformFromOtherToHome = new RigidBodyTransform();
            region.getTransformToWorld(transformFromOtherToHome);
            transformFromOtherToHome.preMultiplyInvertOther(transformFromHomeToWorld);

            for (int i = 0; i < region.getConcaveHullSize(); i++)
            {
               Point3D concaveHullVertexHome = new Point3D(region.getConcaveHull()[i]);
               concaveHullVertexHome.applyTransform(transformFromOtherToHome);
               cluster.addRawPointInLocal3D(concaveHullVertexHome);
            }
         }
      }

      if (debug)
      {
         for (Cluster cluster : clusters)
         {
            System.out.println("Created a cluster of type: " + cluster.getType() + " with " + cluster.getRawPointsInLocal2D().size() + " points");
         }
      }
   }

   public static void performExtrusions(Point2D initialObserver, ExtrusionDistanceCalculator calculator, List<Cluster> clusters)
   {
      for (Cluster cluster : clusters)
      {
         ClusterTools.extrudeCluster(cluster, initialObserver, calculator, clusters);
      }
   }
}
