package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
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
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.lists.ListWrappingIndexTools;

public class ClusterTools
{
   private static final double NAV_TO_NON_NAV_DISTANCE = 0.01;
   private static final boolean debug = false;

   public static List<Point2D> extrudePolygon(boolean extrudeToTheLeft, Cluster cluster, ExtrusionDistanceCalculator calculator)
   {
      if (cluster.getNumberOfRawPoints() == 2)
      {
         Point2D endpoint0 = cluster.getRawPointInLocal2D(0);
         Point2D endpoint1 = cluster.getRawPointInLocal2D(1);
         double obstacleHeight0 = cluster.getRawPointInLocal3D(0).getZ();
         double obstacleHeight1 = cluster.getRawPointInLocal3D(1).getZ();
         double extrusionDistance0 = calculator.computeExtrusionDistance(cluster, endpoint0, obstacleHeight0);
         double extrusionDistance1 = calculator.computeExtrusionDistance(cluster, endpoint1, obstacleHeight1);

         return extrudeLine(endpoint0, extrusionDistance0, endpoint1, extrusionDistance1, 5);
      }

      List<Point2D> extrusions = new ArrayList<>();

      List<Point2D> rawPoints = cluster.getRawPointsInLocal2D();

      for (int i = 0; i < cluster.getNumberOfRawPoints(); i++)
      {
         Point2D previousPoint = ListWrappingIndexTools.getPrevious(i, rawPoints);
         Point2D pointToExtrude = rawPoints.get(i);
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

      if (cluster.getType() == Type.LINE)
      {
         int numberOfExtrusionsAtEndpoints = 5;
         Point2D endpoint0 = cluster.getRawPointInLocal2D(0);
         Point2D endpoint1 = cluster.getRawPointInLocal2D(1);
         double obstacleHeight0 = cluster.getRawPointInLocal3D(0).getZ();
         double obstacleHeight1 = cluster.getRawPointInLocal3D(1).getZ();

         double extrusionDistance0 = nonNavigableCalculator.computeExtrusionDistance(cluster, endpoint0, obstacleHeight0);
         double extrusionDistance1 = nonNavigableCalculator.computeExtrusionDistance(cluster, endpoint1, obstacleHeight1);
         cluster.addNonNavigableExtrusionsInLocal(extrudeLine(endpoint0, extrusionDistance0, endpoint1, extrusionDistance1, numberOfExtrusionsAtEndpoints));

         extrusionDistance0 = navigableCalculator.computeExtrusionDistance(cluster, endpoint0, obstacleHeight0);
         extrusionDistance1 = navigableCalculator.computeExtrusionDistance(cluster, endpoint1, obstacleHeight1);
         cluster.addNavigableExtrusionsInLocal(extrudeLine(endpoint0, extrusionDistance0, endpoint1, extrusionDistance1, numberOfExtrusionsAtEndpoints));
      }

      if (cluster.getType() == Type.POLYGON)
      {
         boolean extrudeToTheLeft = cluster.getExtrusionSide() != ExtrusionSide.INSIDE;
         cluster.addNonNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, cluster, nonNavigableCalculator));
         cluster.addNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, cluster, navigableCalculator));
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

         for (Point3D point : cluster.getNonNavigableExtrusionsInWorld())
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
                                                List<PlanarRegion> polygonObstacleRegions, List<Cluster> clusters, RigidBodyTransform transformToWorld,
                                                VisibilityGraphsParameters visibilityGraphsParameters)
   {
      for (PlanarRegion region : lineObstacleRegions)
      {
         if (regions.contains(region))
         {
            Cluster cluster = new Cluster();
            clusters.add(cluster);
            cluster.setType(Type.LINE);
            cluster.setTransformToWorld(transformToWorld);

            ArrayList<Point3D> points = new ArrayList<>();
            RigidBodyTransform transToWorld = new RigidBodyTransform();
            region.getTransformToWorld(transToWorld);

            for (int i = 0; i < region.getConvexHull().getNumberOfVertices(); i++)
            {
               Point3D concaveHullVertexWorld = new Point3D(region.getConvexHull().getVertex(i));
               concaveHullVertexWorld.applyTransform(transToWorld);
               points.add(concaveHullVertexWorld);
            }

            LinearRegression3D linearRegression = new LinearRegression3D(points);
            linearRegression.calculateRegression();

            //Convert to local frame
            Point3D[] extremes = linearRegression.getTheTwoPointsFurthestApart();
            cluster.addRawPointsInWorld3D(extremes);
         }
      }

      for (PlanarRegion region : polygonObstacleRegions)
      {
         if (regions.contains(region))
         {
            Cluster cluster = new Cluster();
            clusters.add(cluster);
            cluster.setType(Type.POLYGON);
            cluster.setTransformToWorld(transformToWorld);

            RigidBodyTransform transToWorld = new RigidBodyTransform();
            region.getTransformToWorld(transToWorld);

            for (int i = 0; i < region.getConcaveHullSize(); i++)
            {
               Point3D concaveHullVertexWorld = new Point3D(region.getConcaveHull()[i]);
               concaveHullVertexWorld.applyTransform(transToWorld);
               cluster.addRawPointInWorld(concaveHullVertexWorld);
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

   public static interface ExtrusionDistanceCalculator
   {
      /**
       * @param clusterInExtrusion the cluster being extruded. Do not modify.
       * @param pointToExtrude the coordinates of the point being extruded. Do not modify.
       * @param obstacleHeight the height of the obstacle from which the point to extrude is
       *           created.
       * @return positive value representing the ditance between the raw points of a cluster and the
       *         extrusion.
       */
      double computeExtrusionDistance(Cluster clusterInExtrusion, Point2DReadOnly pointToExtrude, double obstacleHeight);
   }
}
