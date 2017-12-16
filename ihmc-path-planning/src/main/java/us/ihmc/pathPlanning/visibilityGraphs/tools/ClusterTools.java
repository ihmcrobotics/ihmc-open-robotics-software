package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.robotics.geometry.PlanarRegion;

public class ClusterTools
{
   private static final boolean debug = false;

   public static int determineExtrusionSide(Cluster cluster, Point2DReadOnly observer)
   {
      int index = 0;

      for (int i = 0; i < cluster.getNumberOfNormals(); i++)
      {
         if (isNormalVisible(cluster, i, observer))
         {
            index = i;
            break;
         }
      }

      if (index % 2 == 0)
      {
         index = 0;
      }
      else
      {
         index = 1;
      }

      return index;
   }

   private static boolean isNormalVisible(Cluster cluster, int normalIndex, Point2DReadOnly observer)
   {
      List<Point2D> rawPointsInLocal = cluster.getRawPointsInLocal2D();
      for (int i = 1; i < rawPointsInLocal.size(); i++)
      {
         Point2D target = new Point2D(cluster.getNormalInLocal(normalIndex));

         Point2D startPt = rawPointsInLocal.get(i - 1);
         Point2D endPt = rawPointsInLocal.get(i);

         if (EuclidGeometryTools.doLineSegment2DsIntersect(observer, target, startPt, endPt))
         {
            return false;
         }
      }
      return true;
   }

   public static void extrudedNonNavigableBoundary(int index, Cluster cluster, double extrusionDistance)
   {
      for (int i = 0; i < cluster.getRawPointsInLocal2D().size() - 2; i++)
      {
         Point2D point1 = cluster.getRawPointInLocal2D(i);
         Point2D point2 = cluster.getRawPointInLocal2D(i + 1);
         Point2D point3 = cluster.getRawPointInLocal2D(i + 2);

         Vector2D vec1 = new Vector2D();
         Vector2D vec2 = new Vector2D();
         vec1.sub(point2, point1);
         vec2.sub(point3, point2);

         Point2D normal1 = cluster.getSafeNormalInLocal(index);
         Point2D normal2 = cluster.getSafeNormalInLocal(index + 2);

         Point2D intersectionPoint = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(normal1, vec1, normal2, vec2);

         if (intersectionPoint == null)
         {
            if (debug)
               PrintTools.error("Failed to extrude non-navigable boundary for region " + index + " \n" + "point1: " + point1 + "\n" + "point2: " + point2 + "\n"
                     + "point3: " + point3 + "\n" + "vec1: " + vec1 + "\n" + "vec2: " + vec2 + "\n" + "normal1: " + normal1 + "\n" + "normal2: " + normal2
                     + "\n" + "extrusionDistance: " + extrusionDistance);
            continue;
         }

         if (intersectionPoint.distance(normal1) < 1E-6)
         {
            intersectionPoint.interpolate(normal1, normal2, 0.5);
         }

         Vector2D directionOfIntersectionExtrusion = new Vector2D();
         Point2D adjustedIntersection = new Point2D();

         directionOfIntersectionExtrusion.sub(intersectionPoint, point2);
         directionOfIntersectionExtrusion.normalize();

         adjustedIntersection.scaleAdd(extrusionDistance, directionOfIntersectionExtrusion, point2);

         cluster.addNonNavigableExtrusionInLocal(adjustedIntersection);

         index = index + 2;
      }

      if (cluster.isObstacleClosed() && !cluster.getNonNavigableExtrusionsInLocal().isEmpty())
      {
         cluster.addNonNavigableExtrusionInLocal(cluster.getNonNavigableExtrusionsInLocal().get(0));
      }
   }

   public static void extrudedNavigableBoundary(int index, Cluster cluster, double extrusionDistance)
   {
      for (int i = 0; i < cluster.getRawPointsInLocal2D().size() - 2; i++)
      {
         Point2D point1 = cluster.getRawPointInLocal2D(i);
         Point2D point2 = cluster.getRawPointInLocal2D(i + 1);
         Point2D point3 = cluster.getRawPointInLocal2D(i + 2);

         Vector2D vec1 = new Vector2D();
         Vector2D vec2 = new Vector2D();
         vec1.sub(point2, point1);
         vec2.sub(point3, point2);

         Point2D normal1 = cluster.getSafeNormalInLocal(index);
         Point2D normal2 = cluster.getSafeNormalInLocal(index + 2);

         Point2D intersectionPoint = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(normal1, vec1, normal2, vec2);

         if (intersectionPoint == null)
         {
            if (debug)
               PrintTools.error("Failed to extrude navigable boundary for region " + index + " \n" + "point1: " + point1 + "\n" + "point2: " + point2 + "\n"
                     + "point3: " + point3 + "\n" + "vec1: " + vec1 + "\n" + "vec2: " + vec2 + "\n" + "normal1: " + normal1 + "\n" + "normal2: " + normal2
                     + "\n" + "extrusionDistance: " + extrusionDistance);
            continue;
         }

         if (intersectionPoint.distance(normal1) < 1E-6)
         {
            intersectionPoint.interpolate(normal1, normal2, 0.5);
         }

         Vector2D normalIntersection = new Vector2D();
         normalIntersection.sub(intersectionPoint, point2);
         normalIntersection.normalize();

         Point2D adjustedIntersection = new Point2D();
         adjustedIntersection.scaleAdd(extrusionDistance, normalIntersection, point2);

         cluster.addNavigableExtrusionInLocal(adjustedIntersection);

         index = index + 2;
      }

      if (cluster.isObstacleClosed() && !cluster.getNavigableExtrusionsInLocal().isEmpty())
      {
         cluster.addNavigableExtrusionInLocal(cluster.getNavigableExtrusionInLocal(0));
      }

   }

   public static List<Point2D> extrudeLine(Point2DReadOnly endpoint1, Point2DReadOnly endpoint2, double extrusionDistance)
   {
      ArrayList<Point2D> points = new ArrayList<>();

      Vector2D lineDirection = new Vector2D();
      Point2D endExtrusion1 = new Point2D();
      Point2D endExtrusion2 = new Point2D();

      lineDirection.sub(endpoint2, endpoint1);
      lineDirection.normalize();

      endExtrusion1.scaleAdd(extrusionDistance, lineDirection, endpoint2);
      endExtrusion2.scaleAdd(-extrusionDistance, lineDirection, endpoint1);

      List<Point2D> perpendicularBisectorSegment2D = EuclidGeometryTools.perpendicularBisectorSegment2D(endpoint1, endpoint2, extrusionDistance);
      Point2D midNormal1 = perpendicularBisectorSegment2D.get(0);
      Point2D midNormal2 = perpendicularBisectorSegment2D.get(1);

      points.add(endExtrusion2);
      points.add(extrudeCorner(endpoint1, lineDirection, endExtrusion2, midNormal1, extrusionDistance));
      points.add(midNormal1);
      points.add(extrudeCorner(endpoint2, lineDirection, endExtrusion1, midNormal1, extrusionDistance));
      points.add(endExtrusion1);
      points.add(extrudeCorner(endpoint2, lineDirection, endExtrusion1, midNormal2, extrusionDistance));
      points.add(midNormal2);
      points.add(extrudeCorner(endpoint1, lineDirection, endExtrusion2, midNormal2, extrusionDistance));
      points.add(endExtrusion2);

      return points;
   }

   // FIXME That method is terrible
   private static Point2D extrudeCorner(Point2DReadOnly pointOnLine, Vector2DReadOnly cornerNormal, Point2DReadOnly extrudedPoint1,
                                        Point2DReadOnly extrudedPoint2, double extrusion)
   {
      Vector2D cornerTangent = EuclidGeometryTools.perpendicularVector2D(cornerNormal);
      Vector2D vecExtrToCorner = new Vector2D();

      Point2D inter1 = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(extrudedPoint1, cornerTangent, extrudedPoint2, cornerNormal);

      vecExtrToCorner.sub(inter1, pointOnLine);
      vecExtrToCorner.normalize();

      Point2D extrusion1 = new Point2D();
      extrusion1.scaleAdd(extrusion, vecExtrToCorner, pointOnLine);

      return extrusion1;
   }

   public static void extrudeCluster(Cluster cluster, Point2DReadOnly observer, double extrusionDistance, List<Cluster> listOfClusters)
   {
      int extrusionIndex = 0;
      if (cluster.getType() == Type.LINE)
      {
         double extrusionDist1 = extrusionDistance - 0.01 + cluster.getAdditionalExtrusionDistance();
         double extrusionDist2 = extrusionDistance + cluster.getAdditionalExtrusionDistance();

         List<Point2D> nonNavExtrusions = ClusterTools.extrudeLine(cluster.getRawPointInLocal2D(0), cluster.getRawPointInLocal2D(1), extrusionDist1);
         List<Point2D> navExtrusions = ClusterTools.extrudeLine(cluster.getRawPointInLocal2D(0), cluster.getRawPointInLocal2D(1), extrusionDist2);

         cluster.addNonNavigableExtrusionsInLocal(nonNavExtrusions);
         cluster.addNavigableExtrusionsInLocal(navExtrusions);
      }

      if (cluster.getType() == Type.POLYGON)
      {
         //                  System.out.println("Extruding Polygon");
         generateNormalsFromRawBoundaryMap(extrusionDistance, listOfClusters);

         if (cluster.isObstacleClosed() && cluster.getExtrusionSide() != ExtrusionSide.AUTO)
         {
            if (cluster.getExtrusionSide() == ExtrusionSide.INSIDE)
            {
               extrusionIndex = 1;
            }
            else
            {
               extrusionIndex = 0;
            }
         }
         else
         {
            extrusionIndex = ClusterTools.determineExtrusionSide(cluster, observer);
         }

         extrudePolygon(cluster, extrusionIndex, extrusionDistance);
      }
   }

   public static void extrudePolygon(Cluster cluster, int extrusionIndex, double extrusionDistance)
   {
      extrusionDistance = extrusionDistance + cluster.getAdditionalExtrusionDistance();

      double extrusionDist1 = extrusionDistance - 0.01;
      double extrusionDist2 = extrusionDistance;

      ClusterTools.extrudedNonNavigableBoundary(extrusionIndex, cluster, extrusionDist1);
      ClusterTools.extrudedNavigableBoundary(extrusionIndex, cluster, extrusionDist2);
   }

   public static void generateNormalsFromRawBoundaryMap(double extrusionDistance, List<Cluster> listOfClusters)
   {
      for (Cluster cluster : listOfClusters)
      {
         List<Point2D> rawPoints = cluster.getRawPointsInLocal2D();
         for (int i = 0; i < rawPoints.size() - 1; i++)
         {
            Point2D first = rawPoints.get(i);
            Point2D second = rawPoints.get(i + 1);
            generateNormalsForSegment(first, second, cluster, extrusionDistance);
         }
      }
   }

   public static void generateNormalsForSegment(Point2DReadOnly first, Point2DReadOnly second, Cluster cluster, double extrusionDistance)
   {
      List<Point2D> points = EuclidGeometryTools.perpendicularBisectorSegment2D(first, second, 0.001);

      for (Point2D normalPoint : points)
      {
         cluster.addNormalInLocal(normalPoint);
      }

      points = EuclidGeometryTools.perpendicularBisectorSegment2D(first, second, extrusionDistance + cluster.getAdditionalExtrusionDistance());

      for (Point2D normalPoint : points)
      {
         cluster.addSafeNormalInLocal(normalPoint);
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
      cluster.addRawPointsInLocal2D(homeRegion.getConcaveHull(), true);
      cluster.setExtrusionSide(ExtrusionSide.INSIDE);
      cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.1));
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

            if (PlanarRegionTools.isRegionTooHighToStep(region, homeRegion, visibilityGraphsParameters.getTooHighToStepDistance()))
            {
               cluster.setAdditionalExtrusionDistance(0);
            }
            else
            {
               cluster.setAdditionalExtrusionDistance(visibilityGraphsParameters.getExtrusionDistanceIfNotTooHighToStep()
                     - visibilityGraphsParameters.getExtrusionDistance());
            }

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
            cluster.addRawPointsInWorld3D(extremes, false);
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

            Vector3D normal1 = PlanarRegionTools.calculateNormal(region);
            if (Math.abs(normal1.getZ()) >= 0.5) //if its closer to being flat you can probably step on it -->> extrude less
            {
               if (PlanarRegionTools.isRegionTooHighToStep(region, homeRegion, visibilityGraphsParameters.getTooHighToStepDistance())) //is flat but too high to step so its an obstacle
               {
                  cluster.setAdditionalExtrusionDistance(0);
               }
               else
               {
                  cluster.setAdditionalExtrusionDistance(visibilityGraphsParameters.getExtrusionDistanceIfNotTooHighToStep()
                        - visibilityGraphsParameters.getExtrusionDistance());
               }
            }

            RigidBodyTransform transToWorld = new RigidBodyTransform();
            region.getTransformToWorld(transToWorld);

            for (int i = 0; i < region.getConcaveHullSize(); i++)
            {
               Point3D concaveHullVertexWorld = new Point3D(region.getConcaveHull()[i]);
               concaveHullVertexWorld.applyTransform(transToWorld);
               cluster.addRawPointInWorld(concaveHullVertexWorld);
            }

            cluster.setClusterClosure(true);
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

   public static void performExtrusions(Point2D initialObserver, double extrusionDistance, List<Cluster> clusters)
   {
      for (Cluster cluster : clusters)
      {
         ClusterTools.extrudeCluster(cluster, initialObserver, extrusionDistance, clusters);
      }
   }

}
