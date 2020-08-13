package us.ihmc.robotEnvironmentAwareness.slam.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.mutable.MutableDouble;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.jOctoMap.tools.OcTreeNearestNeighborTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;

public class SLAMTools
{
   public static Scan toScan(Point3DReadOnly[] points, Tuple3DReadOnly sensorPosition)
   {
      PointCloud pointCloud = new PointCloud();

      for (int i = 0; i < points.length; i++)
      {
         double x = points[i].getX();
         double y = points[i].getY();
         double z = points[i].getZ();
         pointCloud.add(x, y, z);
      }
      return new Scan(new Point3D(sensorPosition), pointCloud);
   }

   public static Scan toScan(Point3DReadOnly[] points, Tuple3DReadOnly sensorPosition, ConvexPolygon2DReadOnly mapHull, double windowMargin)
   {
      PointCloud pointCloud = new PointCloud();

      for (int i = 0; i < points.length; i++)
      {
         if (!mapHull.getBoundingBox().isInsideEpsilon(points[i].getX(), points[i].getY(), -windowMargin))
            continue;

         if (mapHull.isPointInside(points[i].getX(), points[i].getY(), -windowMargin))
         {
            double x = points[i].getX();
            double y = points[i].getY();
            double z = points[i].getZ();
            pointCloud.add(x, y, z);
         }
      }
      return new Scan(new Point3D(sensorPosition), pointCloud);
   }

   public static Point3D[] createConvertedPointsToSensorPose(RigidBodyTransformReadOnly sensorPose, Point3DReadOnly[] pointCloud)
   {
      Point3D[] convertedPoints = new Point3D[pointCloud.length];
      for (int i = 0; i < pointCloud.length; i++)
      {
         convertedPoints[i] = createConvertedPointToSensorPose(sensorPose, pointCloud[i]);
      }

      return convertedPoints;
   }

   public static Plane3D createConvertedSurfaceElementToSensorPose(RigidBodyTransformReadOnly sensorPose, Plane3DReadOnly surfaceElement)
   {
      Plane3D convertedSurfel = new Plane3D();
      sensorPose.inverseTransform(surfaceElement.getPoint(), convertedSurfel.getPoint());
      sensorPose.inverseTransform(surfaceElement.getNormal(), convertedSurfel.getNormal());

      return convertedSurfel;
   }

   public static Point3D createConvertedPointToSensorPose(RigidBodyTransformReadOnly sensorPose, Point3DReadOnly point)
   {
      Point3D convertedPoint = new Point3D();
      sensorPose.inverseTransform(point, convertedPoint);
      return convertedPoint;
   }

   public static NormalOcTree computeOctreeData(Point3DReadOnly[] pointCloud, Tuple3DReadOnly sensorPosition, double octreeResolution)
   {
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = pointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(toScan(pointCloud, sensorPosition));

      NormalOcTree octree = new NormalOcTree(octreeResolution);

      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);

      octree.updateNormals();
      return octree;
   }

   public static double computeDistancePointToNormalOctree(NormalOcTree octree, Point3DReadOnly point)
   {
      OcTreeKey occupiedKey = octree.coordinateToKey(point);
      OcTreeKey nearestKey = new OcTreeKey();
      OcTreeNearestNeighborTools.findNearestNeighbor(octree.getRoot(), octree.keyToCoordinate(occupiedKey), nearestKey);

      MutableDouble nearestHitDistanceSquared = new MutableDouble(Double.POSITIVE_INFINITY);

      double resolution = octree.getResolution();
      OcTreeNearestNeighborTools.findRadiusNeighbors(octree.getRoot(), octree.keyToCoordinate(nearestKey), resolution * 1.5, node ->
      {
         nearestHitDistanceSquared.setValue(Math.min(nearestHitDistanceSquared.doubleValue(), node.getHitLocationCopy().distanceSquared(point)));
      });

      return Math.sqrt(nearestHitDistanceSquared.getValue());
   }

   public static double computePerpendicularDistancePointToNormalOctree(NormalOcTree octree, Point3DReadOnly point)
   {
      OcTreeKey occupiedKey = octree.coordinateToKey(point);
      OcTreeKey nearestKey = new OcTreeKey();
      OcTreeNearestNeighborTools.findNearestNeighbor(octree.getRoot(), octree.keyToCoordinate(occupiedKey), nearestKey);

      MutableDouble nearestHitDistanceSquared = new MutableDouble(Double.POSITIVE_INFINITY);

      double resolution = octree.getResolution();
      OcTreeNearestNeighborTools.findRadiusNeighbors(octree.getRoot(), octree.keyToCoordinate(nearestKey), resolution * 2, node ->
      {
         nearestHitDistanceSquared.setValue(Math.min(nearestHitDistanceSquared.doubleValue(),
                                                     EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, node.getHitLocationCopy(), node.getNormalCopy())));
      });

      return nearestHitDistanceSquared.getValue();
   }

   public static double computeBoundedPerpendicularDistancePointToNormalOctree(NormalOcTree octree, Point3DReadOnly point, double bound)
   {
      OcTreeKey occupiedKey = octree.coordinateToKey(point);
      OcTreeKey nearestKey = new OcTreeKey();
      OcTreeNearestNeighborTools.findNearestNeighbor(octree.getRoot(), octree.keyToCoordinate(occupiedKey), nearestKey);

      MutableDouble nearestHitDistanceSquared = new MutableDouble(Double.POSITIVE_INFINITY);

      double resolution = octree.getResolution();
      OcTreeNearestNeighborTools.findRadiusNeighbors(octree.getRoot(), octree.keyToCoordinate(nearestKey), resolution * 2, node ->
      {
         double linearDistance = point.distance(node.getHitLocationCopy());

         double distance = linearDistance;

         if (linearDistance < bound)
         {
            double distanceToSurfel = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, node.getHitLocationCopy(), node.getNormalCopy());
            distance = distanceToSurfel;
         }

         nearestHitDistanceSquared.setValue(Math.min(nearestHitDistanceSquared.doubleValue(), distance));
      });

      return nearestHitDistanceSquared.getValue();
   }

   /**
    * Computes the convex hull of all the {@code mapOctree} nodes in the sensor frame (z-axis is
    * depth).
    */
   public static ConvexPolygon2D computeMapConvexHullInSensorFrame(NormalOcTree mapOctree, RigidBodyTransformReadOnly sensorPose)
   {
      List<Point3D> vertex = new ArrayList<>();

      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createIterable(mapOctree.getRoot());
      // TODO Consider using a bounding box as follows:
      //      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createLeafBoundingBoxIteratable(octree.getRoot(), boundingBox);

      for (NormalOcTreeNode node : iterable)
      {
         Point3D hitLocation = node.getHitLocationCopy();
         sensorPose.inverseTransform(hitLocation);
         vertex.add(hitLocation);
      }
      Vertex3DSupplier supplier = Vertex3DSupplier.asVertex3DSupplier(vertex);
      return new ConvexPolygon2D(supplier);
   }
}
