package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class EnvironmentMappingTools
{
   public static StereoVisionPointCloudMessage createBoundingBoxFilteredMessage(StereoVisionPointCloudMessage originalMessage,
                                                                                BoundingBoxParametersMessage boundingBox)
   {
      StereoVisionPointCloudMessage filteredMessage = new StereoVisionPointCloudMessage();
      filteredMessage.getSensorPosition().set(originalMessage.getSensorPosition());
      filteredMessage.getSensorOrientation().set(originalMessage.getSensorOrientation());

      RigidBodyTransform transformWorldToSensor = new RigidBodyTransform(originalMessage.getSensorOrientation(), originalMessage.getSensorPosition());
      transformWorldToSensor.invert();

      int numberOfPoints = originalMessage.getColors().size();
      Point3D point = new Point3D();
      Point3D convertedPoint = new Point3D();
      for (int i = 0; i < numberOfPoints; i++)
      {
         Float pointCloud = originalMessage.getPointCloud();
         point.setX(pointCloud.get(i * 3 + 0));
         point.setY(pointCloud.get(i * 3 + 1));
         point.setZ(pointCloud.get(i * 3 + 2));
         convertedPoint.set(point);
         transformWorldToSensor.transform(convertedPoint);
         if (boundingBox.getMinX() < convertedPoint.getX() && convertedPoint.getX() < boundingBox.getMaxX())
         {
            if (boundingBox.getMinY() < convertedPoint.getY() && convertedPoint.getY() < boundingBox.getMaxY())
            {
               if (boundingBox.getMinZ() < convertedPoint.getZ() && convertedPoint.getZ() < boundingBox.getMaxZ())
               {
                  filteredMessage.getColors().add(originalMessage.getColors().get(i));
                  filteredMessage.getPointCloud().add(originalMessage.getPointCloud().get(i * 3 + 0));
                  filteredMessage.getPointCloud().add(originalMessage.getPointCloud().get(i * 3 + 1));
                  filteredMessage.getPointCloud().add(originalMessage.getPointCloud().get(i * 3 + 2));
               }
            }
         }
      }

      return filteredMessage;
   }

   private static Scan toScan(Float data, Point3DReadOnly sensorPosition)
   {
      PointCloud pointCloud = new PointCloud();

      int bufferIndex = 0;

      while (bufferIndex < data.size())
      {
         float x = data.getQuick(bufferIndex++);
         float y = data.getQuick(bufferIndex++);
         float z = data.getQuick(bufferIndex++);
         pointCloud.add(x, y, z);
      }
      return new Scan(sensorPosition, pointCloud);
   }

   private static Scan toScan(Point3D[] points, Tuple3DReadOnly sensorPosition)
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

   public static List<Plane3D> createOctreeNodePlanes(NormalOcTree normalOcTree)
   {
      List<Plane3D> octreeNodePlanes = new ArrayList<Plane3D>();

      NormalOcTreeMessage normalOctreeMessage = OcTreeMessageConverter.convertToMessage(normalOcTree);
      UIOcTree octreeForViz = new UIOcTree(normalOctreeMessage);

      for (UIOcTreeNode uiOcTreeNode : octreeForViz)
      {
         Vector3D planeNormal = new Vector3D();
         Point3D pointOnPlane = new Point3D();

         uiOcTreeNode.getNormal(planeNormal);
         uiOcTreeNode.getHitLocation(pointOnPlane);
         Plane3D plane = new Plane3D(pointOnPlane, planeNormal);
         octreeNodePlanes.add(plane);
      }

      return octreeNodePlanes;
   }

   public static NormalOcTree computeOctreeData(List<Point3D[]> pointCloudMap, List<RigidBodyTransform> sensorPoses, double octreeResolution)
   {
      ScanCollection scanCollection = new ScanCollection();
      for (int i = 0; i < pointCloudMap.size(); i++)
      {
         int numberOfPoints = pointCloudMap.get(i).length;

         scanCollection.setSubSampleSize(numberOfPoints);
         scanCollection.addScan(toScan(pointCloudMap.get(i), sensorPoses.get(i).getTranslation()));
      }

      NormalOcTree referenceOctree = new NormalOcTree(octreeResolution);

      referenceOctree.insertScanCollection(scanCollection, false);

      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      referenceOctree.setNormalEstimationParameters(normalEstimationParameters);

      referenceOctree.updateNormals();
      return referenceOctree;
   }

   public static NormalOcTree computeOctreeData(Point3D[] pointCloud, Tuple3DReadOnly sensorPosition, double octreeResolution)
   {
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = pointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(toScan(pointCloud, sensorPosition));

      NormalOcTree referenceOctree = new NormalOcTree(octreeResolution);

      referenceOctree.insertScanCollection(scanCollection, false);

      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      referenceOctree.setNormalEstimationParameters(normalEstimationParameters);

      referenceOctree.updateNormals();
      return referenceOctree;
   }

   public static NormalOcTree computeOctreeData(StereoVisionPointCloudMessage pointCloud, double octreeResolution)
   {
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = pointCloud.getColors().size();

      scanCollection.setSubSampleSize(numberOfPoints);
      Point3D sensorPosition = pointCloud.getSensorPosition();
      scanCollection.addScan(toScan(pointCloud.getPointCloud(), sensorPosition));

      NormalOcTree referenceOctree = new NormalOcTree(octreeResolution);

      referenceOctree.insertScanCollection(scanCollection, false);

      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      referenceOctree.setNormalEstimationParameters(normalEstimationParameters);

      referenceOctree.updateNormals();

      return referenceOctree;
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(NormalOcTree referenceOctree, Point3DReadOnly sensorPosition)
   {
      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(sensorPosition);

      segmentationCalculator.compute(referenceOctree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      return rawData;
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(StereoVisionPointCloudMessage pointCloud, double octreeResolution)
   {
      NormalOcTree referenceOctree = computeOctreeData(pointCloud, octreeResolution);

      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(pointCloud.getSensorPosition());

      segmentationCalculator.compute(referenceOctree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      return rawData;
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(Point3D[] pointCloud, Tuple3DReadOnly sensorPosition, double octreeResolution)
   {
      NormalOcTree referenceOctree = computeOctreeData(pointCloud, sensorPosition, octreeResolution);

      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(sensorPosition);

      segmentationCalculator.compute(referenceOctree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      return rawData;
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(List<Point3D[]> pointCloudMap, List<RigidBodyTransform> sensorPoses,
                                                                                  double octreeResolution)
   {
      NormalOcTree referenceOctree = computeOctreeData(pointCloudMap, sensorPoses, octreeResolution);

      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(false);

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(new Point3D(0.0, 0.0, 20.0)); //TODO: work this for every poses.

      segmentationCalculator.compute(referenceOctree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      return rawData;
   }

   public static PlanarRegionsList buildNewMap(List<PlanarRegionSegmentationRawData> newRawData, PlanarRegionsList oldMap,
                                               CustomRegionMergeParameters customRegionMergeParameters,
                                               ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters)
   {
      List<PlanarRegion> oldMapCopy = new ArrayList<>(oldMap.getPlanarRegionsAsList());
      //oldMap.clear();
      List<PlanarRegion> unmergedCustomPlanarRegions = CustomPlanarRegionHandler.mergeCustomRegionsToEstimatedRegions(oldMapCopy, newRawData,
                                                                                                                      customRegionMergeParameters);

      PlanarRegionsList newMap = PlanarRegionPolygonizer.createPlanarRegionsList(newRawData, concaveHullFactoryParameters, polygonizerParameters);
      unmergedCustomPlanarRegions.forEach(newMap::addPlanarRegion);

      return newMap;
   }

   /**
    * Sensor pose is assumed as its X is toward right side, Y is toward down side.
    */
   public static Point3D[] createConvertPointsToSensorPose(RigidBodyTransform sensorPose, Point3D[] pointCloud)
   {
      Point3D[] convertedPoints = new Point3D[pointCloud.length];
      RigidBodyTransform inverseTransformer = new RigidBodyTransform(sensorPose);
      inverseTransformer.invert();
      Point3D convertedPoint = new Point3D();
      for (int i = 0; i < pointCloud.length; i++)
      {
         inverseTransformer.transform(pointCloud[i], convertedPoint);
         convertedPoints[i] = new Point3D(convertedPoint);
      }

      return convertedPoints;
   }

   public static Point3D[] createPointsInPreviousView(RigidBodyTransform sensorPose, RigidBodyTransform previousSensorPose, Point3D[] pointCloud)
   {
      Point3D[] convertedPoints = createConvertPointsToSensorPose(sensorPose, pointCloud);

      // TODO: redefine this X-Y window with 2Dconcave hull.
      double maxX = 0.0;
      double minX = Double.MAX_VALUE;
      double maxY = 0.0;
      double minY = Double.MAX_VALUE;

      for (int i = 0; i < convertedPoints.length; i++)
      {
         maxX = Math.max(maxX, convertedPoints[i].getX());
         maxY = Math.max(maxY, convertedPoints[i].getY());

         minX = Math.min(minX, convertedPoints[i].getX());
         minY = Math.min(minY, convertedPoints[i].getY());
      }

      Point3D[] convertedPointsToPreviousSensorPose = createConvertPointsToSensorPose(previousSensorPose, pointCloud);
      boolean[] heckyFlags = new boolean[convertedPointsToPreviousSensorPose.length];
      int numberOfPointsInPreviousView = 0;
      for (int i = 0; i < convertedPoints.length; i++)
      {
         Point3D point = convertedPointsToPreviousSensorPose[i];
         heckyFlags[i] = false;
         if (minX < point.getX() && point.getX() < maxX)
         {
            if (minY < point.getY() && point.getY() < maxY)
            {
               heckyFlags[i] = true;
               numberOfPointsInPreviousView++;
            }
         }
      }

      Point3D[] pointsInPreviousView = new Point3D[numberOfPointsInPreviousView];
      int index = 0;
      for (int i = 0; i < convertedPoints.length; i++)
      {
         if (heckyFlags[i])
         {
            pointsInPreviousView[index] = new Point3D(pointCloud[i]);
            index++;
         }
      }

      return pointsInPreviousView;
   }
}
