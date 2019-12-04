package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAMTools
{
   public static Point3D[] extractPointsFromMessage(StereoVisionPointCloudMessage message)
   {
      int numberOfPoints = message.getColors().size();
      Point3D[] pointCloud = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i] = new Point3D();
         MessageTools.unpackScanPoint(message, i, pointCloud[i]);
      }
      return pointCloud;
   }

   public static RigidBodyTransform extractSensorPoseFromMessage(StereoVisionPointCloudMessage message)
   {
      return new RigidBodyTransform(message.getSensorOrientation(), message.getSensorPosition());
   }

   public static Point3D[] createConvertedPointsToSensorPose(RigidBodyTransformReadOnly sensorPose, Point3DReadOnly[] pointCloud)
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

   public static Point3D[] createConvertedPointsToOtherSensorPose(RigidBodyTransformReadOnly sensorPose, RigidBodyTransformReadOnly otherSensorPose,
                                                                  Point3DReadOnly[] pointCloudToSensorPose)
   {
      Point3D[] pointCloudToWorld = new Point3D[pointCloudToSensorPose.length];
      for (int i = 0; i < pointCloudToWorld.length; i++)
      {
         pointCloudToWorld[i] = new Point3D(pointCloudToSensorPose[i]);
         sensorPose.transform(pointCloudToWorld[i]);
      }

      return createConvertedPointsToSensorPose(otherSensorPose, pointCloudToWorld);
   }

   private static Scan toScan(Point3DReadOnly[] points, Tuple3DReadOnly sensorPosition)
   {
      PointCloud pointCloud = new PointCloud();

      for (int i = 0; i < points.length; i++)
      {
         double x = points[i].getX();
         double y = points[i].getY();
         double z = points[i].getZ();
         pointCloud.add(x, y, z);
      }
      //return new Scan(new Point3D(sensorPosition), pointCloud);
      return new Scan(new Point3D(0.0, 0.0, 40.0), pointCloud);   // TODO: fix this with REPORT 1 in IhmcSLAMFrame.
   }

   public static NormalOcTree computeOctreeData(Point3DReadOnly[] pointCloud, Tuple3DReadOnly sensorPosition, double octreeResolution)
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

   public static NormalOcTree computeOctreeData(List<Point3DReadOnly[]> pointCloudMap, List<RigidBodyTransformReadOnly> sensorPoses, double octreeResolution)
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

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(List<Point3DReadOnly[]> pointCloudMap,
                                                                                  List<RigidBodyTransformReadOnly> sensorPoses, double octreeResolution)
   {
      NormalOcTree referenceOctree = computeOctreeData(pointCloudMap, sensorPoses, octreeResolution);

      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(false);

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(new Point3D(0.0, 0.0, 20.0)); //TODO: work this for every poses.

      segmentationCalculator.compute(referenceOctree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      return rawData;
   }
   
   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(Point3DReadOnly[] pointCloud, Tuple3DReadOnly sensorPosition, double octreeResolution)
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

   public static List<Plane3D> computeValidPlanes(PlanarRegionsList planarRegionsMap, IhmcSLAMFrame frame, double octreeResolution, double validRatio,
                                                  double maximumDistance, double maximumAngle)
   {
      NormalOcTree octree = frame.computeOctreeInPreviousView(octreeResolution);

      int numberOfPlanarRegions = planarRegionsMap.getNumberOfPlanarRegions();
      List<Plane3D> validPlanes = new ArrayList<>();
      NormalOcTreeMessage normalOctreeMessage = OcTreeMessageConverter.convertToMessage(octree);
      UIOcTree octreeForViz = new UIOcTree(normalOctreeMessage);
      for (UIOcTreeNode uiOcTreeNode : octreeForViz)
      {
         if (!uiOcTreeNode.isNormalSet() || !uiOcTreeNode.isHitLocationSet())
            continue;

         Vector3D planeNormal = new Vector3D();
         Point3D pointOnPlane = new Point3D();

         uiOcTreeNode.getNormal(planeNormal);
         uiOcTreeNode.getHitLocation(pointOnPlane);
         Plane3D octreePlane = new Plane3D(pointOnPlane, planeNormal);

         int indexClosestPlanarRegion = -1;
         double minimumDistance = Double.MAX_VALUE;
         for (int j = 0; j < numberOfPlanarRegions; j++)
         {
            PlanarRegion planarRegion = planarRegionsMap.getPlanarRegion(j);
            Plane3D plane = planarRegion.getPlane();
            double distance = plane.distance(octreePlane.getPoint());
            if (distance < minimumDistance)
            {
               minimumDistance = distance;
               indexClosestPlanarRegion = j;
            }
         }
         double angleDistance = Math.abs(planarRegionsMap.getPlanarRegion(indexClosestPlanarRegion).getPlane().getNormal().dot(octreePlane.getNormal()));

         //System.out.println("minimumDistance " + minimumDistance + " Math.cos(maximumAngle) " + Math.cos(maximumAngle));
         if (minimumDistance < maximumDistance && angleDistance > Math.cos(maximumAngle))
         {
            validPlanes.add(octreePlane);
         }
      }

      double ratio = (double) validPlanes.size() / octreeForViz.getNumberOfNodes();
      System.out.println("octreeForViz.getNumberOfNodes() " + octreeForViz.getNumberOfNodes() + " validPlanes are " + validPlanes.size() + " ratio " + ratio);

      if (ratio < validRatio)
         return null;

      return validPlanes;
   }
}
