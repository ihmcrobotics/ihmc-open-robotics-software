package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;

public class SurfaceElementICPSLAM extends SLAMBasics
{
   public static final boolean DEBUG = false;
   public Point3D[] correctedSourcePointsToWorld;

   private final PlanarRegionSegmentationCalculator segmentationCalculator;

   public SurfaceElementICPSLAM(double octreeResolution)
   {
      super(octreeResolution);
      
      segmentationCalculator = new PlanarRegionSegmentationCalculator();

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);
      surfaceNormalFilterParameters.setSurfaceNormalLowerBound(Math.toRadians(-40.0));
      surfaceNormalFilterParameters.setSurfaceNormalUpperBound(Math.toRadians(40.0));

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);

      polygonizerParameters.setConcaveHullThreshold(0.15);
   }


   private void insertNewPointCloud(SLAMFrame frame)
   {
      Point3DReadOnly[] pointCloud = frame.getPointCloud();
      RigidBodyTransformReadOnly sensorPose = frame.getSensorPose();

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = frame.getPointCloud().length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(pointCloud, sensorPose.getTranslation()));

      octree.insertScanCollection(scanCollection, false);
      octree.enableParallelComputationForNormals(true);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);
   }

   @Override
   public void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      super.addKeyFrame(pointCloudMessage);

      SLAMFrame firstFrame = getLatestFrame();

      insertNewPointCloud(firstFrame);
   }

   @Override
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      boolean success = super.addFrame(pointCloudMessage);

      if (success)
      {
         SLAMFrame newFrame = getLatestFrame();
         insertNewPointCloud(newFrame);
      }

      return success;
   }

   public void updatePlanarRegionsMap()
   {
      octree.updateNormals();
      segmentationCalculator.setSensorPosition(getLatestFrame().getSensorPose().getTranslation());
      segmentationCalculator.compute(octree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(SLAMFrame frame)
   {
      NormalOcTree frameMap = new NormalOcTree(getOctreeResolution());
      
      Point3DReadOnly[] pointCloud = frame.getPointCloud();
      RigidBodyTransformReadOnly sensorPose = frame.getSensorPose();

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = frame.getPointCloud().length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(pointCloud, sensorPose.getTranslation()));
      
      frameMap.insertScanCollection(scanCollection, false);
      frameMap.enableParallelComputationForNormals(true);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      frameMap.setNormalEstimationParameters(normalEstimationParameters);
      frameMap.updateNormals();
      
      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createIterable(frameMap.getRoot());
      int index = 0;
      for (NormalOcTreeNode node : iterable)
      {
         System.out.println(index + " " + node.getNumberOfHits());
         index++;
         Vector3D normal = new Vector3D();
         Point3D hitLocation = new Point3D();
         node.getNormal(normal);
         node.getHitLocation(hitLocation);
      }
      
      return new RigidBodyTransform();
   }



}
