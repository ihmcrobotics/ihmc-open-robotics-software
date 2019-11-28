package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAM
{
   private static final boolean DEBUG = true;

   public static final double OCTREE_RESOLUTION = 0.02;

   public static final double VALID_PLANES_RATIO_THRESHOLD = 0.3;

   public static final double MAXIMUM_DISTANCE_OF_SIMILARITY = 0.1;
   public static final double MAXIMUM_ANGLE_OF_SIMILARITY = Math.toRadians(30.0);

   private final List<Point3DReadOnly[]> originalPointCloudMap = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> originalSensorPoses = new ArrayList<>();

   private final List<IhmcSLAMFrame> slamFrames = new ArrayList<>();
   private final List<Point3DReadOnly[]> pointCloudMap = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> sensorPoses = new ArrayList<>();

   private PlanarRegionsList planarRegionsMap;
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();

   public IhmcSLAM()
   {

   }

   private void updatePlanarRegionsMap()
   {
      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloudMap, sensorPoses, OCTREE_RESOLUTION);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
      System.out.println("planarRegionsMap " + planarRegionsMap.getNumberOfPlanarRegions());
   }

   private IhmcSLAMFrame getLatestFrame()
   {
      return slamFrames.get(slamFrames.size() - 1);
   }

   public void addFirstFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(pointCloudMessage);
      originalPointCloudMap.add(frame.getPointCloud());
      originalSensorPoses.add(frame.getSensorPose());

      slamFrames.add(frame);
      pointCloudMap.add(frame.getPointCloud());
      sensorPoses.add(frame.getSensorPose());

      updatePlanarRegionsMap();
   }

   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(getLatestFrame(), pointCloudMessage);
      originalPointCloudMap.add(frame.getPointCloud());
      originalSensorPoses.add(frame.getSensorPose());

      boolean mergeable = true;
      List<Plane3D> validPlanes = IhmcSLAMTools.computeValidPlanes(planarRegionsMap, frame, OCTREE_RESOLUTION, VALID_PLANES_RATIO_THRESHOLD,
                                                                   MAXIMUM_DISTANCE_OF_SIMILARITY, MAXIMUM_ANGLE_OF_SIMILARITY);
      if (validPlanes == null)
         mergeable = false;

      if (mergeable)
      {
         //TODO: do slam.
         slamFrames.add(frame);
         pointCloudMap.add(frame.getPointCloud());
         sensorPoses.add(frame.getSensorPose());

         updatePlanarRegionsMap();
      }

      if(DEBUG)
         System.out.println("mergeable " + mergeable);

      return mergeable;
   }

   public List<Point3DReadOnly[]> getOriginalPointCloudMap()
   {
      return originalPointCloudMap;
   }

   public List<RigidBodyTransformReadOnly> getOriginalSensorPoses()
   {
      return originalSensorPoses;
   }

   public List<Point3DReadOnly[]> getPointCloudMap()
   {
      return pointCloudMap;
   }

   public List<RigidBodyTransformReadOnly> getSensorPoses()
   {
      return sensorPoses;
   }

   public PlanarRegionsList getPlanarRegionsMap()
   {
      return planarRegionsMap;
   }

}
