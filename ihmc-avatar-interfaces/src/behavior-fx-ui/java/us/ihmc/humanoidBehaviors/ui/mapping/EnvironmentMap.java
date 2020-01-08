package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class EnvironmentMap
{
   private final long initialTimeStamp;
   private final TDoubleArrayList timeMap = new TDoubleArrayList();
   private final List<RigidBodyTransformReadOnly> sensorPoses = new ArrayList<>();
   private final List<Point3DReadOnly[]> pointCloudMap = new ArrayList<>();
   private final List<NormalOcTree> octreeMap = new ArrayList<>();

   private PlanarRegionsList planarRegionsMap;

   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

   private static final double OCTREE_RESOLUTION = 0.02;

   public EnvironmentMap(long firstTimeStamp, StereoVisionPointCloudMessage message)
   {
      initialTimeStamp = firstTimeStamp;
      timeMap.add(0.0);
      pointCloudMap.add(IhmcSLAMTools.extractPointsFromMessage(message));
      sensorPoses.add(IhmcSLAMTools.extractSensorPoseFromMessage(message));

      octreeMap.add(EnvironmentMappingTools.computeOctreeData(message, OCTREE_RESOLUTION));
   }

   public void addFrame(long newTimeStamp, StereoVisionPointCloudMessage newPointCloudMessage)
   {
      double time = Conversions.nanosecondsToSeconds(newTimeStamp - initialTimeStamp);
      timeMap.add(time);
      pointCloudMap.add(IhmcSLAMTools.extractPointsFromMessage(newPointCloudMessage));
      sensorPoses.add(IhmcSLAMTools.extractSensorPoseFromMessage(newPointCloudMessage));

      octreeMap.add(EnvironmentMappingTools.computeOctreeData(newPointCloudMessage, OCTREE_RESOLUTION));
   }

   public void computePlanarRegions()
   {
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.03);
      planarRegionSegmentationParameters.setSearchRadius(0.05);

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloudMap, sensorPoses, OCTREE_RESOLUTION,
                                                                                               planarRegionSegmentationParameters);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   public RigidBodyTransformReadOnly getLatestSensorPoses()
   {
      return sensorPoses.get(sensorPoses.size() - 1);
   }

   public PlanarRegionsList getPlanarRegionsMap()
   {
      return planarRegionsMap;
   }

   public List<NormalOcTree> getOctreeMap()
   {
      return octreeMap;
   }

   public double getOctreeResolution()
   {
      return OCTREE_RESOLUTION;
   }
}
