package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class EnvironmentMap
{
   private final long initialTimeStamp;
   private final TDoubleArrayList timeMap = new TDoubleArrayList();
   private final List<RigidBodyTransform> sensorPoses = new ArrayList<>();
   private final List<StereoVisionPointCloudMessage> pointCloudMap = new ArrayList<>();
   private final List<NormalOcTree> octreeMap = new ArrayList<>();

   private PlanarRegionsList planarRegionsMap;

   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();

   private static final double OCTREE_RESOLUTION = 0.02;

   public EnvironmentMap(long firstTimeStamp, StereoVisionPointCloudMessage pointCloud)
   {
      initialTimeStamp = firstTimeStamp;
      timeMap.add(0.0);
      pointCloudMap.add(pointCloud);
      sensorPoses.add(new RigidBodyTransform(pointCloud.getSensorOrientation(), pointCloud.getSensorPosition()));
      
      octreeMap.add(EnvironmentMappingTools.computeOctreeData(pointCloud, OCTREE_RESOLUTION));
      
      List<PlanarRegionSegmentationRawData> rawData = EnvironmentMappingTools.computePlanarRegionRawData(pointCloud, OCTREE_RESOLUTION);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   public void addFrame(long newTimeStamp, StereoVisionPointCloudMessage newPointCloud)
   {
      double time = Conversions.nanosecondsToSeconds(newTimeStamp - initialTimeStamp);
      timeMap.add(time);
      octreeMap.add(EnvironmentMappingTools.computeOctreeData(newPointCloud, OCTREE_RESOLUTION));

      List<PlanarRegionSegmentationRawData> rawData = EnvironmentMappingTools.computePlanarRegionRawData(newPointCloud, OCTREE_RESOLUTION);

      planarRegionsMap = EnvironmentMappingTools.buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters,
                                                             polygonizerParameters);
      
      sensorPoses.add(new RigidBodyTransform(newPointCloud.getSensorOrientation(), newPointCloud.getSensorPosition()));
   }

   public RigidBodyTransform getLatestSensorPoses()
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
