package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
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

   private static final boolean CONSIDER_TIME_DELAY = true;
   private static final double ASSUMED_TIME_DELAY = 0.15;

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

      Point3D[] extractPointsFromMessage = IhmcSLAMTools.extractPointsFromMessage(newPointCloudMessage);
      RigidBodyTransform extractSensorPoseFromMessage = IhmcSLAMTools.extractSensorPoseFromMessage(newPointCloudMessage);
      if (CONSIDER_TIME_DELAY)
      {
         // TODO: handle if exceed previous time gap. (timeMap).
         // TODO: implement time delayed sensor pose using ASSUMED_TIME_DELAY.
         // TODO: linear approximation now. in future, cubic spline would be needed.
         double timeGap = time - timeMap.get(timeMap.size() - 2);
         double alphaFromPrevious = 1.0 - ASSUMED_TIME_DELAY / timeGap;

         Point3D newPosition = new Point3D();
         Quaternion newOrientation = new Quaternion();

         RigidBodyTransformReadOnly previousSensorPose = sensorPoses.get(sensorPoses.size() - 1);
         Tuple3DReadOnly previousSensorPosition = previousSensorPose.getTranslation();
         Quaternion previousOrientation = new Quaternion(previousSensorPose.getRotation());

         newPosition.interpolate(previousSensorPosition, extractSensorPoseFromMessage.getTranslation(), alphaFromPrevious);
         newOrientation.interpolate(previousOrientation, new Quaternion(extractSensorPoseFromMessage.getRotation()), alphaFromPrevious);

         RigidBodyTransform newSensorPose = new RigidBodyTransform(newOrientation, newPosition);
         Point3D[] newPointCloud = IhmcSLAMTools.createConvertedPointsToWorld(extractSensorPoseFromMessage, extractPointsFromMessage, newSensorPose);

         System.out.println(time + " " + alphaFromPrevious + " " + previousSensorPosition + " " + newPosition);
         pointCloudMap.add(newPointCloud);
         sensorPoses.add(newSensorPose);
      }
      else
      {
         pointCloudMap.add(extractPointsFromMessage);
         sensorPoses.add(extractSensorPoseFromMessage);
      }

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
