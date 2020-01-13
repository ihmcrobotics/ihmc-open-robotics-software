package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.EnvironmentMappingTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMResult;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMergerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionSLAMEnvironmentMap
{
   private final List<RigidBodyTransform> sensorPoses = new ArrayList<>();
   private final List<Point3D[]> pointCloudMap = new ArrayList<>();
   private final List<StereoVisionPointCloudMessage> rawPointCloudMap = new ArrayList<>();

   private PlanarRegionsList planarRegionsMap;

   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();

   private static final double OCTREE_RESOLUTION = 0.02;
   
   public PlanarRegionSLAMEnvironmentMap()
   {
      
   }
   
   public void addKeyFrame(StereoVisionPointCloudMessage message)
   {
      rawPointCloudMap.add(message);
      List<PlanarRegionSegmentationRawData> rawData = EnvironmentMappingTools.computePlanarRegionRawData(message, OCTREE_RESOLUTION);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
      System.out.println("getNumberOfPlanarRegions "+ planarRegionsMap.getNumberOfPlanarRegions());
      addKeyFrame(getPointsFromMessage(message), getSensorPoseFromMessage(message));
   }
   
   public void addFrame(StereoVisionPointCloudMessage message)
   {
      rawPointCloudMap.add(message);
      addFrame(getPointsFromMessage(message), getSensorPoseFromMessage(message));
   }
   
   public PlanarRegionsList getPlanarRegionsMap()
   {
      return planarRegionsMap;
   }
   
   public List<Point3D[]> getPointCloudMap()
   {
      return pointCloudMap;
   }

   private void addKeyFrame(Point3D[] pointCloud, RigidBodyTransform sensorPose)
   {
      pointCloudMap.add(pointCloud);
      sensorPoses.add(sensorPose);
      computePlanarRegionsMap();
   }

   private void addFrame(Point3D[] pointCloud, RigidBodyTransform sensorPose)
   {
      if (planarRegionsMap.getNumberOfPlanarRegions() == 0)
         return;

      List<PlanarRegionSegmentationRawData> rawData = EnvironmentMappingTools.computePlanarRegionRawData(pointCloud, sensorPose.getTranslation(),
                                                                                                         OCTREE_RESOLUTION);

      PlanarRegionsList newData = EnvironmentMappingTools.buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters,
                                                                      polygonizerParameters);


      RigidBodyTransform transformFromIncomingToMap = new RigidBodyTransform();//slamResult.getTransformFromIncomingToMap();
      LogTools.info("\nSlam result: transformFromIncomingToMap = \n" + transformFromIncomingToMap);

      transformFromIncomingToMap.transform(sensorPose);
      for (int i = 0; i < pointCloud.length; i++)
      {
         transformFromIncomingToMap.transform(pointCloud[i]);
      }

      pointCloudMap.add(pointCloud);
      sensorPoses.add(sensorPose);
      computePlanarRegionsMap();
   }

   private void computePlanarRegionsMap()
   {
      for (int i = 0; i < pointCloudMap.size(); i++)
      {
//         List<PlanarRegionSegmentationRawData> rawData = EnvironmentMappingTools.computePlanarRegionRawData(pointCloudMap.get(i),
//                                                                                                            sensorPoses.get(i).getTranslation(),
//                                                                                                            OCTREE_RESOLUTION);
//
//         planarRegionsMap = EnvironmentMappingTools.buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters,
//                                                                polygonizerParameters);
      }
      
      System.out.println("getNumberOfPlanarRegions "+ planarRegionsMap.getNumberOfPlanarRegions());
   }
   
   private Point3D[] getPointsFromMessage(StereoVisionPointCloudMessage message)
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

   private RigidBodyTransform getSensorPoseFromMessage(StereoVisionPointCloudMessage message)
   {
      return new RigidBodyTransform(message.getSensorOrientation(), message.getSensorPosition());
   }
}
