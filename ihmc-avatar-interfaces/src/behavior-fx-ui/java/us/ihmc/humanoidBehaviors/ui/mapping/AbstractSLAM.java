package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class AbstractSLAM<T extends SLAMFrame>
{
   private final boolean naiveSLAM;
   private final double octreeResolution;

   private final List<Point3DReadOnly[]> originalPointCloudMap = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> originalSensorPoses = new ArrayList<>();

   private final List<SLAMFrame> slamFrames = new ArrayList<>();
   private final List<Point3DReadOnly[]> pointCloudMap = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> sensorPoses = new ArrayList<>();

   protected PlanarRegionsList planarRegionsMap;
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();
   private final PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();

   public AbstractSLAM(boolean doNaiveSLAM, double octreeResolution)
   {
      this.naiveSLAM = doNaiveSLAM;
      this.octreeResolution = octreeResolution;

      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.03);
      planarRegionSegmentationParameters.setSearchRadius(0.05);

      customRegionMergeParameters.setMaxDistanceFromPlane(0.03);
      customRegionMergeParameters.setSearchRadius(0.05);
   }
   
   public abstract SLAMFrame createFrame(StereoVisionPointCloudMessage pointCloudMessage);
   public abstract SLAMFrame createFrame(SLAMFrame previousFrame, StereoVisionPointCloudMessage pointCloudMessage);

   public void addFirstFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      SLAMFrame frame = createFrame(pointCloudMessage);
      originalPointCloudMap.add(frame.getOriginalPointCloud());
      originalSensorPoses.add(frame.getOriginalSensorPose());

      slamFrames.add(frame);
      pointCloudMap.add(frame.getPointCloud());
      sensorPoses.add(frame.getSensorPose());

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frame.getOriginalPointCloud(),
                                                                                               frame.getInitialSensorPoseToWorld().getTranslation(),
                                                                                               octreeResolution, planarRegionSegmentationParameters);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      SLAMFrame frame = createFrame(getLatestFrame(), pointCloudMessage);
      originalPointCloudMap.add(frame.getOriginalPointCloud());
      originalSensorPoses.add(frame.getOriginalSensorPose());

      RigidBodyTransform optimizedMultiplier = new RigidBodyTransform();
      if (naiveSLAM)
      {
         optimizedMultiplier = new RigidBodyTransform();
      }
      else
      {
         optimizedMultiplier = computeOptimizedMultiplier(frame);
      }

      System.out.println();
      System.out.println(optimizedMultiplier);

      frame.updateSLAM(optimizedMultiplier);

      slamFrames.add(frame);
      pointCloudMap.add(frame.getPointCloud());
      sensorPoses.add(frame.getSensorPose());

      if (!naiveSLAM)
         updatePlanarRegionsMap(frame);

      return true;
   }

   public abstract RigidBodyTransform computeOptimizedMultiplier(SLAMFrame newFrame);

   public void doNaiveSLAM()
   {
      updatePlanarRegionsMap();
   }

   private void updatePlanarRegionsMap(SLAMFrame frame)
   {
      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frame.getPointCloud(), frame.getSensorPose().getTranslation(),
                                                                                               octreeResolution, planarRegionSegmentationParameters);

      planarRegionsMap = EnvironmentMappingTools.buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters,
                                                             polygonizerParameters);
   }

   private void updatePlanarRegionsMap()
   {
      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloudMap, sensorPoses, octreeResolution,
                                                                                               planarRegionSegmentationParameters);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   private SLAMFrame getLatestFrame()
   {
      return slamFrames.get(slamFrames.size() - 1);
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

   public SLAMFrame getSLAMFrame(int i)
   {
      return slamFrames.get(i);
   }
   
   public double getOctreeResolution()
   {
      return octreeResolution;
   }
}
