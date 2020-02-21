package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

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
import us.ihmc.robotEnvironmentAwareness.slam.tools.IhmcSLAMTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAM implements IhmcSLAMInterface
{
   private final double octreeResolution;

   private final AtomicReference<IhmcSLAMFrame> latestSlamFrame = new AtomicReference<>(null);
   private final List<Point3DReadOnly[]> pointCloudMap = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> sensorPoses = new ArrayList<>();

   protected PlanarRegionsList planarRegionsMap;
   protected final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   protected final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   protected final CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();
   protected final PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();

   public IhmcSLAM(double octreeResolution)
   {
      this.octreeResolution = octreeResolution;

      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.03);
      planarRegionSegmentationParameters.setMinRegionSize(150);
   }

   @Override
   public void addFirstFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(pointCloudMessage);
      latestSlamFrame.set(frame);

      pointCloudMap.add(frame.getPointCloud());
      sensorPoses.add(frame.getSensorPose());

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frame.getOriginalPointCloud(),
                                                                                               frame.getInitialSensorPoseToWorld().getTranslation(),
                                                                                               octreeResolution, planarRegionSegmentationParameters);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   @Override
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(getLatestFrame(), pointCloudMessage);

      RigidBodyTransformReadOnly optimizedMultiplier = computeFrameCorrectionTransformer(frame);

      if (optimizedMultiplier == null)
      {
         return false;
      }
      else
      {
         frame.updateOptimizedCorrection(optimizedMultiplier);
         
         latestSlamFrame.set(frame);

         pointCloudMap.add(frame.getPointCloud());
         sensorPoses.add(frame.getSensorPose());

         return true;
      }
   }

   @Override
   public void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(getLatestFrame(), pointCloudMessage);
      latestSlamFrame.set(frame);
      
      RigidBodyTransformReadOnly optimizedMultiplier = new RigidBodyTransform();

      frame.updateOptimizedCorrection(optimizedMultiplier);

      pointCloudMap.add(frame.getPointCloud());
      sensorPoses.add(frame.getSensorPose());
   }

   @Override
   public void updatePlanarRegionsMap()
   {
      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloudMap, sensorPoses, octreeResolution,
                                                                                               planarRegionSegmentationParameters);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   @Override
   public void clear()
   {
      latestSlamFrame.set(null);
      pointCloudMap.clear();
      sensorPoses.clear();
   }

   public boolean isEmpty()
   {
      if (latestSlamFrame.get() == null)
         return true;
      else
         return false;
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

   protected IhmcSLAMFrame getLatestFrame()
   {
      return latestSlamFrame.get();
   }

   public double getOctreeResolution()
   {
      return octreeResolution;
   }
}
