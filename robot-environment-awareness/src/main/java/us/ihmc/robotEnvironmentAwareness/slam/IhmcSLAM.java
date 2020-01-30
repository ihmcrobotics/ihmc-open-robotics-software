package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
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

   private final List<Point3DReadOnly[]> originalPointCloudMap = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> originalSensorPoses = new ArrayList<>();

   private final List<IhmcSLAMFrame> slamFrames = new ArrayList<>();
   private final List<Point3DReadOnly[]> pointCloudMap = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> sensorPoses = new ArrayList<>();

   protected PlanarRegionsList planarRegionsMap;
   protected final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   protected final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   protected final CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();
   protected final PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();

   private static final double OPTIMIZER_POSITION_LIMIT = 0.1;
   private static final double OPTIMIZER_ANGLE_LIMIT = Math.toRadians(10.);

   protected static final TDoubleArrayList INITIAL_INPUT = new TDoubleArrayList();
   protected static final TDoubleArrayList LOWER_LIMIT = new TDoubleArrayList();
   protected static final TDoubleArrayList UPPER_LIMIT = new TDoubleArrayList();

   public static boolean ENABLE_ORIENTATION_CORRECTION = false; //TODO CHANGE

   static
   {
      for (int i = 0; i < 3; i++)
      {
         INITIAL_INPUT.add(0.0);
         LOWER_LIMIT.add(-OPTIMIZER_POSITION_LIMIT);
         UPPER_LIMIT.add(OPTIMIZER_POSITION_LIMIT);
      }
      if (ENABLE_ORIENTATION_CORRECTION)
      {
         for (int i = 0; i < 3; i++)
         {
            INITIAL_INPUT.add(0.0);
            LOWER_LIMIT.add(-OPTIMIZER_ANGLE_LIMIT);
            UPPER_LIMIT.add(OPTIMIZER_ANGLE_LIMIT);
         }
      }
   }

   public IhmcSLAM(double octreeResolution)
   {
      this.octreeResolution = octreeResolution;

      //TODO: tune.
//      planarRegionSegmentationParameters.setSearchRadius(0.03);
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.03);
//      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(15.0));
      planarRegionSegmentationParameters.setMinRegionSize(100);
   }

   @Override
   public void addFirstFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(pointCloudMessage);
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

   @Override
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(getLatestFrame(), pointCloudMessage);
      originalPointCloudMap.add(frame.getOriginalPointCloud());
      originalSensorPoses.add(frame.getOriginalSensorPose());

      RigidBodyTransformReadOnly optimizedMultiplier = computeFrameCorrectionTransformer(frame);

      if (optimizedMultiplier == null)
      {
         return false;
      }
      else
      {
         frame.updateOptimizedCorrection(optimizedMultiplier);

         slamFrames.add(frame);
         pointCloudMap.add(frame.getPointCloud());
         sensorPoses.add(frame.getSensorPose());

         return true;
      }
   }

   @Override
   public void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(getLatestFrame(), pointCloudMessage);
      originalPointCloudMap.add(frame.getOriginalPointCloud());
      originalSensorPoses.add(frame.getOriginalSensorPose());

      RigidBodyTransformReadOnly optimizedMultiplier = new RigidBodyTransform();

      frame.updateOptimizedCorrection(optimizedMultiplier);

      slamFrames.add(frame);
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

   public void clear()
   {
      originalPointCloudMap.clear();
      originalSensorPoses.clear();

      slamFrames.clear();
      pointCloudMap.clear();
      sensorPoses.clear();
   }

   public boolean isEmpty()
   {
      if (slamFrames.size() == 0)
         return true;
      else
         return false;
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

   public IhmcSLAMFrame getSLAMFrame(int i)
   {
      return slamFrames.get(i);
   }

   protected IhmcSLAMFrame getLatestFrame()
   {
      return slamFrames.get(slamFrames.size() - 1);
   }

   public double getOctreeResolution()
   {
      return octreeResolution;
   }
}
