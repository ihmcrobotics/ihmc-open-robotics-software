package us.ihmc.humanoidBehaviors.ui.mapping;

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
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

public class IhmcSLAM
{
   private static final boolean DEBUG = true;

   private final boolean naiveSLAM;

   public static final double OCTREE_RESOLUTION = 0.02;

   public static final double VALID_PLANES_RATIO_THRESHOLD = 0.1;

   public static final double MAXIMUM_DISTANCE_OF_SIMILARITY = 0.05;
   public static final double MAXIMUM_ANGLE_OF_SIMILARITY = Math.toRadians(5.0);

   private final List<Point3DReadOnly[]> originalPointCloudMap = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> originalSensorPoses = new ArrayList<>();

   private final List<IhmcSLAMFrame> slamFrames = new ArrayList<>();
   private final List<Point3DReadOnly[]> pointCloudMap = new ArrayList<>();
   private final List<RigidBodyTransformReadOnly> sensorPoses = new ArrayList<>();

   private PlanarRegionsList planarRegionsMap;
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();

   private static final double OPTIMIZER_POSITION_LIMIT = 0.05;
   private static final double OPTIMIZER_ANGLE_LIMIT = Math.toRadians(8.0);

   public static final TDoubleArrayList initialQuery = new TDoubleArrayList();
   public static final TDoubleArrayList lowerLimit = new TDoubleArrayList();
   public static final TDoubleArrayList upperLimit = new TDoubleArrayList();

   static
   {
      for (int i = 0; i < 3; i++)
      {
         initialQuery.add(0.0);
         lowerLimit.add(-OPTIMIZER_POSITION_LIMIT);
         upperLimit.add(OPTIMIZER_POSITION_LIMIT);
      }
      for (int i = 0; i < 3; i++)
      {
         initialQuery.add(0.0);
         lowerLimit.add(-OPTIMIZER_ANGLE_LIMIT);
         upperLimit.add(OPTIMIZER_ANGLE_LIMIT);
      }
   }

   public IhmcSLAM()
   {
      this.naiveSLAM = false;
   }

   public IhmcSLAM(boolean naiveSLAM)
   {
      this.naiveSLAM = naiveSLAM;
   }

   //TODO: fix this.
   private void updatePlanarRegionsMap(IhmcSLAMFrame frame)
   {
      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frame.getPointCloud(), frame.getSensorPose().getTranslation(),
                                                                                               OCTREE_RESOLUTION);

      System.out.println("rawData " + rawData.size());

      planarRegionsMap = EnvironmentMappingTools.buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters,
                                                             polygonizerParameters);
   }

   private void updatePlanarRegionsMap()
   {
      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloudMap, sensorPoses, OCTREE_RESOLUTION);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   private IhmcSLAMFrame getLatestFrame()
   {
      return slamFrames.get(slamFrames.size() - 1);
   }

   public void addFirstFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(pointCloudMessage);
      originalPointCloudMap.add(frame.getOriginalPointCloud());
      originalSensorPoses.add(frame.getOriginalSensorPose());

      slamFrames.add(frame);
      pointCloudMap.add(frame.getPointCloud());
      sensorPoses.add(frame.getSensorPose());

      updatePlanarRegionsMap();
   }

   public List<List<IhmcSurfaceElement>> allSurfaceElements = new ArrayList<>();
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      IhmcSLAMFrame frame = new IhmcSLAMFrame(getLatestFrame(), pointCloudMessage);
      originalPointCloudMap.add(frame.getOriginalPointCloud());
      originalSensorPoses.add(frame.getOriginalSensorPose());

      RigidBodyTransform optimizedMultiplier = new RigidBodyTransform();
      if (!naiveSLAM)
      {
         System.out.println("planarRegionsMap.getNumberOfPlanarRegions() " + planarRegionsMap.getNumberOfPlanarRegions());
         List<IhmcSurfaceElement> surfaceElements = IhmcSLAMTools.computeMergeableSurfaceElements(planarRegionsMap, frame, OCTREE_RESOLUTION,
                                                                                              VALID_PLANES_RATIO_THRESHOLD, MAXIMUM_DISTANCE_OF_SIMILARITY,
                                                                                              MAXIMUM_ANGLE_OF_SIMILARITY);
         if (surfaceElements == null)
         {
            optimizedMultiplier = new RigidBodyTransform();
         }
         else
         {
            allSurfaceElements.add(surfaceElements);
            optimizedMultiplier = computeOptimizedMultiplier(surfaceElements, frame.getInitialSensorPoseToWorld());
         }
      }

      frame.updateSLAM(optimizedMultiplier);

      //      System.out.println("optimizedMultiplier");
      //      System.out.println(optimizedMultiplier);

      slamFrames.add(frame);
      pointCloudMap.add(frame.getPointCloud());
      sensorPoses.add(frame.getSensorPose());

//            updatePlanarRegionsMap();
      updatePlanarRegionsMap(frame);

      return true;
   }

   private RigidBodyTransform computeOptimizedMultiplier(List<IhmcSurfaceElement> surfaceElements, RigidBodyTransformReadOnly transformWorldToSensorPose)
   {
      PreMultiplierOptimizerCostFunction function = new PreMultiplierOptimizerCostFunction(surfaceElements, transformWorldToSensorPose);
      GradientDescentModule optimizer = new GradientDescentModule(function, initialQuery);

      int maxIterations = 100;
      double convergenceThreshold = 10E-5;
      double optimizerStepSize = -0.5;
      double optimizerPerturbationSize = 0.0001;

      optimizer.setInputLowerLimit(lowerLimit);
      optimizer.setInputUpperLimit(upperLimit);
      optimizer.setMaximumIterations(maxIterations);
      optimizer.setConvergenceThreshold(convergenceThreshold);
      optimizer.setStepSize(optimizerStepSize);
      optimizer.setPerturbationSize(optimizerPerturbationSize);

      int run = optimizer.run();
      System.out.println("optimizer Query() " + run + " " + function.getQuery(initialQuery) + " " + optimizer.getOptimalQuery());
      TDoubleArrayList optimalInput = optimizer.getOptimalInput();

      RigidBodyTransform transformer = new RigidBodyTransform();
      function.convertToSensorPoseMultiplier(optimalInput, transformer);

      return transformer;
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
