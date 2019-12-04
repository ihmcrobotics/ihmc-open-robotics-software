package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.Plane3D;
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

   private static final double OPTIMIZER_POSITION_LIMIT = 0.05;
   private static final double OPTIMIZER_ANGLE_LIMIT = Math.toRadians(3.0);

   private static final TDoubleArrayList initialQuery = new TDoubleArrayList();
   private static final TDoubleArrayList lowerLimit = new TDoubleArrayList();
   private static final TDoubleArrayList upperLimit = new TDoubleArrayList();

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

   //TODO: fix this.
   private void updatePlanarRegionsMap(IhmcSLAMFrame frame)
   {
      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frame.getPointCloud(),
                                                                                                         frame.getSensorPose().getTranslation(),
                                                                                                         OCTREE_RESOLUTION);

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
         RigidBodyTransform optimizedMultiplier = computeOptimizedMultiplier(validPlanes, frame.getInitialSensorPoseToWorld());
         frame.updateSLAM(optimizedMultiplier);
         System.out.println("optimizedMultiplier");
         System.out.println(optimizedMultiplier);

         slamFrames.add(frame);
         pointCloudMap.add(frame.getPointCloud());
         sensorPoses.add(frame.getSensorPose());

         updatePlanarRegionsMap();
         //updatePlanarRegionsMap(frame);
      }

      if (DEBUG)
         System.out.println("mergeable " + mergeable);

      return mergeable;
   }

   private RigidBodyTransform computeOptimizedMultiplier(List<Plane3D> validPlanes, RigidBodyTransformReadOnly transformWorldToSensorPose)
   {
      PreMultiplierOptimizerCostFunction function = new PreMultiplierOptimizerCostFunction(planarRegionsMap, validPlanes, transformWorldToSensorPose);
      GradientDescentModule optimizer = new GradientDescentModule(function, initialQuery);

      int maxIterations = 100;
      double convergenceThreshold = 10E-5;
      double optimizerStepSize = -0.1;
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

      System.out.println(optimalInput.get(0) + " " + optimalInput.get(1));
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
