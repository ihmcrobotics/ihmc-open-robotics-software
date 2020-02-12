package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.slam.tools.IhmcSLAMTools;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public class RandomICPSLAM extends IhmcSLAM
{
   public static final boolean DEBUG = true;

   private static final int DEFAULT_NUMBER_OF_SOURCE_POINTS = 300;

   private static final double DEFAULT_WINDOW_MINIMUM_DEPTH = 0.5;
   private static final double DEFAULT_WINDOW_MAXIMUM_DEPTH = 1.5;
   private static final double DEFAULT_MINIMUM_OVERLAPPED_RATIO = 0.4;

   private static final double DEFAULT_WINDOW_MARGIN = 0.1;
   private static final double DEFAULT_MAXIMUM_INITIAL_DISTANCE_RATIO = 10.0;
   private static final double DEFAULT_MINIMUM_INLIERS_RATIO_OF_KEY_FRAME = 0.95;
   private static final int DEFAULT_MAXIMUM_OCTREE_SEARCHING_SIZE = 5;

   private final AtomicInteger numberOfSourcePoints = new AtomicInteger(DEFAULT_NUMBER_OF_SOURCE_POINTS);
   private final AtomicInteger searchingSize = new AtomicInteger(DEFAULT_MAXIMUM_OCTREE_SEARCHING_SIZE);
   private final AtomicDouble minimumOverlappedRatio = new AtomicDouble(DEFAULT_MINIMUM_OVERLAPPED_RATIO);
   private final AtomicDouble windowMargin = new AtomicDouble(DEFAULT_WINDOW_MARGIN);
   private final AtomicDouble minimumInliersRatio = new AtomicDouble(DEFAULT_MINIMUM_INLIERS_RATIO_OF_KEY_FRAME);

   private final NormalOcTree octree;
   private final PlanarRegionSegmentationCalculator segmentationCalculator;

   private static final double OPTIMIZER_POSITION_LIMIT = 0.1;
   private static final double OPTIMIZER_ANGLE_LIMIT = Math.toRadians(10.);

   protected static final TDoubleArrayList INITIAL_INPUT = new TDoubleArrayList();
   protected static final TDoubleArrayList LOWER_LIMIT = new TDoubleArrayList();
   protected static final TDoubleArrayList UPPER_LIMIT = new TDoubleArrayList();

   private final AtomicDouble latestComputationTime = new AtomicDouble();

   public static boolean ENABLE_YAW_CORRECTION = false;

   static
   {
      for (int i = 0; i < 3; i++)
      {
         INITIAL_INPUT.add(0.0);
         LOWER_LIMIT.add(-OPTIMIZER_POSITION_LIMIT);
         UPPER_LIMIT.add(OPTIMIZER_POSITION_LIMIT);
      }
      if (ENABLE_YAW_CORRECTION)
      {
         INITIAL_INPUT.add(0.0);
         LOWER_LIMIT.add(-OPTIMIZER_ANGLE_LIMIT);
         UPPER_LIMIT.add(OPTIMIZER_ANGLE_LIMIT);
      }
   }

   // debugging variables.
   public Point3D[] sourcePointsToWorld;
   public Point3D[] correctedSourcePointsToWorld;

   public RandomICPSLAM(double octreeResolution)
   {
      super(octreeResolution);

      octree = new NormalOcTree(octreeResolution);

      segmentationCalculator = new PlanarRegionSegmentationCalculator();

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);
      surfaceNormalFilterParameters.setSurfaceNormalLowerBound(Math.toRadians(-10.0));
      surfaceNormalFilterParameters.setSurfaceNormalUpperBound(Math.toRadians(10.0));

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(new Point3D(0.0, 0.0, 20.0)); //TODO: work this for every poses.

   }

   public NormalOcTree getOctree()
   {
      return octree;
   }

   public void clear()
   {
      super.clear();
      octree.clear();
   }

   private void insertNewPointCloud(IhmcSLAMFrame frame)
   {
      Point3DReadOnly[] pointCloud = frame.getPointCloud();
      RigidBodyTransformReadOnly sensorPose = frame.getSensorPose();

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = getLatestFrame().getPointCloud().length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(IhmcSLAMTools.toScan(pointCloud, sensorPose.getTranslation(), DEFAULT_WINDOW_MINIMUM_DEPTH, DEFAULT_WINDOW_MAXIMUM_DEPTH));

      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);
   }

   @Override
   public void addFirstFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      super.addFirstFrame(pointCloudMessage);

      IhmcSLAMFrame firstFrame = getLatestFrame();

      insertNewPointCloud(firstFrame);
   }

   @Override
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      boolean success = super.addFrame(pointCloudMessage);

      if (success)
      {
         IhmcSLAMFrame newFrame = getLatestFrame();
         insertNewPointCloud(newFrame);

         octree.updateNormals();
         segmentationCalculator.compute(octree.getRoot());
      }

      return success;
   }

   @Override
   public void updatePlanarRegionsMap()
   {
      octree.updateNormals();
      segmentationCalculator.compute(octree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(IhmcSLAMFrame frame)
   {
      // TODO: FB-347: if the angle distance between original sensor pose orientation and new one, think it is key frame.
      // see the overlapped area.
      Point3D[] sourcePointsToSensor = IhmcSLAMTools.createSourcePointsToSensorPose(frame, octree, numberOfSourcePoints.get(), minimumOverlappedRatio.get(),
                                                                                    windowMargin.get());

      if (sourcePointsToSensor == null)
      {
         if (DEBUG)
            System.out.println("small overlapped area");
         return new RigidBodyTransform();
      }
      else
      {
         if (DEBUG)
            this.sourcePointsToWorld = IhmcSLAMTools.createConvertedPointsToWorld(frame.getInitialSensorPoseToWorld(), sourcePointsToSensor);

         RigidBodyTransformReadOnly transformWorldToSensorPose = frame.getInitialSensorPoseToWorld();
         RandomICPSLAMFrameOptimizerCostFunction costFunction = new RandomICPSLAMFrameOptimizerCostFunction(transformWorldToSensorPose, sourcePointsToSensor);

         double initialQuery = costFunction.getQuery(INITIAL_INPUT);
         if (DEBUG)
            System.out.println("frame distance " + initialQuery);

         if (initialQuery > DEFAULT_MAXIMUM_INITIAL_DISTANCE_RATIO * getOctreeResolution())
         {
            if (DEBUG)
               System.out.println("too far. will not be merged.");
            return null;
         }
         else
         {
            int numberOfInliers = IhmcSLAMTools.countNumberOfInliers(octree, transformWorldToSensorPose, sourcePointsToSensor, searchingSize.get());
            if (numberOfInliers > minimumInliersRatio.get() * sourcePointsToSensor.length)
            {
               if (DEBUG)
                  System.out.println("close enough. many inliers.");
               return new RigidBodyTransform();
            }

            GradientDescentModule optimizer = new GradientDescentModule(costFunction, INITIAL_INPUT);
            int maxIterations = 300;
            double convergenceThreshold = 1 * 10E-5;
            double optimizerStepSize = -1.0;
            double optimizerPerturbationSize = 0.00001;
            optimizer.setInputLowerLimit(LOWER_LIMIT);
            optimizer.setInputUpperLimit(UPPER_LIMIT);
            optimizer.setMaximumIterations(maxIterations);
            optimizer.setConvergenceThreshold(convergenceThreshold);
            optimizer.setStepSize(optimizerStepSize);
            optimizer.setPerturbationSize(optimizerPerturbationSize);
            optimizer.setReducingStepSizeRatio(2);
            int run = optimizer.run();
            latestComputationTime.set((double) Math.round(optimizer.getComputationTime() * 100) / 100);
            if (DEBUG)
               System.out.println("optimization result # [" + run + "], #" + optimizer.getComputationTime() + " sec # " + "Init Q: " + initialQuery
                     + ", Opt Q: " + optimizer.getOptimalQuery());
            TDoubleArrayList optimalInput = optimizer.getOptimalInput();
            RigidBodyTransform transformer = new RigidBodyTransform();
            costFunction.convertToSensorPoseMultiplier(optimalInput, transformer);
            if (DEBUG)
            {
               RigidBodyTransform optimizedSensorPose = new RigidBodyTransform(transformWorldToSensorPose);
               optimizedSensorPose.multiply(transformer);
               correctedSourcePointsToWorld = IhmcSLAMTools.createConvertedPointsToWorld(optimizedSensorPose, sourcePointsToSensor);
            }

            return transformer;
         }
      }
   }

   public double getComputationTimeForLatestFrame()
   {
      return latestComputationTime.get();
   }

   public void updateParameters(IhmcSLAMParameters parameters)
   {
      numberOfSourcePoints.set(parameters.getNumberOfSourcePoints());
      searchingSize.set(parameters.getMaximumICPSearchingSize());
      minimumOverlappedRatio.set(parameters.getMinimumOverlappedRatio());
      windowMargin.set(parameters.getWindowMargin());
      minimumInliersRatio.set(parameters.getMinimumInliersRatioOfKeyFrame());
   }

   class RandomICPSLAMFrameOptimizerCostFunction implements SingleQueryFunction
   {
      final Point3DReadOnly[] sourcePointsToSensor;
      private static final double TRANSLATION_TO_ANGLE_RATIO = 2.0;
      protected final RigidBodyTransformReadOnly transformWorldToSensorPose;
      private final RigidBodyTransform yawRotator = new RigidBodyTransform();

      RandomICPSLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose, Point3DReadOnly[] sourcePointsToSensor)
      {
         this.transformWorldToSensorPose = transformWorldToSensorPose;
         this.sourcePointsToSensor = sourcePointsToSensor;
      }

      void convertToSensorPoseMultiplier(TDoubleArrayList input, RigidBodyTransform transformToPack)
      {
         RigidBodyTransform newSensorPose = new RigidBodyTransform();
         convertToSensorPose(input, newSensorPose);

         newSensorPose.normalizeRotationPart();
         newSensorPose.preMultiplyInvertOther(transformWorldToSensorPose);

         transformToPack.set(newSensorPose);
      }

      void convertToSensorPose(TDoubleArrayList input, RigidBodyTransform sensorPoseToPack)
      {
         sensorPoseToPack.set(transformWorldToSensorPose);
         sensorPoseToPack.appendTranslation(input.get(0), input.get(1), input.get(2));
         if (ENABLE_YAW_CORRECTION)
         {
            yawRotator.setIdentity();
            yawRotator.appendYawRotation(input.get(3) * TRANSLATION_TO_ANGLE_RATIO);
            sensorPoseToPack.preMultiply(yawRotator);
         }
      }

      void convertToPointCloudTransformer(TDoubleArrayList input, RigidBodyTransform transformToPack)
      {
         RigidBodyTransform newSensorPose = new RigidBodyTransform();
         convertToSensorPose(input, newSensorPose);

         transformToPack.set(newSensorPose);
         transformToPack.multiplyInvertOther(transformWorldToSensorPose);
      }

      @Override
      public double getQuery(TDoubleArrayList values)
      {
         /**
          * values are difference in 6 dimensions : dx, dy, dz, du, dv, dw
          */
         RigidBodyTransform newSensorPose = new RigidBodyTransform();
         convertToSensorPose(values, newSensorPose);

         double totalDistance = 0;
         Point3D newSourcePointToWorld = new Point3D();
         for (Point3DReadOnly sourcePoint : sourcePointsToSensor)
         {
            newSourcePointToWorld.set(sourcePoint);
            newSensorPose.transform(newSourcePointToWorld);

            double distance = IhmcSLAMTools.computeDistanceToNormalOctree(octree, newSourcePointToWorld, DEFAULT_MAXIMUM_OCTREE_SEARCHING_SIZE);

            if (distance < 0)
            {
               distance = DEFAULT_MAXIMUM_OCTREE_SEARCHING_SIZE * getOctreeResolution();
            }

            totalDistance = totalDistance + distance;
         }

         double squareOfInput = 0.0;
         for (double value : values.toArray())
         {
            squareOfInput = squareOfInput + value * value;
         }

         double cost = 1 * totalDistance / sourcePointsToSensor.length + 0 * squareOfInput;

         return cost;
      }
   }
}
