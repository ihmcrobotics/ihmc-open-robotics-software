package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public class RandomICPSLAM extends SLAMBasics
{
   // debugging variables.
   public static final boolean DEBUG = false;
   public Point3D[] correctedSourcePointsToWorld;

   private final AtomicReference<RandomICPSLAMParameters> parameters = new AtomicReference<>(new RandomICPSLAMParameters());

   private final PlanarRegionSegmentationCalculator segmentationCalculator;

   private final GradientDescentModule optimizer;
   private static final double OPTIMIZER_POSITION_LIMIT = 0.1;
   private static final double OPTIMIZER_ANGLE_LIMIT = Math.toRadians(10.);

   private static final TDoubleArrayList INITIAL_INPUT = new TDoubleArrayList();
   private static final TDoubleArrayList LOWER_LIMIT = new TDoubleArrayList();
   private static final TDoubleArrayList UPPER_LIMIT = new TDoubleArrayList();

   private final AtomicDouble latestComputationTime = new AtomicDouble();
   private Point3D[] sourcePointsToWorld;

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

   public RandomICPSLAM(double octreeResolution)
   {
      super(octreeResolution);

      segmentationCalculator = new PlanarRegionSegmentationCalculator();

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);
      surfaceNormalFilterParameters.setSurfaceNormalLowerBound(Math.toRadians(-40.0));
      surfaceNormalFilterParameters.setSurfaceNormalUpperBound(Math.toRadians(40.0));

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);

      polygonizerParameters.setConcaveHullThreshold(0.15);

      optimizer = new GradientDescentModule(null, INITIAL_INPUT);
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
   }

   private void insertNewPointCloud(SLAMFrame frame)
   {
      Point3DReadOnly[] pointCloud = frame.getPointCloud();
      RigidBodyTransformReadOnly sensorPose = frame.getSensorPose();

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = getLatestFrame().getPointCloud().length;

      scanCollection.setSubSampleSize(numberOfPoints);
      RandomICPSLAMParameters parameters = this.parameters.get();
      scanCollection.addScan(SLAMTools.toScan(pointCloud, sensorPose.getTranslation(), parameters.getMinimumDepth(), parameters.getMaximumDepth()));

      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);
   }

   @Override
   public void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      super.addKeyFrame(pointCloudMessage);

      SLAMFrame firstFrame = getLatestFrame();

      insertNewPointCloud(firstFrame);
   }

   @Override
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      boolean success = super.addFrame(pointCloudMessage);

      if (success)
      {
         SLAMFrame newFrame = getLatestFrame();
         insertNewPointCloud(newFrame);
      }

      return success;
   }

   public void updatePlanarRegionsMap()
   {
      octree.updateNormals();
      segmentationCalculator.setSensorPosition(getLatestFrame().getSensorPose().getTranslation());
      segmentationCalculator.compute(octree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(SLAMFrame frame)
   {
      RandomICPSLAMParameters parameters = this.parameters.get();
      Point3D[] sourcePointsToSensor = SLAMTools.createSourcePointsToSensorPose(frame,
                                                                                octree,
                                                                                parameters.getNumberOfSourcePoints(),
                                                                                parameters.getMinimumOverlappedRatio(),
                                                                                parameters.getWindowMargin());

      if (sourcePointsToSensor == null)
      {
         // TODO: this frame would be handled when robot revisit this area.
         if (DEBUG)
            System.out.println("small overlapped area");
         frame.setConfidenceFactor(-1.0);
         return new RigidBodyTransform();
      }
      else
      {
         this.sourcePointsToWorld = SLAMTools.createConvertedPointsToWorld(frame.getInitialSensorPoseToWorld(), sourcePointsToSensor);

         RigidBodyTransformReadOnly transformWorldToSensorPose = frame.getInitialSensorPoseToWorld();
         RandomICPSLAMFrameOptimizerCostFunction costFunction = new RandomICPSLAMFrameOptimizerCostFunction(transformWorldToSensorPose, sourcePointsToSensor);

         double initialQuery = costFunction.getQuery(INITIAL_INPUT);
         if (DEBUG)
            System.out.println("frame distance " + initialQuery);

         if (initialQuery > parameters.getMaximumInitialDistanceRatio() * getOctreeResolution())
         {
            if (DEBUG)
               System.out.println("too far. will not be merged.");
            frame.setConfidenceFactor(0.0);
            return null;
         }
         else
         {
            int numberOfInliers = SLAMTools.countNumberOfInliers(octree,
                                                                 transformWorldToSensorPose,
                                                                 sourcePointsToSensor,
                                                                 parameters.getMaximumICPSearchingSize());
            if (numberOfInliers > parameters.getMinimumInliersRatioOfKeyFrame() * sourcePointsToSensor.length)
            {
               if (DEBUG)
                  System.out.println("close enough. many inliers.");
               frame.setConfidenceFactor(1.0);
               return new RigidBodyTransform();
            }

            optimizer.redefineModule(costFunction);
            optimizer.setStepSize(-1.0);
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
               correctedSourcePointsToWorld = SLAMTools.createConvertedPointsToWorld(optimizedSensorPose, sourcePointsToSensor);
               double finalQuery = costFunction.getQuery(optimalInput);
               System.out.println("finalQuery " + finalQuery);
            }

            //TODO: put proper value based on final distance.
            frame.setConfidenceFactor(1 - optimizer.getOptimalQuery() / getOctreeResolution() / 1);
            return transformer;
         }
      }
   }

   public double getComputationTimeForLatestFrame()
   {
      return latestComputationTime.get();
   }

   public Point3DReadOnly[] getSourcePointsToWorldLatestFrame()
   {
      return sourcePointsToWorld;
   }

   public void updateParameters(RandomICPSLAMParameters parameters)
   {
      this.parameters.set(parameters);
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

         newSensorPose.getRotation().normalize();
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

      @Override
      public double getQuery(TDoubleArrayList values)
      {
         RigidBodyTransform newSensorPose = new RigidBodyTransform();
         convertToSensorPose(values, newSensorPose);

         double totalDistance = 0;
         Point3D newSourcePointToWorld = new Point3D();
         for (Point3DReadOnly sourcePoint : sourcePointsToSensor)
         {
            newSourcePointToWorld.set(sourcePoint);
            newSensorPose.transform(newSourcePointToWorld);

            int maximumICPSearchingSize = parameters.get().getMaximumICPSearchingSize();
            double distance = SLAMTools.computeDistanceToNormalOctree(octree, newSourcePointToWorld, maximumICPSearchingSize);

            if (distance < 0)
            {
               distance = maximumICPSearchingSize * getOctreeResolution();
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
