package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.slam.tools.IhmcSLAMTools;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

public class TwoStageSLAM extends IhmcSLAM
{
   public static final boolean DEBUG = true;

   private static final int NUMBER_OF_SOURCE_POINTS = 300;

   private static final double WINDOW_MINIMUM_DEPTH = 0.5;
   private static final double WINDOW_MAXIMUM_DEPTH = 1.5;
   private static final double MINIMUM_OVERLAPPED_RATIO = 0.4;

   private static final int MAXIMUM_OCTREE_SEARCHING_SIZE = 5;

   protected static final TDoubleArrayList FIRST_INITIAL_INPUT = new TDoubleArrayList();
   protected static final TDoubleArrayList FIRST_LOWER_LIMIT = new TDoubleArrayList();
   protected static final TDoubleArrayList FIRST_UPPER_LIMIT = new TDoubleArrayList();

   protected static final TDoubleArrayList SECOND_INITIAL_INPUT = new TDoubleArrayList();
   protected static final TDoubleArrayList SECOND_LOWER_LIMIT = new TDoubleArrayList();
   protected static final TDoubleArrayList SECOND_UPPER_LIMIT = new TDoubleArrayList();

   private final NormalOcTree octree;
   private final PlanarRegionSegmentationCalculator segmentationCalculator;

   private boolean useYaw = true;

   public TwoStageSLAM(double octreeResolution)
   {
      super(octreeResolution);

      for (int i = 0; i < 3; i++)
      {
         FIRST_INITIAL_INPUT.add(0.0);
         FIRST_LOWER_LIMIT.add(-Math.toRadians(10.0));
         FIRST_UPPER_LIMIT.add(Math.toRadians(10.0));

         SECOND_INITIAL_INPUT.add(0.0);
         SECOND_LOWER_LIMIT.add(-0.1);
         SECOND_UPPER_LIMIT.add(0.1);
      }
      if (useYaw)
      {
         SECOND_INITIAL_INPUT.add(0.0);
         SECOND_LOWER_LIMIT.add(-Math.toRadians(10.0));
         SECOND_UPPER_LIMIT.add(Math.toRadians(10.0));
      }

      octree = new NormalOcTree(octreeResolution);

      segmentationCalculator = new PlanarRegionSegmentationCalculator();

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);
      surfaceNormalFilterParameters.setSurfaceNormalLowerBound(Math.toRadians(-40.0));
      surfaceNormalFilterParameters.setSurfaceNormalUpperBound(Math.toRadians(40.0));

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(new Point3D(0.0, 0.0, 20.0));
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
      scanCollection.addScan(IhmcSLAMTools.toScan(pointCloud, sensorPose.getTranslation(), WINDOW_MINIMUM_DEPTH, WINDOW_MAXIMUM_DEPTH));

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

         List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();
         planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
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

   private List<IhmcSurfaceElement> computeSurfaceElements(IhmcSLAMFrame frame)
   {
      // Planar Regions for new frame.
      double octreeResolutionForNewFrame = 0.03;
      PlanarRegionSegmentationParameters planarRegionSegmentationParametersFrame = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParametersFrame.setSearchRadius(octreeResolutionForNewFrame * 2.5);
      List<PlanarRegionSegmentationRawData> rawDataFrame = IhmcSLAMTools.computePlanarRegionRawData(frame.getOriginalPointCloud(),
                                                                                                    frame.getOriginalSensorPose().getTranslation(),
                                                                                                    octreeResolutionForNewFrame,
                                                                                                    planarRegionSegmentationParametersFrame, false);
      PlanarRegionsList planarRegionsFrame = PlanarRegionPolygonizer.createPlanarRegionsList(rawDataFrame, concaveHullFactoryParameters, polygonizerParameters);

      RigidBodyTransformReadOnly sensorPose = frame.getInitialSensorPoseToWorld();

      double minimumOverlappedRatio = 0.5;
      List<IhmcSurfaceElement> surfaceElements = new ArrayList<>();
      for (int i = 0; i < planarRegionsFrame.getNumberOfPlanarRegions(); i++)
      {
         double maximumOverlappedRatio = 0.0;
         int indexOfMostOverlappedPlanarRegion = -1;

         PlanarRegion planarRegion = planarRegionsFrame.getPlanarRegion(i);
         List<ConvexPolygon2D> convertedConvexPolygonsToSensor = IhmcSLAMTools.createConvexPolygon2DToSensorPose(sensorPose, planarRegion);

         double projectedArea = 0;
         for (int j = 0; j < convertedConvexPolygonsToSensor.size(); j++)
         {
            projectedArea += convertedConvexPolygonsToSensor.get(j).getArea();
         }

         for (int j = 0; j < planarRegionsMap.getNumberOfPlanarRegions(); j++)
         {
            List<ConvexPolygon2D> planarRegionMapConvexPolygonsToSensor = IhmcSLAMTools.createConvexPolygon2DToSensorPose(sensorPose,
                                                                                                                          planarRegionsMap.getPlanarRegion(j));

            double overlappedArea = IhmcSLAMTools.computeOverlappedArea(convertedConvexPolygonsToSensor, planarRegionMapConvexPolygonsToSensor);

            double overlappedRatio = overlappedArea / projectedArea;
            if (overlappedRatio < minimumOverlappedRatio)
               continue;
            else
            {
               if (overlappedRatio > maximumOverlappedRatio)
               {
                  maximumOverlappedRatio = overlappedRatio;
                  indexOfMostOverlappedPlanarRegion = j;
               }
            }
         }

         if (indexOfMostOverlappedPlanarRegion != -1)
         {
            IhmcSurfaceElement element = new IhmcSurfaceElement(octreeResolutionForNewFrame);
            element.setPlane(planarRegion.getPlane());
            element.setMergeablePlanarRegion(planarRegionsMap.getPlanarRegion(indexOfMostOverlappedPlanarRegion));
            surfaceElements.add(element);
         }
      }

      return surfaceElements;
   }

   private GradientDescentModule createFirstOptimizer(FristStageSLAMFrameOptimizerCostFunction firstCostFunction)
   {
      // run.
      GradientDescentModule firstOptimizer = new GradientDescentModule(firstCostFunction, FIRST_INITIAL_INPUT);
      int maxIterations = 300;
      double convergenceThreshold = 1 * 10E-5;
      double optimizerStepSize = -1.0;
      double optimizerPerturbationSize = 0.00001;
      firstOptimizer.setInputLowerLimit(FIRST_LOWER_LIMIT);
      firstOptimizer.setInputUpperLimit(FIRST_UPPER_LIMIT);
      firstOptimizer.setMaximumIterations(maxIterations);
      firstOptimizer.setConvergenceThreshold(convergenceThreshold);
      firstOptimizer.setStepSize(optimizerStepSize);
      firstOptimizer.setPerturbationSize(optimizerPerturbationSize);
      firstOptimizer.setReducingStepSizeRatio(2);

      return firstOptimizer;
   }

   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(IhmcSLAMFrame frame)
   {
      // see the overlapped area.
      Point3D[] sourcePointsToSensor = IhmcSLAMTools.createSourcePointsToSensorPoseWithKinematicGuess(frame, octree, NUMBER_OF_SOURCE_POINTS,
                                                                                                      MINIMUM_OVERLAPPED_RATIO);

      if (sourcePointsToSensor == null) // if it is too small overlapped,
      {
         if (DEBUG)
            System.out.println("small overlapped area");
         return new RigidBodyTransform();
      }
      else
      {
         // First stage.
         RigidBodyTransformReadOnly originalSensorPose = frame.getInitialSensorPoseToWorld();
         List<IhmcSurfaceElement> surfaceElements = computeSurfaceElements(frame);
         RigidBodyTransform firstStageTransformer = new RigidBodyTransform();

         if (surfaceElements.size() == 0)
         {
            System.out.println("no surfaces " + surfaceElements.size());
         }
         else
         {
            // Define optimization cost function.
            FristStageSLAMFrameOptimizerCostFunction firstCostFunction = new FristStageSLAMFrameOptimizerCostFunction(originalSensorPose, surfaceElements);
            double firstInitialQuery = firstCostFunction.getQuery(FIRST_INITIAL_INPUT);

            GradientDescentModule firstOptimizer = createFirstOptimizer(firstCostFunction);

            int firstRun = firstOptimizer.run();
            System.out.println("first result # [" + firstRun + "], #" + firstOptimizer.getComputationTime() + " sec # " + "Init Q: " + firstInitialQuery
                  + ", Opt Q: " + firstOptimizer.getOptimalQuery());
            TDoubleArrayList firstOptimalInput = firstOptimizer.getOptimalInput();
            firstCostFunction.convertToSensorPoseMultiplier(firstOptimalInput, firstStageTransformer);
         }

         System.out.println("firstStageTransformer");
         System.out.println(firstStageTransformer);

         // Second stage.
         RigidBodyTransform newSensorPose = new RigidBodyTransform(originalSensorPose);
         newSensorPose.multiply(firstStageTransformer);

         SecondStageSLAMFrameOptimizerCostFunction secondCostFunction = new RandomICPSLAMFrameOptimizerCostFunction(newSensorPose, sourcePointsToSensor);
         double initialQuery = secondCostFunction.getQuery(SECOND_INITIAL_INPUT);

         GradientDescentModule optimizer = new GradientDescentModule(secondCostFunction, SECOND_INITIAL_INPUT);
         int maxIterations = 300;
         double convergenceThreshold = 1 * 10E-5;
         double optimizerStepSize = -1.0;
         double optimizerPerturbationSize = 0.00001;
         optimizer.setInputLowerLimit(SECOND_LOWER_LIMIT);
         optimizer.setInputUpperLimit(SECOND_UPPER_LIMIT);
         optimizer.setMaximumIterations(maxIterations);
         optimizer.setConvergenceThreshold(convergenceThreshold);
         optimizer.setStepSize(optimizerStepSize);
         optimizer.setPerturbationSize(optimizerPerturbationSize);
         optimizer.setReducingStepSizeRatio(2);
         int run = optimizer.run();
         System.out.println("optimization result # [" + run + "], #" + optimizer.getComputationTime() + " sec # " + "Init Q: " + initialQuery + ", Opt Q: "
               + optimizer.getOptimalQuery());
         TDoubleArrayList optimalInput = optimizer.getOptimalInput();
         RigidBodyTransform transformer = new RigidBodyTransform();
         secondCostFunction.convertToSensorPoseMultiplier(optimalInput, transformer);
         System.out.println("transformer");
         System.out.println(transformer);

         transformer.preMultiply(firstStageTransformer);

         return transformer;
      }
   }

   class RandomICPSLAMFrameOptimizerCostFunction extends SecondStageSLAMFrameOptimizerCostFunction
   {
      final Point3DReadOnly[] sourcePointsToSensor;

      RandomICPSLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose, Point3DReadOnly[] sourcePointsToSensor)
      {
         super(transformWorldToSensorPose, useYaw);
         this.sourcePointsToSensor = sourcePointsToSensor;
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

            double distance = IhmcSLAMTools.computeDistanceToNormalOctree(octree, newSourcePointToWorld, MAXIMUM_OCTREE_SEARCHING_SIZE);

            if (distance < 0)
            {
               distance = MAXIMUM_OCTREE_SEARCHING_SIZE * getOctreeResolution();
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
