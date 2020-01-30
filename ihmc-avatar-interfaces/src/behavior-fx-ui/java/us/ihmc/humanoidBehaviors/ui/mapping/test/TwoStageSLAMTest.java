package us.ihmc.humanoidBehaviors.ui.mapping.test;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.IhmcSLAMViewer;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.slam.IhmcSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.IhmcSLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.IhmcSurfaceElement;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrameOptimizerCostFunction;
import us.ihmc.robotEnvironmentAwareness.slam.TwoStageSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.tools.IhmcSLAMTools;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SimulatedStereoVisionPointCloudMessageLibrary;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

public class TwoStageSLAMTest
{
   private final StereoVisionPointCloudMessage messageOne;
   private final StereoVisionPointCloudMessage driftedMessageTwo;
   private final IhmcSLAMFrame frameOne;
   private final IhmcSLAMFrame frameTwo;

   private static final String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
   private static final boolean useRealData = false;

   public TwoStageSLAMTest()
   {
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      double movingForward = 0.1;
      double fixedHeight = 1.0;

      double sensorPitchAngle = Math.toRadians(90.0 + 70.0);
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, fixedHeight);
      sensorPoseOne.appendPitchRotation(sensorPitchAngle);
      sensorPoseOne.appendYawRotation(Math.toRadians(-90.0));
      messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne, new RigidBodyTransform(), stairHeight, stairWidth,
                                                                                            stairLength, stairLength, true);

      double translationX = movingForward / 2;
      double translationY = 0.0;
      double translationZ = 0.0;
      double rotateY = Math.toRadians(0.0);
      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      preMultiplier.setTranslation(translationX, translationY, translationZ);
      preMultiplier.appendPitchRotation(rotateY);

      RigidBodyTransform sensorPoseTwo = new RigidBodyTransform();
      sensorPoseTwo.setTranslation(movingForward, 0.0, fixedHeight);
      sensorPoseTwo.appendPitchRotation(sensorPitchAngle);
      sensorPoseTwo.appendYawRotation(Math.toRadians(-90.0));

      RigidBodyTransform randomTransformer = createRandomDriftedTransform(new Random(0612L), 0.1, 10.0);
      randomTransformer.appendTranslation(0.02, 0.0, 0.01);
      preMultiplier.multiply(randomTransformer);
      sensorPoseTwo.multiply(randomTransformer);

      driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, preMultiplier, stairHeight, stairWidth,
                                                                                                   stairLength - movingForward, stairLength + movingForward,
                                                                                                   true);

      if (useRealData)
      {
         frameOne = new IhmcSLAMFrame(messages.get(47));
         frameTwo = new IhmcSLAMFrame(frameOne, messages.get(48));
      }
      else
      {
         frameOne = new IhmcSLAMFrame(messageOne);
         frameTwo = new IhmcSLAMFrame(frameOne, driftedMessageTwo);
      }
   }

   @Test
   public void testFirstStage()
   {
      double octreeResolutionForPR = 0.02;
      double octreeResolutionForNewFrame = 0.04; // TODO: 0.05 has only one planar regions.

      PlanarRegionsList planarRegionsMap;
      PlanarRegionsList planarRegionsFrame;

      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParametersFrame = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParametersFrame.setSearchRadius(octreeResolutionForNewFrame * 2.5);

      // Planar Regions.
      List<PlanarRegionSegmentationRawData> rawDataMap = IhmcSLAMTools.computePlanarRegionRawData(frameOne.getPointCloud(),
                                                                                                  frameOne.getSensorPose().getTranslation(),
                                                                                                  octreeResolutionForPR, planarRegionSegmentationParameters,
                                                                                                  false);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawDataMap, concaveHullFactoryParameters, polygonizerParameters);
      System.out.println("planarRegionsMap " + planarRegionsMap.getNumberOfPlanarRegions());

      // Planar Regions for new frame.
      List<PlanarRegionSegmentationRawData> rawDataFrame = IhmcSLAMTools.computePlanarRegionRawData(frameTwo.getOriginalPointCloud(),
                                                                                                    frameTwo.getOriginalSensorPose().getTranslation(),
                                                                                                    octreeResolutionForNewFrame,
                                                                                                    planarRegionSegmentationParametersFrame, false);
      planarRegionsFrame = PlanarRegionPolygonizer.createPlanarRegionsList(rawDataFrame, concaveHullFactoryParameters, polygonizerParameters);
      System.out.println("planarRegionsFrame " + planarRegionsFrame.getNumberOfPlanarRegions());

      // Define elements
      RigidBodyTransformReadOnly sensorPose = frameTwo.getInitialSensorPoseToWorld();

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

         System.out.println(i + " projectedArea " + projectedArea);

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

      // Define optimization cost function.
      FistStageSLAMFrameOptimizerCostFunction costFunction = new FistStageSLAMFrameOptimizerCostFunction(sensorPose, surfaceElements);

      TDoubleArrayList orientingInitialInput = new TDoubleArrayList();
      TDoubleArrayList orientingLowerLimit = new TDoubleArrayList();
      TDoubleArrayList orientingUpperLimit = new TDoubleArrayList();
      for (int i = 0; i < 3; i++)
      {
         orientingInitialInput.add(0.0);
         orientingLowerLimit.add(-Math.toRadians(10.));
         orientingUpperLimit.add(Math.toRadians(10.));
      }
      // run.
      double initialQuery = costFunction.getQuery(orientingInitialInput);
      GradientDescentModule optimizer = new GradientDescentModule(costFunction, orientingInitialInput);
      int maxIterations = 300;
      double convergenceThreshold = 1 * 10E-5;
      double optimizerStepSize = -1.0;
      double optimizerPerturbationSize = 0.00001;
      optimizer.setInputLowerLimit(orientingLowerLimit);
      optimizer.setInputUpperLimit(orientingUpperLimit);
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
      costFunction.convertToSensorPoseMultiplier(optimalInput, transformer);

      frameTwo.updateOptimizedCorrection(transformer);

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();
      slamViewer.addPointCloud(frameOne.getOriginalPointCloud(), Color.BLUE);
      slamViewer.addPointCloud(frameTwo.getOriginalPointCloud(), Color.BLACK);

      slamViewer.addPointCloud(frameTwo.getPointCloud(), Color.GREEN);

      //      slamViewer.addPlanarRegions(planarRegionsMap);
      //      slamViewer.addPlanarRegions(planarRegionsFrame);

      slamViewer.start("testFirstStage");

      ThreadTools.sleepForever();
   }

   @Test
   public void testPlanarRegionConversion()
   {
      RigidBodyTransform transformOfPlanarRegion = new RigidBodyTransform();
      transformOfPlanarRegion.setTranslation(0.0, 0.0, 0.0);
      List<ConvexPolygon2D> planarRegionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(0.0, 0.0);
      convexPolygon.addVertex(0.0, 2.0);
      convexPolygon.addVertex(2.0, 2.0);
      convexPolygon.addVertex(2.0, 0.0);
      convexPolygon.update();
      planarRegionConvexPolygons.add(convexPolygon);
      PlanarRegion planarRegion = new PlanarRegion(transformOfPlanarRegion, planarRegionConvexPolygons);

      RigidBodyTransform sensorPose = new RigidBodyTransform();
      sensorPose.setTranslation(0.0, 0.0, 1.0);
      sensorPose.appendPitchRotation(Math.toRadians(190.0));

      List<ConvexPolygon2D> createConvexPolygon2DToSensorPose = IhmcSLAMTools.createConvexPolygon2DToSensorPose(sensorPose, planarRegion);
      for (int i = 0; i < createConvexPolygon2DToSensorPose.size(); i++)
      {
         System.out.println(i);
         ConvexPolygon2D convexPolygon2D = createConvexPolygon2DToSensorPose.get(i);
         for (int j = 0; j < convexPolygon2D.getNumberOfVertices(); j++)
         {
            System.out.println(convexPolygon2D.getVertex(j));
         }
      }

      double assumedTranslation = -Math.tan(Math.toRadians(10.0) * 1.0);
      double actualTranslation = createConvexPolygon2DToSensorPose.get(0).getVertex(1).getX();
      System.out.println("assumedTranslation " + assumedTranslation + ", actualTranslation " + actualTranslation);
      assertTrue(Math.abs(actualTranslation - assumedTranslation) < 0.01);
   }

   @Test
   public void testOverlappedAreaComputation()
   {
      double[][] vertexSetOne = new double[4][2];
      double[][] vertexSetTwo = new double[4][2];

      vertexSetOne[0][0] = 3.0;
      vertexSetOne[0][1] = 0.0;

      vertexSetOne[1][0] = 3.0;
      vertexSetOne[1][1] = 3.0;

      vertexSetOne[2][0] = 0.0;
      vertexSetOne[2][1] = 3.0;

      vertexSetOne[3][0] = 0.0;
      vertexSetOne[3][1] = 0.0;

      vertexSetTwo[0][0] = 2.0;
      vertexSetTwo[0][1] = 2.0;

      vertexSetTwo[1][0] = -2.0;
      vertexSetTwo[1][1] = 2.0;

      vertexSetTwo[2][0] = 2.0;
      vertexSetTwo[2][1] = -5.0;

      vertexSetTwo[3][0] = 2.0;
      vertexSetTwo[3][1] = -5.0;

      Vertex2DSupplier supplierOne = Vertex2DSupplier.asVertex2DSupplier(vertexSetOne);
      ConvexPolygon2D polygonOne = new ConvexPolygon2D(supplierOne);

      Vertex2DSupplier supplierTwo = Vertex2DSupplier.asVertex2DSupplier(vertexSetTwo);
      ConvexPolygon2D polygonTwo = new ConvexPolygon2D(supplierTwo);

      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
      double overlappedArea = convexPolygonTools.computeIntersectionAreaOfPolygons(polygonOne, polygonTwo);
      System.out.println("overlappedArea " + overlappedArea);
   }

   @Test
   public void testOptimizationForSimulatedPointCloud()
   {
      double octreeResolution = 0.02;
      TwoStageSLAM slam = new TwoStageSLAM(octreeResolution);

      slam.addFirstFrame(messageOne);
      slam.addFrame(driftedMessageTwo);

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();
      slamViewer.addPointCloud(slam.getSLAMFrame(0).getOriginalPointCloud(), Color.BLUE);
      slamViewer.addPointCloud(slam.getSLAMFrame(1).getOriginalPointCloud(), Color.BLACK);
      slamViewer.addSensorPose(slam.getSLAMFrame(1).getOriginalSensorPose(), Color.BLACK);

      slamViewer.addPointCloud(slam.getSLAMFrame(1).getPointCloud(), Color.GREEN);
      slamViewer.addSensorPose(slam.getSLAMFrame(1).getSensorPose(), Color.GREEN);

      slamViewer.start("testOptimizationForSimulatedPointCloud");

      ThreadTools.sleepForever();
   }

   private RigidBodyTransform createRandomDriftedTransform(Random random, double positionBound, double angleBoundDegree)
   {
      RigidBodyTransform randomTransform = new RigidBodyTransform();
      int positionAxis = random.nextInt(2);
      int angleAxis = random.nextInt(2);

      double randomAngle = 2 * Math.toRadians(angleBoundDegree) * (random.nextDouble() - 0.5);
      if (angleAxis == 0)
         randomTransform.appendRollRotation(randomAngle);
      if (angleAxis == 1)
         randomTransform.appendPitchRotation(randomAngle);
      else
         randomTransform.appendYawRotation(randomAngle);

      double randomTranslation = 2 * positionBound * (random.nextDouble() - 0.5);
      if (positionAxis == 0)
         randomTransform.appendTranslation(randomTranslation, 0.0, 0.0);
      if (positionAxis == 1)
         randomTransform.appendTranslation(0.0, randomTranslation, 0.0);
      else
         randomTransform.appendTranslation(0.0, 0.0, randomTranslation);

      System.out.println("positionAxis " + positionAxis + " " + randomTranslation);
      System.out.println("angleAxis " + angleAxis + " " + Math.toDegrees(randomAngle));

      return randomTransform;
   }

   class FistStageSLAMFrameOptimizerCostFunction extends SLAMFrameOptimizerCostFunction
   {
      final List<IhmcSurfaceElement> surfaceElements;
      static final double POSITION_WEIGHT = 5.0;
      static final double ANGLE_WEIGHT = 5.0;
      static final double SNAPPING_PARALLEL_WEIGHT = 5.0;

      static final double LEAST_SQUARE_WEIGHT = 0.1;

      FistStageSLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose, List<IhmcSurfaceElement> surfaceElements)
      {
         super(transformWorldToSensorPose);
         this.surfaceElements = surfaceElements;
      }

      @Override
      public double getQuery(TDoubleArrayList values)
      {
         /**
          * values are difference in 6 dimensions : dx, dy, dz, du, dv, dw
          */
         RigidBodyTransform transformer = new RigidBodyTransform();
         convertToPointCloudTransformer(values, transformer);

         double cost = 0;
         for (int i = 0; i < surfaceElements.size(); i++)
         {
            double distance = surfaceElements.get(i).getDistance(transformer, POSITION_WEIGHT, ANGLE_WEIGHT);

            double squareOfInput = 0.0;
            for (double value : values.toArray())
            {
               squareOfInput = squareOfInput + value * value;
            }

            double distanceCost = distance + LEAST_SQUARE_WEIGHT * squareOfInput;
            {
               cost = cost + distanceCost;
            }
         }

         return cost;
      }

      /**
       * newSensorPose = originalSensorPose * transformToPack;
       */
      @Override
      public void convertToSensorPoseMultiplier(TDoubleArrayList input, RigidBodyTransform transformToPack)
      {
         transformToPack.setRotationYawPitchRoll(input.get(0) / ANGLE_SCALER, input.get(1) / ANGLE_SCALER, input.get(2) / ANGLE_SCALER);
      }

      @Override
      public void convertToPointCloudTransformer(TDoubleArrayList input, RigidBodyTransform transformToPack)
      {
         RigidBodyTransform preMultiplier = new RigidBodyTransform();
         convertToSensorPoseMultiplier(input, preMultiplier);

         transformToPack.set(transformWorldToSensorPose);
         transformToPack.multiply(preMultiplier);
         transformToPack.multiplyInvertOther(transformWorldToSensorPose);
      }
   }
}
