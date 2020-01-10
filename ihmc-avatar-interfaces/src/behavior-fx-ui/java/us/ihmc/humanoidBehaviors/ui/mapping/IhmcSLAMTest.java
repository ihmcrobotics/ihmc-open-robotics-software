package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

public class IhmcSLAMTest
{
   @Test
   public void testViewer()
   {
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      String pointCloudPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
      File pointCloudFile = new File(pointCloudPath);
      List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      viewer.addStereoMessage(messagesFromFile.get(20), Color.GREEN, Color.GREEN);
      viewer.addStereoMessage(messagesFromFile.get(25), Color.BLUE, Color.BLUE);
      viewer.start("testViewer");

      ThreadTools.sleepForever();
   }

   @Test
   public void testSimulatedPointCloudFrameForStair()
   {
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      StereoVisionPointCloudMessage message = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(stairHeight, stairWidth, stairLength);
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addStereoMessage(message);
      viewer.start("testSimulatedPointCloudFrameForStair");

      ThreadTools.sleepForever();
   }

   @Test
   public void testSimulatedContinuousFramesForStair()
   {
      double movingForward = 0.05;

      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(stairHeight, stairWidth, stairLength,
                                                                                                                          stairLength);

      double translationX = movingForward / 2;
      double translationY = 0.0;
      double translationZ = 0.0;
      double rotateY = Math.toRadians(1.0);
      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      preMultiplier.setTranslation(translationX, translationY, translationZ);
      preMultiplier.appendPitchRotation(rotateY);
      StereoVisionPointCloudMessage messageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(preMultiplier, stairHeight,
                                                                                                                          stairWidth,
                                                                                                                          stairLength - movingForward,
                                                                                                                          stairLength + movingForward, true);
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addStereoMessage(messageOne, Color.BLUE);
      viewer.addStereoMessage(messageTwo, Color.GREEN);
      viewer.start("testSimulatedContinuousFramesForStair");

      ThreadTools.sleepForever();
   }

   @Test
   public void testSimulatedBadFrame()
   {
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(stairHeight, stairWidth, stairLength);

      double translationX = -0.02;
      double translationY = 0.03;
      double translationZ = 0.0;
      double rotateY = Math.toRadians(3.0);
      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      preMultiplier.setTranslation(translationX, translationY, translationZ);
      preMultiplier.appendPitchRotation(rotateY);
      StereoVisionPointCloudMessage messageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(preMultiplier, stairHeight,
                                                                                                                          stairWidth, stairLength);

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addStereoMessage(messageOne, Color.BLUE);
      viewer.addStereoMessage(messageTwo, Color.GREEN);
      viewer.start("testSimulatedBadFrame");

      ThreadTools.sleepForever();
   }

   @Test
   public void testPlanarRegionsForSimulatedPointCloudFrame()
   {
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      double octreeResolution = 0.02;
      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.02);

      StereoVisionPointCloudMessage message = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(stairHeight, stairWidth, stairLength);
      Point3DReadOnly[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);
      RigidBodyTransformReadOnly sensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(message);

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloud, sensorPose.getTranslation(), octreeResolution,
                                                                                               planarRegionSegmentationParameters);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addPlanarRegions(planarRegionsMap);
      viewer.start("testPlanarRegionsForSimulatedPointCloudFrame");

      ThreadTools.sleepForever();
   }

   @Test
   public void testPlanarRegionsForFlatGround()
   {
      double groundWidth = 1.5;
      double stairLength = 0.3;
      int numberOfMessages = 10;

      double octreeResolution = 0.02;
      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.02);

      List<Point3DReadOnly[]> pointCloudMap = new ArrayList<>();

      StereoVisionPointCloudMessage firstMessage = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(0.0, groundWidth, stairLength,
                                                                                                                            stairLength, false);
      Point3DReadOnly[] firstPointCloud = IhmcSLAMTools.extractPointsFromMessage(firstMessage);
      RigidBodyTransformReadOnly firstSensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(firstMessage);
      pointCloudMap.add(firstPointCloud);

      List<PlanarRegionSegmentationRawData> firstRawData = IhmcSLAMTools.computePlanarRegionRawData(firstPointCloud, firstSensorPose.getTranslation(),
                                                                                                    octreeResolution, planarRegionSegmentationParameters);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(firstRawData, concaveHullFactoryParameters, polygonizerParameters);

      for (int i = 1; i < numberOfMessages; i++)
      {
         RigidBodyTransform preMultiplier = new RigidBodyTransform();
         preMultiplier.setTranslation(i * stairLength, 0.0, 0.0);
         StereoVisionPointCloudMessage message = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(preMultiplier, 0.0, groundWidth,
                                                                                                                          stairLength, stairLength, false);

         Point3DReadOnly[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);
         RigidBodyTransformReadOnly sensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(message);
         pointCloudMap.add(pointCloud);

         List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloud, sensorPose.getTranslation(), octreeResolution,
                                                                                                  planarRegionSegmentationParameters);

         planarRegionsMap = EnvironmentMappingTools.buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters,
                                                                polygonizerParameters);
      }

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addPlanarRegions(planarRegionsMap);
      for (int i = 0; i < pointCloudMap.size(); i++)
      {
         int redScaler = (int) (0xFF * (1 - (double) i / pointCloudMap.size()));
         int blueScaler = (int) (0xFF * ((double) i / pointCloudMap.size()));
         Color color = Color.rgb(redScaler, 0, blueScaler);
         viewer.addPointCloud(pointCloudMap.get(i), color);
      }

      viewer.start("testPlanarRegionsForFlatGround");

      ThreadTools.sleepForever();
   }

   @Test
   public void testInverseInterpolation()
   {
      double movingForward = 0.1;
      double fixedHeight = 1.0;

      double sensorPitchAngle = Math.toRadians(90.0 + 70.0);
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      double octreeResolution = 0.02;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, fixedHeight);
      sensorPoseOne.appendPitchRotation(sensorPitchAngle);
      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne,
                                                                                                                          new RigidBodyTransform(), stairHeight,
                                                                                                                          stairWidth, stairLength, stairLength,
                                                                                                                          false);

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

      RigidBodyTransform randomTransformer = createRandomDriftedTransform(new Random(0612L), 0.05, 5.0);
      preMultiplier.multiply(randomTransformer);
      sensorPoseTwo.multiply(randomTransformer);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, preMultiplier,
                                                                                                                                 stairHeight, stairWidth,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 false);

      IhmcSLAMFrame frameOne = new IhmcSLAMFrame(messageOne);
      IhmcSLAMFrame driftedFrameTwo = new IhmcSLAMFrame(frameOne, driftedMessageTwo);
      driftedFrameTwo.computeOctreeInPreviousView(octreeResolution);
      NormalOcTree overlappedOctreeNode = driftedFrameTwo.getOctreeNodesInPreviousView();

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addSensorPose(frameOne.getSensorPose(), Color.BLUE);
      viewer.addSensorPose(driftedFrameTwo.getSensorPose(), Color.YELLOW);
      viewer.addPointCloud(frameOne.getPointCloud(), Color.BLUE);
      viewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      viewer.addOctree(overlappedOctreeNode, Color.GREEN, octreeResolution);
      viewer.start("testInverseInterpolation");

      ThreadTools.sleepForever();
   }

   @Test
   public void testDetectingSimilarPlanarRegions()
   {
      double movingForward = 0.1;
      double fixedHeight = 1.0;

      double sensorPitchAngle = Math.toRadians(90.0 + 70.0);
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      double octreeResolution = 0.02;
      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.02);

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, fixedHeight);
      sensorPoseOne.appendPitchRotation(sensorPitchAngle);
      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne,
                                                                                                                          new RigidBodyTransform(), stairHeight,
                                                                                                                          stairWidth, stairLength, stairLength,
                                                                                                                          true);

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

      RigidBodyTransform randomTransformer = createRandomDriftedTransform(new Random(0612L), 0.15, 15.0);
      preMultiplier.multiply(randomTransformer);
      sensorPoseTwo.multiply(randomTransformer);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, preMultiplier,
                                                                                                                                 stairHeight, stairWidth,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 false);

      IhmcSLAMFrame frameOne = new IhmcSLAMFrame(messageOne);
      IhmcSLAMFrame driftedFrameTwo = new IhmcSLAMFrame(frameOne, driftedMessageTwo);

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frameOne.getPointCloud(),
                                                                                               frameOne.getSensorPose().getTranslation(), octreeResolution,
                                                                                               planarRegionSegmentationParameters);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
      System.out.println("planarRegionsMap " + planarRegionsMap.getNumberOfPlanarRegions());

      double validRatio = 0.0;
      double maximumDistance = 0.1;
      double maximumAngle = Math.toRadians(30.0);
      driftedFrameTwo.computeOctreeInPreviousView(octreeResolution);
      driftedFrameTwo.computeMergeableSurfaceElements(planarRegionsMap, octreeResolution, validRatio, maximumDistance, maximumAngle);
      List<IhmcSurfaceElement> surfaceElements = driftedFrameTwo.getMergeableSurfaceElements();

      List<IhmcSurfaceElement> groupOne = new ArrayList<>();
      List<IhmcSurfaceElement> groupTwo = new ArrayList<>();
      for (IhmcSurfaceElement element : surfaceElements)
      {
         if (element.getMergeablePlanarRegionId() == planarRegionsMap.getPlanarRegion(0).getRegionId())
            groupOne.add(element);
         if (element.getMergeablePlanarRegionId() == planarRegionsMap.getPlanarRegion(1).getRegionId())
            groupTwo.add(element);
      }

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addSensorPose(frameOne.getSensorPose(), Color.BLUE);
      viewer.addSensorPose(driftedFrameTwo.getSensorPose(), Color.YELLOW);
      viewer.addPointCloud(frameOne.getPointCloud(), Color.BLUE);
      viewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      viewer.addOctree(groupOne, Color.CORAL);
      viewer.addOctree(groupTwo, Color.BLACK);

      viewer.start("testDetectingSimilarPlanarRegions");

      ThreadTools.sleepForever();
   }

   @Test
   public void testOptimization()
   {
      double movingForward = 0.1;
      double fixedHeight = 1.0;

      double sensorPitchAngle = Math.toRadians(90.0 + 70.0);
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      double octreeResolution = 0.02;
      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.02);

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, fixedHeight);
      sensorPoseOne.appendPitchRotation(sensorPitchAngle);
      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne,
                                                                                                                          new RigidBodyTransform(), stairHeight,
                                                                                                                          stairWidth, stairLength, stairLength,
                                                                                                                          true);

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

      RigidBodyTransform randomTransformer = createRandomDriftedTransform(new Random(0612L), 0.05, 5.0);
      preMultiplier.multiply(randomTransformer);
      sensorPoseTwo.multiply(randomTransformer);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, preMultiplier,
                                                                                                                                 stairHeight, stairWidth,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 false);

      IhmcSLAMFrame frameOne = new IhmcSLAMFrame(messageOne);
      IhmcSLAMFrame driftedFrameTwo = new IhmcSLAMFrame(frameOne, driftedMessageTwo);

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frameOne.getPointCloud(),
                                                                                               frameOne.getSensorPose().getTranslation(), octreeResolution,
                                                                                               planarRegionSegmentationParameters);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);

      double validRatio = 0.0;
      double maximumDistance = 0.1;
      double maximumAngle = Math.toRadians(30.0);
      driftedFrameTwo.computeOctreeInPreviousView(octreeResolution);
      driftedFrameTwo.computeMergeableSurfaceElements(planarRegionsMap, octreeResolution, validRatio, maximumDistance, maximumAngle);
      List<IhmcSurfaceElement> surfaceElements = driftedFrameTwo.getMergeableSurfaceElements();

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addSensorPose(frameOne.getSensorPose(), Color.BLUE);
      viewer.addSensorPose(driftedFrameTwo.getSensorPose(), Color.YELLOW);
      viewer.addPointCloud(frameOne.getPointCloud(), Color.BLUE);
      viewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      viewer.addOctree(surfaceElements, Color.CORAL);
      viewer.start("testOptimization");

      IhmcSLAMViewer localViewer = new IhmcSLAMViewer();
      localViewer.addPointCloud(frameOne.getPointCloud(), Color.RED);
      localViewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      localViewer.addOctree(surfaceElements, Color.CORAL);
      localViewer.addPlanarRegions(planarRegionsMap);
      IhmcSLAMFrameOptimizerCostFunction function = new IhmcSLAMFrameOptimizerCostFunction(surfaceElements, driftedFrameTwo.getInitialSensorPoseToWorld());
      GradientDescentModule optimizer = new GradientDescentModule(function, IhmcSLAM.initialQuery);

      int maxIterations = 200;
      double convergenceThreshold = 10E-5;
      double optimizerStepSize = -0.1;
      double optimizerPerturbationSize = 0.0001;

      optimizer.setInputLowerLimit(IhmcSLAM.lowerLimit);
      optimizer.setInputUpperLimit(IhmcSLAM.upperLimit);
      optimizer.setMaximumIterations(maxIterations);
      optimizer.setConvergenceThreshold(convergenceThreshold);
      optimizer.setStepSize(optimizerStepSize);
      optimizer.setPerturbationSize(optimizerPerturbationSize);
      optimizer.setReducingStepSizeRatio(2);

      int run = optimizer.run();
      System.out.println(run + " " + function.getQuery(IhmcSLAM.initialQuery) + " " + optimizer.getOptimalQuery());
      TDoubleArrayList optimalInput = optimizer.getOptimalInput();
      System.out.println(optimalInput.get(0) + " " + optimalInput.get(1) + " " + optimalInput.get(2) + " " + optimalInput.get(3));

      RigidBodyTransform slamTransformer = new RigidBodyTransform();
      function.convertToSensorPoseMultiplier(optimalInput, slamTransformer);
      System.out.println("slam transformer");
      System.out.println(slamTransformer);

      System.out.println("randomTransformer");
      System.out.println(randomTransformer);

      driftedFrameTwo.updateSLAM(slamTransformer);

      System.out.println("Test Result : " + slamTransformer.geometricallyEquals(randomTransformer, 0.05));
      System.out.println("Position Diff    : " + slamTransformer.getTranslation().geometricallyEquals(randomTransformer.getTranslation(), 0.05));
      System.out.println("Orientation Diff : " + Math.toRadians(slamTransformer.getRotation().distance(randomTransformer.getRotation())) + " deg.");

      Color color = Color.rgb(0, 255, 0);
      localViewer.addPointCloud(driftedFrameTwo.getPointCloud(), color);
      localViewer.start("iteration ");

      ThreadTools.sleepForever();
   }

   @Test
   public void testOptimizationForFlatGround()
   {
      double movingForward = 0.1;
      double fixedHeight = 1.0;

      double sensorPitchAngle = Math.toRadians(90.0 + 70.0);
      double stairHeight = 0.0;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      double octreeResolution = 0.02;
      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.02);

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, fixedHeight);
      sensorPoseOne.appendPitchRotation(sensorPitchAngle);
      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne,
                                                                                                                          new RigidBodyTransform(), stairHeight,
                                                                                                                          stairWidth, stairLength, stairLength,
                                                                                                                          true);

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

      RigidBodyTransform randomTransformer = createRandomDriftedTransform(new Random(0612L), 0.05, 5.0);
      preMultiplier.multiply(randomTransformer);
      sensorPoseTwo.multiply(randomTransformer);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, preMultiplier,
                                                                                                                                 stairHeight, stairWidth,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 false);

      IhmcSLAMFrame frameOne = new IhmcSLAMFrame(messageOne);
      IhmcSLAMFrame driftedFrameTwo = new IhmcSLAMFrame(frameOne, driftedMessageTwo);

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frameOne.getPointCloud(),
                                                                                               frameOne.getSensorPose().getTranslation(), octreeResolution,
                                                                                               planarRegionSegmentationParameters);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);

      double validRatio = 0.0;
      double maximumDistance = 0.1;
      double maximumAngle = Math.toRadians(30.0);
      driftedFrameTwo.computeOctreeInPreviousView(octreeResolution);
      driftedFrameTwo.computeMergeableSurfaceElements(planarRegionsMap, octreeResolution, validRatio, maximumDistance, maximumAngle);
      List<IhmcSurfaceElement> surfaceElements = driftedFrameTwo.getMergeableSurfaceElements();

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addSensorPose(frameOne.getSensorPose(), Color.BLUE);
      viewer.addSensorPose(driftedFrameTwo.getSensorPose(), Color.YELLOW);
      viewer.addPointCloud(frameOne.getPointCloud(), Color.BLUE);
      viewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      viewer.addOctree(surfaceElements, Color.CORAL);
      viewer.addPlanarRegions(planarRegionsMap);
      viewer.start("testOptimization");

      IhmcSLAMViewer localViewer = new IhmcSLAMViewer();
      localViewer.addPointCloud(frameOne.getPointCloud(), Color.RED);
      localViewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      localViewer.addOctree(surfaceElements, Color.CORAL);
      IhmcSLAMFrameOptimizerCostFunction function = new IhmcSLAMFrameOptimizerCostFunction(surfaceElements, driftedFrameTwo.getInitialSensorPoseToWorld());
      GradientDescentModule optimizer = new GradientDescentModule(function, IhmcSLAM.initialQuery);

      int maxIterations = 200;
      double convergenceThreshold = 10E-5;
      double optimizerStepSize = -0.1;
      double optimizerPerturbationSize = 0.0001;

      optimizer.setInputLowerLimit(IhmcSLAM.lowerLimit);
      optimizer.setInputUpperLimit(IhmcSLAM.upperLimit);
      optimizer.setMaximumIterations(maxIterations);
      optimizer.setConvergenceThreshold(convergenceThreshold);
      optimizer.setStepSize(optimizerStepSize);
      optimizer.setPerturbationSize(optimizerPerturbationSize);
      optimizer.setReducingStepSizeRatio(2);

      int run = optimizer.run();
      System.out.println(run + " " + function.getQuery(IhmcSLAM.initialQuery) + " " + optimizer.getOptimalQuery());
      TDoubleArrayList optimalInput = optimizer.getOptimalInput();
      System.out.println(optimalInput.get(0) + " " + optimalInput.get(1) + " " + optimalInput.get(2) + " " + optimalInput.get(3));

      RigidBodyTransform slamTransformer = new RigidBodyTransform();
      function.convertToSensorPoseMultiplier(optimalInput, slamTransformer);
      System.out.println("slam transformer");
      System.out.println(slamTransformer);

      System.out.println("randomTransformer");
      System.out.println(randomTransformer);

      driftedFrameTwo.updateSLAM(slamTransformer);

      System.out.println("Test Result : " + slamTransformer.geometricallyEquals(randomTransformer, 0.05));
      System.out.println("Position Diff    : " + slamTransformer.getTranslation().geometricallyEquals(randomTransformer.getTranslation(), 0.05));
      System.out.println("Orientation Diff : " + Math.toRadians(slamTransformer.getRotation().distance(randomTransformer.getRotation())) + " deg.");

      Color color = Color.rgb(0, 255, 0);
      localViewer.addPointCloud(driftedFrameTwo.getPointCloud(), color);

      for (IhmcSurfaceElement element : surfaceElements)
         element.transform(slamTransformer);
      localViewer.addOctree(surfaceElements, Color.BEIGE);

      //localViewer.addPlanarRegions(planarRegionsMap);
      localViewer.start("iteration ");

      ThreadTools.sleepForever();
   }

   @Test
   public void testTransformer()
   {
      RigidBodyTransform transformer = new RigidBodyTransform();
      transformer.setTranslation(0.1, 0.0, 0.0);
      transformer.appendYawRotation(Math.toRadians(30.0));

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();
      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(1.0, 0.0, 1.0);
      sensorPoseOne.appendPitchRotation(Math.toRadians(70.0 + 90.0));

      slamViewer.addSensorPose(sensorPoseOne, Color.RED);

      RigidBodyTransform pointToSensorPose = new RigidBodyTransform();
      pointToSensorPose.setTranslation(0.1, 0.1, 1.0);
      pointToSensorPose.preMultiply(sensorPoseOne);
      slamViewer.addSensorPose(pointToSensorPose, Color.ORANGE);

      sensorPoseOne.preMultiply(transformer);
      pointToSensorPose.preMultiply(transformer);
      slamViewer.addSensorPose(sensorPoseOne, Color.BLACK);
      slamViewer.addSensorPose(pointToSensorPose, Color.DARKORANGE);

      slamViewer.start("testTransformer");

      ThreadTools.sleepForever();
   }

   @Test
   public void testNewPlanarRegionComingOut()
   {
      String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
      String planarRegionsPath = "E:\\Data\\SimpleArea3\\20191127_222138_PlanarRegion\\";

      boolean doNaiveSLAM = false;
      boolean showLidarPlanarRegions = false;

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath));
      System.out.println("number of messages " + messages.size());

      IhmcSLAM slam = new IhmcSLAM(doNaiveSLAM);
      slam.addFirstFrame(messages.get(0));
      slam.addFrame(messages.get(35));
      slam.addFrame(messages.get(36));
      slam.addFrame(messages.get(37));
      slam.addFrame(messages.get(38));
      slam.addFrame(messages.get(39));
      slam.addFrame(messages.get(40));
      slam.addFrame(messages.get(42));
      slam.addFrame(messages.get(43));
      slam.addFrame(messages.get(44));
      slam.addFrame(messages.get(45));
      slam.addFrame(messages.get(46));
      slam.addFrame(messages.get(47));
      slam.addFrame(messages.get(48));
      slam.addFrame(messages.get(49));
      slam.addFrame(messages.get(50));
      slam.addFrame(messages.get(51));
      slam.addFrame(messages.get(54));

      if (doNaiveSLAM)
         slam.doNaiveSLAM();

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      for (int i = 0; i < slam.getOriginalPointCloudMap().size(); i++)
      {
         //slamViewer.addPointCloud(slam.getOriginalPointCloudMap().get(i), Color.BLACK);
         //slamViewer.addSensorPose(slam.getOriginalSensorPoses().get(i), Color.BLACK);
      }
      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         //slamViewer.addPointCloud(slam.getPointCloudMap().get(i), Color.BLUE);
         //slamViewer.addSensorPose(slam.getSensorPoses().get(i), Color.BLUE);
      }

      //      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());
      if (showLidarPlanarRegions)
      {
         PlanarRegionsList importPlanarRegionData = PlanarRegionFileTools.importPlanarRegionData(new File(planarRegionsPath));
         for (int i = 0; i < importPlanarRegionData.getNumberOfPlanarRegions(); i++)
         {
            importPlanarRegionData.getPlanarRegion(i).setRegionId(0xFF0000);
         }
         slamViewer.addPlanarRegions(importPlanarRegionData);
      }

      slamViewer.start("EndToEnd");

      //      IhmcSLAMViewer slamViewer2 = new IhmcSLAMViewer();
      //      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      //      List<IhmcSurfaceElement> mergeableSurfaceElements = slam.getSLAMFrame(3).getMergeableSurfaceElements();
      //      System.out.println(mergeableSurfaceElements.size());
      //      for (IhmcSurfaceElement element : mergeableSurfaceElements)
      //         planarRegionsList.addPlanarRegion(element.getMergeablePlanarRegion());
      //      slamViewer2.addPlanarRegions(planarRegionsList);
      //      slamViewer2.addOctree(slam.getSLAMFrame(3).getMergeableSurfaceElements(), Color.BEIGE);
      //      slamViewer2.start("part");

      ThreadTools.sleepForever();
   }

   @Test
   public void testEndToEnd()
   {

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
}
