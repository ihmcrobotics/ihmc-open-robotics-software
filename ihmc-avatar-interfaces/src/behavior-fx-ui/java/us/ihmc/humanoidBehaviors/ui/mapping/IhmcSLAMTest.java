package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
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
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAMTest
{
   @Test
   public void testViewer()
   {
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
      List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath));

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

      StereoVisionPointCloudMessage message = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(stairHeight, stairWidth, stairLength);
      Point3DReadOnly[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);
      RigidBodyTransformReadOnly sensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(message);

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloud, sensorPose.getTranslation(), octreeResolution);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addPlanarRegions(planarRegionsMap);
      viewer.start("testPlanarRegionsForSimulatedPointCloudFrame");

      ThreadTools.sleepForever();
   }

   @Test
   public void testPlanarRegionsForFlatGround() // TODO: should be fixed.
   {
      double groundWidth = 1.5;
      double stairLength = 0.3;
      int numberOfMessages = 10;

      double octreeResolution = 0.02;
      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();
      List<Point3DReadOnly[]> pointCloudMap = new ArrayList<>();

      StereoVisionPointCloudMessage firstMessage = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(0.0, groundWidth, stairLength,
                                                                                                                            stairLength, false);
      Point3DReadOnly[] firstPointCloud = IhmcSLAMTools.extractPointsFromMessage(firstMessage);
      RigidBodyTransformReadOnly firstSensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(firstMessage);
      pointCloudMap.add(firstPointCloud);

      List<PlanarRegionSegmentationRawData> firstRawData = IhmcSLAMTools.computePlanarRegionRawData(firstPointCloud, firstSensorPose.getTranslation(),
                                                                                                    octreeResolution);
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

         List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloud, sensorPose.getTranslation(), octreeResolution);
         System.out.println("rawData " + rawData.size());

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

   @Test // TODO : Fix with Sylvain. see IhmcSLAMFrame.
   public void testInverseInterpolation()
   {
      double movingForward = 0.05;

      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      double octreeResolution = 0.02;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, 1.0);
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
      sensorPoseTwo.setTranslation(movingForward, 0.0, 1.0);
      StereoVisionPointCloudMessage messageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, preMultiplier,
                                                                                                                          stairHeight, stairWidth,
                                                                                                                          stairLength - movingForward,
                                                                                                                          stairLength + movingForward, false);

      IhmcSLAMFrame frameOne = new IhmcSLAMFrame(messageOne);
      IhmcSLAMFrame frameTwo = new IhmcSLAMFrame(frameOne, messageTwo);
      NormalOcTree overlappedOctreeNode = frameTwo.computeOctreeInPreviousView(octreeResolution);

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addSensorPose(frameOne.getSensorPose(), Color.BLUE);
      viewer.addSensorPose(frameTwo.getSensorPose(), Color.GREEN);
      viewer.addPointCloud(frameTwo.getPointCloud(), Color.GREEN);
      viewer.addOctree(overlappedOctreeNode, Color.BEIGE, octreeResolution);
      viewer.start("testSimulatedBadFrame");

      ThreadTools.sleepForever();
   }

   @Test
   public void testDetectingSimilarPlanarRegions()
   {

   }

   @Test
   public void testOptimization()
   {

   }

   @Test
   public void testSnapping()
   {

   }

   @Test
   public void testHeightChangeBasedFlatGroundDetection()
   {

   }

   @Test
   public void testEndToEnd()
   {

   }

}
