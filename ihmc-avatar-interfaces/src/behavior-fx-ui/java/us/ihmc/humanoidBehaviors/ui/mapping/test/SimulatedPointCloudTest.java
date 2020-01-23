package us.ihmc.humanoidBehaviors.ui.mapping.test;

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
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.IhmcSLAMViewer;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.slam.tools.IhmcSLAMTools;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SimulatedStereoVisionPointCloudMessageLibrary;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SimulatedPointCloudTest
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

         planarRegionsMap = buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters, polygonizerParameters);
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

   private PlanarRegionsList buildNewMap(List<PlanarRegionSegmentationRawData> newRawData, PlanarRegionsList oldMap,
                                         CustomRegionMergeParameters customRegionMergeParameters, ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                         PolygonizerParameters polygonizerParameters)
   {
      List<PlanarRegion> oldMapCopy = new ArrayList<>(oldMap.getPlanarRegionsAsList());
      List<PlanarRegion> unmergedCustomPlanarRegions = CustomPlanarRegionHandler.mergeCustomRegionsToEstimatedRegions(oldMapCopy, newRawData,
                                                                                                                      customRegionMergeParameters);

      PlanarRegionsList newMap = PlanarRegionPolygonizer.createPlanarRegionsList(newRawData, concaveHullFactoryParameters, polygonizerParameters);
      unmergedCustomPlanarRegions.forEach(newMap::addPlanarRegion);

      return newMap;
   }
}
