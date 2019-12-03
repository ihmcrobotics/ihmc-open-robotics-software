package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.List;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
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

      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(stairHeight, stairWidth,
                                                                                                                          stairLength + movingForward,
                                                                                                                          stairLength - movingForward);

      double translationX = movingForward;
      double translationY = 0.00;
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

      StereoVisionPointCloudMessage message = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(stairHeight, stairWidth, stairLength);
      Point3DReadOnly[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);
      RigidBodyTransformReadOnly sensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(message);

      double octreeResolution = 0.02;
      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloud, sensorPose.getTranslation(), octreeResolution);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addPlanarRegions(planarRegionsMap);
      viewer.start("testPlanarRegionsForSimulatedPointCloudFrame");

      ThreadTools.sleepForever();
   }

   @Test
   public void testInverseInterpolation()
   {

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
