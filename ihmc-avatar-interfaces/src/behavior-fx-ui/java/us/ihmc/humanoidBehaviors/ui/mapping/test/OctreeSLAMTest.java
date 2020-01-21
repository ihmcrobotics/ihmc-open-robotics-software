package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.io.File;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.ui.mapping.SimulatedStereoVisionPointCloudMessageLibrary;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.octreeBasedSurfaceElement.OctreeSLAMCopy;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.IhmcSLAMViewer;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class OctreeSLAMTest
{
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

      OctreeSLAMCopy slam = new OctreeSLAMCopy(octreeResolution);
      slam.addFirstFrame(messageOne);
      slam.addFrame(driftedMessageTwo);

      IhmcSLAMViewer originalViewer = new IhmcSLAMViewer();

      originalViewer.addPointCloud(slam.getOriginalPointCloudMap().get(0), Color.BLUE);
      originalViewer.addPointCloud(slam.getOriginalPointCloudMap().get(1), Color.GREEN);
      originalViewer.start("originalViewer ");

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      slamViewer.addPointCloud(slam.getPointCloudMap().get(0), Color.BLUE);
      slamViewer.addPointCloud(slam.getPointCloudMap().get(1), Color.GREEN);
      slamViewer.start("slamViewer ");

      ThreadTools.sleepForever();
   }

   @Test
   public void testNewPlanarRegionComingOut()
   {
      String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
      String planarRegionsPath = "E:\\Data\\SimpleArea3\\20191127_222138_PlanarRegion\\";

      boolean showLidarPlanarRegions = false;

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath));
      System.out.println("number of messages " + messages.size());

      OctreeSLAMCopy slam = new OctreeSLAMCopy(0.02);
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

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      for (int i = 0; i < slam.getOriginalPointCloudMap().size(); i++)
      {
         //slamViewer.addPointCloud(slam.getOriginalPointCloudMap().get(i), Color.BLACK);
         //slamViewer.addSensorPose(slam.getOriginalSensorPoses().get(i), Color.BLACK);
      }
      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         slamViewer.addPointCloud(slam.getPointCloudMap().get(i), Color.BLUE);
         slamViewer.addSensorPose(slam.getSensorPoses().get(i), Color.BLUE);
      }

      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());
      if (showLidarPlanarRegions)
      {
         PlanarRegionsList importPlanarRegionData = PlanarRegionFileTools.importPlanarRegionData(new File(planarRegionsPath));
         for (int i = 0; i < importPlanarRegionData.getNumberOfPlanarRegions(); i++)
         {
            importPlanarRegionData.getPlanarRegion(i).setRegionId(0xFF0000);
         }
         slamViewer.addPlanarRegions(importPlanarRegionData);
      }

      slamViewer.start("testNewPlanarRegionComingOut");

      ThreadTools.sleepForever();
   }

   @Test
   public void testEndToEnd()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath));
      System.out.println("number of messages " + messages.size());

      OctreeSLAMCopy slam = new OctreeSLAMCopy(0.02);
      slam.addFirstFrame(messages.get(0));
      for (int i = 1; i < messages.size(); i++)
         slam.addFrame(messages.get(i));

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      for (int i = 0; i < slam.getOriginalPointCloudMap().size(); i++)
      {
         //slamViewer.addPointCloud(slam.getOriginalPointCloudMap().get(i), Color.BLACK);
         //slamViewer.addSensorPose(slam.getOriginalSensorPoses().get(i), Color.BLACK);
      }
      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         slamViewer.addPointCloud(slam.getPointCloudMap().get(i), Color.BLUE);
         slamViewer.addSensorPose(slam.getSensorPoses().get(i), Color.BLUE);
      }

      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());

      slamViewer.start("testNewPlanarRegionComingOut");

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
}
