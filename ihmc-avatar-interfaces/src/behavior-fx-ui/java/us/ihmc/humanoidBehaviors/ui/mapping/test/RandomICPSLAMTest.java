package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.SLAMViewer;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.slam.RandomICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.RandomICPSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SimulatedStereoVisionPointCloudMessageLibrary;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;
import us.ihmc.robotics.PlanarRegionFileTools;

public class RandomICPSLAMTest
{
   @Test
   @Disabled
   public void testSmallOverlappedFrameDetection()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      SLAMViewer slamViewer = new SLAMViewer();

      slam.addKeyFrame(messages.get(50));
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.BLUE);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.BLUE);

      slam.addFrame(messages.get(51));
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.GREEN);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.GREEN);
      slamViewer.addPointCloud(slam.getLatestFrame().getOriginalPointCloud(), Color.BLACK);

      slamViewer.start("testSmallOverlappedFrameDetection");
      ThreadTools.sleepForever();
   }

   @Test
   @Disabled
   public void testComputeDistance()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      SLAMFrame previousFrame = new SLAMFrame(messages.get(49));
      SLAMFrame frame = new SLAMFrame(previousFrame, messages.get(50));

      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      slam.addKeyFrame(messages.get(49));

      // source points.
      int numberOfSourcePoints = 500;
      double minimumOverlappedRatio = 0.3;

      NormalOcTree octree = slam.getOctree();
      Point3D[] sourcePointsToSensorPose = SLAMTools.createSourcePointsToSensorPose(frame, octree, numberOfSourcePoints, minimumOverlappedRatio, 0.1);
      Point3D[] sourcePointsToWorld = SLAMTools.createConvertedPointsToWorld(frame.getSensorPose(), sourcePointsToSensorPose);

      // compute distance.
      double totalDistance = 0;
      double totalOutliersDistance = 0;
      int numberOfInliers = 0;
      int numberOfOutliers = 0;
      for (Point3DReadOnly sourcePoint : sourcePointsToWorld)
      {
         int maximumSearchingSize = 10;
         double distance = -1.0;
         distance = SLAMTools.computeDistanceToNormalOctree(octree, sourcePoint, maximumSearchingSize);

         if (distance >= 0.0)
         {
            totalDistance = totalDistance + distance;
            numberOfInliers++;

            if (distance > octreeResolution)
            {
               totalOutliersDistance = totalOutliersDistance + distance;
               numberOfOutliers++;
            }
         }
      }
      double frameDistance = totalDistance / numberOfInliers;
      System.out.println("frameDistance " + frameDistance + " " + numberOfInliers);
      System.out.println("frameDistance " + totalOutliersDistance / numberOfOutliers + " " + numberOfOutliers);

      List<Point3D> allPoints = new ArrayList<>();
      NormalOcTreeMessage normalOctreeMessage = OcTreeMessageConverter.convertToMessage(octree);
      UIOcTree octreeForViz = new UIOcTree(normalOctreeMessage);
      for (UIOcTreeNode uiOcTreeNode : octreeForViz)
      {
         if (!uiOcTreeNode.isNormalSet() || !uiOcTreeNode.isHitLocationSet())
            continue;

         Vector3D planeNormal = new Vector3D();
         Point3D pointOnPlane = new Point3D();

         uiOcTreeNode.getNormal(planeNormal);
         uiOcTreeNode.getHitLocation(pointOnPlane);
         allPoints.add(pointOnPlane);
      }
      Point3D[] allPointss = new Point3D[allPoints.size()];
      System.out.println("allPointss " + allPointss.length);
      for (int i = 0; i < allPointss.length; i++)
         allPointss[i] = new Point3D(allPoints.get(i));

      SLAMViewer slamViewer = new SLAMViewer();

      slamViewer.addSensorPose(previousFrame.getSensorPose(), Color.BLUE);
      slamViewer.addSensorPose(frame.getSensorPose(), Color.GREEN);

      slamViewer.addPointCloud(frame.getOriginalPointCloud(), Color.BLUE);
      slamViewer.addPointCloud(sourcePointsToWorld, Color.BLACK);
      slamViewer.addPointCloud(allPointss, Color.YELLOW);

      slamViewer.start("testComputeDistance");
      ThreadTools.sleepForever();
   }

   @Test
   @Disabled
   public void testSourcePoints()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      SLAMViewer originalViewer = new SLAMViewer();

      originalViewer.addStereoMessage(messages.get(46), Color.RED);
      originalViewer.addStereoMessage(messages.get(47), Color.YELLOW);
      originalViewer.addStereoMessage(messages.get(48), Color.GREEN);
      originalViewer.addStereoMessage(messages.get(49), Color.BLUE); // Fix this frame.
      originalViewer.start("testSourcePointsInKinematicOverlappedArea originalViewer");

      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      SLAMViewer slamViewer = new SLAMViewer();

      slam.addKeyFrame(messages.get(46));
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.RED);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.RED);

      slam.addFrame(messages.get(47));
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.YELLOW);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.YELLOW);

      slam.addFrame(messages.get(48));
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.GREEN);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.GREEN);

      slam.addFrame(messages.get(49));
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.BLUE);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.BLUE);

      slamViewer.start("testSourcePointsInKinematicOverlappedArea slamViewer");

      ThreadTools.sleepForever();
   }

   @Test
   @Disabled
   public void testOptimizationForSimulatedPointCloud()
   {
      double movingForward = 0.1;
      double fixedHeight = 1.0;

      double sensorPitchAngle = Math.toRadians(90.0 + 70.0);
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.getTranslation().set(0.0, 0.0, fixedHeight);
      sensorPoseOne.appendPitchRotation(sensorPitchAngle);
      sensorPoseOne.appendYawRotation(Math.toRadians(-90.0));
      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne,
                                                                                                                          new RigidBodyTransform(),
                                                                                                                          stairHeight,
                                                                                                                          stairWidth,
                                                                                                                          stairLength,
                                                                                                                          stairLength,
                                                                                                                          true);

      double translationX = movingForward / 2;
      double translationY = 0.0;
      double translationZ = 0.0;
      double rotateY = Math.toRadians(0.0);
      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      preMultiplier.getTranslation().set(translationX, translationY, translationZ);
      preMultiplier.appendPitchRotation(rotateY);

      RigidBodyTransform sensorPoseTwo = new RigidBodyTransform();
      sensorPoseTwo.getTranslation().set(movingForward, 0.0, fixedHeight);
      sensorPoseTwo.appendPitchRotation(sensorPitchAngle);
      sensorPoseTwo.appendYawRotation(Math.toRadians(-90.0));

      RigidBodyTransform driftingTransformer = new RigidBodyTransform();
      driftingTransformer.appendTranslation(0.0, 0.05, 0.05);
      driftingTransformer.prependYawRotation(Math.toRadians(3.0));

      preMultiplier.multiply(driftingTransformer);
      sensorPoseTwo.multiply(driftingTransformer);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo,
                                                                                                                                 preMultiplier,
                                                                                                                                 stairHeight,
                                                                                                                                 stairWidth,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 true);

      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      SLAMViewer slamViewer = new SLAMViewer();

      slam.addKeyFrame(messageOne);
      slamViewer.addStereoMessage(messageOne, Color.BLUE);
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.BLUE);

      slam.addFrame(driftedMessageTwo);
      slamViewer.addStereoMessage(driftedMessageTwo, Color.BLACK);
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.GREEN);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.GREEN);

      if (RandomICPSLAM.DEBUG)
         slamViewer.addPointCloud(slam.getSourcePointsToWorldLatestFrame(), Color.RED);
      if (slam.correctedSourcePointsToWorld != null)
         slamViewer.addPointCloud(slam.correctedSourcePointsToWorld, Color.YELLOW);

      slamViewer.addOctree(slam.getOctree(), Color.ALICEBLUE, slam.getOctreeResolution(), true);

      slamViewer.start("testOptimizationForSimulatedPointCloud");

      ThreadTools.sleepForever();
   }

   @Test
   @Disabled
   public void testOptimizationForRealData()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      RandomICPSLAMParameters parameters = new RandomICPSLAMParameters();
      parameters.setNumberOfSourcePoints(100);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      SLAMViewer slamViewer = new SLAMViewer();

      slam.updateParameters(parameters);
      slam.addKeyFrame(messages.get(47));
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.BLUE);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.BLUE);

      slam.addFrame(messages.get(48));
      slamViewer.addStereoMessage(messages.get(48), Color.BLACK);
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.GREEN);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.GREEN);

      if (RandomICPSLAM.DEBUG)
         slamViewer.addPointCloud(slam.getSourcePointsToWorldLatestFrame(), Color.RED);
      if (slam.correctedSourcePointsToWorld != null)
         slamViewer.addPointCloud(slam.correctedSourcePointsToWorld, Color.YELLOW);

      slamViewer.start("testOptimizationForRealData");
      ThreadTools.sleepForever();
   }

   @Test
   @Disabled
   public void testSimpleCinderBlockField()
   {
      double sizeOfCinderBlocks = 0.5;
      RigidBodyTransform sensorPose = new RigidBodyTransform();
      sensorPose.getTranslation().set(0.8, 0.0, 1.0);
      sensorPose.appendPitchRotation(Math.toRadians(90.0 + 70.0));
      sensorPose.appendYawRotation(Math.toRadians(-90.0));
      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageCinderBlocks(sensorPose,
                                                                                                                           new RigidBodyTransform(),
                                                                                                                           sizeOfCinderBlocks,
                                                                                                                           sizeOfCinderBlocks);

      double forwardTranslation = 0.05;
      RigidBodyTransform sensorPoseTwo = new RigidBodyTransform(sensorPose);
      sensorPoseTwo.prependTranslation(forwardTranslation, 0.0, 0.0);
      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      preMultiplier.appendTranslation(0.05, 0.04, -0.03);
      StereoVisionPointCloudMessage messageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageCinderBlocks(sensorPoseTwo,
                                                                                                                           preMultiplier,
                                                                                                                           sizeOfCinderBlocks
                                                                                                                                 - forwardTranslation,
                                                                                                                           sizeOfCinderBlocks);
      RandomICPSLAMParameters parameters = new RandomICPSLAMParameters();
      parameters.setNumberOfSourcePoints(500);

      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      SLAMViewer slamViewer = new SLAMViewer();

      slam.updateParameters(parameters);
      slam.addKeyFrame(messageOne);
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.BLUE);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.BLUE);
      slamViewer.addOctree(slam.getOctree(), Color.ALICEBLUE, octreeResolution, true);

      slam.addFrame(messageTwo);
      slamViewer.addStereoMessage(messageTwo, Color.BLACK);
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.GREEN);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.GREEN);

      if (RandomICPSLAM.DEBUG)
         slamViewer.addPointCloud(slam.getSourcePointsToWorldLatestFrame(), Color.RED);
      if (slam.correctedSourcePointsToWorld != null)
         slamViewer.addPointCloud(slam.correctedSourcePointsToWorld, Color.YELLOW);

      slamViewer.start("testSimpleCinderBlockField");

      ThreadTools.sleepForever();
   }

   @Test
   @Disabled
   public void testOptimizationForRealDataUnsolved()
   {
      String stereoPath = "E:\\Data\\20200213_Round_1\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      RandomICPSLAMParameters parameters = new RandomICPSLAMParameters();
      parameters.setNumberOfSourcePoints(500);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      SLAMViewer slamViewer = new SLAMViewer();
      int frameIndexToLookAt = 17;

      for (int i = 0; i < frameIndexToLookAt; i++)
      {
         slam.addKeyFrame(messages.get(i));
      }
      slamViewer.addOctree(slam.getOctree(), Color.BLUE, octreeResolution, true);
      parameters.setWindowMargin(0.02);
      slam.updateParameters(parameters);

      slam.addFrame(messages.get(frameIndexToLookAt));
      slamViewer.addStereoMessage(messages.get(frameIndexToLookAt), Color.BLACK);
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.GREEN);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.GREEN);

      if (RandomICPSLAM.DEBUG)
         slamViewer.addPointCloud(slam.getSourcePointsToWorldLatestFrame(), Color.RED);
      if (slam.correctedSourcePointsToWorld != null)
         slamViewer.addPointCloud(slam.correctedSourcePointsToWorld, Color.YELLOW);

      slamViewer.start("testOptimizationForRealDataUnsolved");
      ThreadTools.sleepForever();
   }

   @Test
   @Disabled
   public void testRandomICPSLAMEndToEnd()
   {
      //String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      //String stereoPath = "E:\\Data\\Walking11-kinematic\\PointCloud\\";
      //String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
      //String stereoPath = "E:\\Data\\20200115_Simple Area\\PointCloud\\";
      //String stereoPath = "E:\\Data\\20200205_Complex\\PointCloud\\";
      //String stereoPath = "E:\\Data\\20200205_Tour2\\PointCloud\\";
      //String stereoPath = "E:\\Data\\20200205_Tour4\\PointCloud\\";
      String stereoPath = "E:\\Data\\20200305_Simple\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      SLAMViewer originalViewer = new SLAMViewer();
      SLAMViewer slamViewer = new SLAMViewer();

      slam.addKeyFrame(messages.get(0));
      for (int i = 1; i < messages.size(); i++)
      {
         System.out.println();
         System.out.println(" ## add frame " + i);
         slam.addFrame(messages.get(i));
         slam.updatePlanarRegionsMap();

         originalViewer.addStereoMessage(messages.get(i), Color.GREEN);

         slamViewer.addStereoMessage(messages.get(i), Color.GREEN);
         slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.BLUE);
         slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.BLUE);
      }

      slamViewer.start("testRandomICPSLAMEndToEnd slamViewer");
      originalViewer.start("testRandomICPSLAMEndToEnd originalViewer");

      SLAMViewer octreeViewer = new SLAMViewer();

      //String path = "E:\\Data\\20200205_Tour2\\20200205_174253_PlanarRegion\\";
      String path = "E:\\Data\\20200305_Simple\\20200305_104707_PlanarRegion\\";
      File file = new File(path);
      octreeViewer.addPlanarRegions(PlanarRegionFileTools.importPlanarRegionData(file));
      octreeViewer.addOctree(slam.getOctree(), Color.CORAL, slam.getOctreeResolution(), true);
      for (int i = 0; i < messages.size(); i++)
      {
         octreeViewer.addStereoMessage(messages.get(i), Color.GREEN);
      }
      octreeViewer.start("octreeViewer");

      ThreadTools.sleepForever();
   }
}
