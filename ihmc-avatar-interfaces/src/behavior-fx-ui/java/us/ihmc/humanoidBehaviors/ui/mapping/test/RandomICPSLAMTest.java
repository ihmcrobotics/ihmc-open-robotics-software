package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.IhmcSLAMViewer;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.slam.IhmcSLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.RandomICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.tools.IhmcSLAMTools;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SimulatedStereoVisionPointCloudMessageLibrary;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;

public class RandomICPSLAMTest
{
   @Test
   public void visualizeBadFrameForFlatTop()
   {
      String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      slam.addFirstFrame(messages.get(42));
      slam.addFrame(messages.get(43));
      slam.addFrame(messages.get(44));

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      slamViewer.addPointCloud(slam.getPointCloudMap().get(0), Color.BLUE);
      slamViewer.addPointCloud(slam.getPointCloudMap().get(1), Color.GREEN);
      slamViewer.addPointCloud(slam.getPointCloudMap().get(2), Color.BLACK);

      slamViewer.addSensorPose(slam.getSensorPoses().get(0), Color.BLUE);
      slamViewer.addSensorPose(slam.getSensorPoses().get(1), Color.GREEN);
      slamViewer.addSensorPose(slam.getSensorPoses().get(2), Color.BLACK);

      slamViewer.start("visualizeBadFrameForNewPlane slamViewer");
      ThreadTools.sleepForever();
   }

   @Test
   public void testSmallOverlappedFrameDetection()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      slam.addFirstFrame(messages.get(50));
      slam.addFrame(messages.get(51));

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      slamViewer.addSensorPose(slam.getSensorPoses().get(0), Color.BLUE);
      slamViewer.addSensorPose(slam.getSensorPoses().get(1), Color.GREEN);

      slamViewer.addPointCloud(slam.getPointCloudMap().get(0), Color.BLUE);
      slamViewer.addPointCloud(slam.getOriginalPointCloudMap().get(1), Color.BLACK);
      slamViewer.addPointCloud(slam.getPointCloudMap().get(1), Color.GREEN);

      slamViewer.start("testSmallOverlappedFrameDetection");
      ThreadTools.sleepForever();
   }

   @Test
   public void testComputeDistance()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      IhmcSLAMFrame previousFrame = new IhmcSLAMFrame(messages.get(49));
      IhmcSLAMFrame frame = new IhmcSLAMFrame(previousFrame, messages.get(50));

      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      slam.addFirstFrame(messages.get(49));

      // source points.
      int numberOfSourcePoints = 500;
      double minimumOverlappedRatio = 0.3;

      NormalOcTree octree = slam.getOctree();
      Point3D[] sourcePointsToSensorPose = IhmcSLAMTools.createSourcePointsToSensorPose(frame, octree, numberOfSourcePoints,
                                                                                                          minimumOverlappedRatio);
      Point3D[] sourcePointsToWorld = IhmcSLAMTools.createConvertedPointsToWorld(frame.getSensorPose(), sourcePointsToSensorPose);

      // compute distance.
      double totalDistance = 0;
      double totalOutliersDistance = 0;
      int numberOfInliers = 0;
      int numberOfOutliers = 0;
      for (Point3DReadOnly sourcePoint : sourcePointsToWorld)
      {
         int maximumSearchingSize = 10;
         double distance = -1.0;
         distance = IhmcSLAMTools.computeDistanceToNormalOctree(octree, sourcePoint, maximumSearchingSize);

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

      Point3D[] closestPoints = new Point3D[IhmcSLAMTools.closestOctreePoints.size()];
      System.out.println("closestPoints " + closestPoints.length);
      for (int i = 0; i < closestPoints.length; i++)
         closestPoints[i] = new Point3D(IhmcSLAMTools.closestOctreePoints.get(i));

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

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      slamViewer.addSensorPose(previousFrame.getSensorPose(), Color.BLUE);
      slamViewer.addSensorPose(frame.getSensorPose(), Color.GREEN);

      slamViewer.addPointCloud(frame.getOriginalPointCloud(), Color.BLUE);
      slamViewer.addPointCloud(sourcePointsToWorld, Color.BLACK);
      slamViewer.addPointCloud(allPointss, Color.YELLOW);
      slamViewer.addPointCloud(closestPoints, Color.RED);

      slamViewer.start("testComputeDistance");
      ThreadTools.sleepForever();
   }

   @Test
   public void testSourcePoints()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      // 49 - 50. 0.4 0.3 0.0 is the best. 50-53 0.4 0.4 -0.1 is good
      // 54-58 check merge immediately enough part.
      // 45-49 with 0.4 0.3 0.0 is the good test set.
      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      slam.addFirstFrame(messages.get(40));
      slam.addFrame(messages.get(47));
      slam.addFrame(messages.get(48));
      slam.addFrame(messages.get(49));

      IhmcSLAMViewer originalViewer = new IhmcSLAMViewer();

      originalViewer.addSensorPose(slam.getSensorPoses().get(0), Color.BLUE);
      originalViewer.addSensorPose(slam.getSensorPoses().get(1), Color.GREEN);

      originalViewer.addPointCloud(slam.getOriginalPointCloudMap().get(0), Color.RED);
      originalViewer.addPointCloud(slam.getOriginalPointCloudMap().get(1), Color.YELLOW);
      originalViewer.addPointCloud(slam.getOriginalPointCloudMap().get(2), Color.GREEN);
      originalViewer.addPointCloud(slam.getOriginalPointCloudMap().get(3), Color.BLUE);

      originalViewer.start("testSourcePointsInKinematicOverlappedArea originalViewer");

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      slamViewer.addSensorPose(slam.getSensorPoses().get(0), Color.BLUE);
      slamViewer.addSensorPose(slam.getSensorPoses().get(1), Color.GREEN);

      slamViewer.addPointCloud(slam.getPointCloudMap().get(0), Color.RED);
      slamViewer.addPointCloud(slam.getPointCloudMap().get(1), Color.YELLOW);
      slamViewer.addPointCloud(slam.getPointCloudMap().get(2), Color.GREEN);
      slamViewer.addPointCloud(slam.getPointCloudMap().get(3), Color.BLUE);

      slamViewer.start("testSourcePointsInKinematicOverlappedArea slamViewer");

      ThreadTools.sleepForever();
   }

   @Test
   public void testOptimizationForSimulatedPointCloud()
   {
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
      sensorPoseTwo.appendYawRotation(Math.toRadians(-90.0));

      RigidBodyTransform driftingTransformer = new RigidBodyTransform();
      driftingTransformer.appendTranslation(0.0, 0.05, 0.05);
      driftingTransformer.prependYawRotation(Math.toRadians(3.0));

      preMultiplier.multiply(driftingTransformer);
      sensorPoseTwo.multiply(driftingTransformer);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, preMultiplier,
                                                                                                                                 stairHeight, stairWidth,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 true);

      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);

      slam.addFirstFrame(messageOne);
      slam.addFrame(driftedMessageTwo);

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();
      slamViewer.addPointCloud(slam.getSLAMFrame(0).getOriginalPointCloud(), Color.BLUE);
      slamViewer.addPointCloud(slam.getSLAMFrame(1).getOriginalPointCloud(), Color.BLACK);
      slamViewer.addSensorPose(slam.getSLAMFrame(1).getOriginalSensorPose(), Color.BLACK);

      slamViewer.addPointCloud(slam.getSLAMFrame(1).getPointCloud(), Color.GREEN);
      slamViewer.addSensorPose(slam.getSLAMFrame(1).getSensorPose(), Color.GREEN);

      if (RandomICPSLAM.DEBUG)
         slamViewer.addPointCloud(slam.sourcePointsToWorld, Color.RED);
      if (slam.correctedSourcePointsToWorld != null)
         slamViewer.addPointCloud(slam.correctedSourcePointsToWorld, Color.YELLOW);

      Point3D[] closestPoints = new Point3D[IhmcSLAMTools.closestOctreePoints.size()];
      System.out.println("closestPoints " + closestPoints.length);
      for (int i = 0; i < closestPoints.length; i++)
         closestPoints[i] = new Point3D(IhmcSLAMTools.closestOctreePoints.get(i));
      slamViewer.addPointCloud(closestPoints, Color.PINK);

      Point3D[] ruler = new Point3D[50];
      for (int i = 0; i < 50; i++)
         ruler[i] = new Point3D(0.15, 0.0, (double) i * octreeResolution);
      slamViewer.addPointCloud(ruler, Color.ALICEBLUE);

      slamViewer.addOctree(slam.getOctree(), Color.ALICEBLUE, slam.getOctreeResolution(), true);

      slamViewer.start("testOptimizationForSimulatedPointCloud");

      ThreadTools.sleepForever();
   }

   @Test
   public void testOptimizationForRealData()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      slam.addFirstFrame(messages.get(47));
      slam.addFrame(messages.get(48));

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      slamViewer.addSensorPose(slam.getSensorPoses().get(0), Color.BLUE);
      slamViewer.addSensorPose(slam.getSensorPoses().get(1), Color.GREEN);

      slamViewer.addPointCloud(slam.getPointCloudMap().get(0), Color.BLUE);
      slamViewer.addPointCloud(slam.getOriginalPointCloudMap().get(1), Color.BLACK);
      slamViewer.addPointCloud(slam.getPointCloudMap().get(1), Color.GREEN);

      if (RandomICPSLAM.DEBUG)
         slamViewer.addPointCloud(slam.sourcePointsToWorld, Color.RED);
      if (slam.correctedSourcePointsToWorld != null)
         slamViewer.addPointCloud(slam.correctedSourcePointsToWorld, Color.YELLOW);

      slamViewer.start("testOptimizationForRealData");
      ThreadTools.sleepForever();
   }

   @Test
   public void testRandomICPSLAMEndToEnd()
   {
      //String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      String stereoPath = "E:\\Data\\Walking11-kinematic\\PointCloud\\";
      //String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
      //String stereoPath = "E:\\Data\\20200115_Simple Area\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      slam.addFirstFrame(messages.get(0));
      for (int i = 1; i < messages.size(); i++)
      {
         System.out.println();
         System.out.println(" ## add frame " + i);
         slam.addFrame(messages.get(i));
      }
      System.out.println(slam.getPointCloudMap().size());

      slam.updatePlanarRegionsMap();

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         slamViewer.addPointCloud(slam.getPointCloudMap().get(i), Color.BLUE);
         slamViewer.addSensorPose(slam.getSensorPoses().get(i), Color.BLUE);
      }
      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());
      slamViewer.start("testRandomICPSLAMEndToEnd slamViewer");

      IhmcSLAMViewer originalViewer = new IhmcSLAMViewer();

      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         originalViewer.addPointCloud(slam.getOriginalPointCloudMap().get(i), Color.GREEN);
         originalViewer.addSensorPose(slam.getOriginalSensorPoses().get(i), Color.GREEN);
      }
      originalViewer.start("testRandomICPSLAMEndToEnd originalViewer");

      IhmcSLAMViewer octreeViewer = new IhmcSLAMViewer();
      octreeViewer.addOctree(slam.getOctree(), Color.ALICEBLUE, slam.getOctreeResolution(), true);
      octreeViewer.start("octreeViewer");

      ThreadTools.sleepForever();
   }
}
