package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;

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
      int numberOfPoints = 5000;
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPose = new RigidBodyTransform();
      sensorPose.setTranslation(0.0, 0.0, 1.0);
      List<RigidBodyTransform> centroidPoses = new ArrayList<>();
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();

      RigidBodyTransform centerOne = new RigidBodyTransform();
      ConvexPolygon2D polygonOne = new ConvexPolygon2D();
      polygonOne.addVertex(stairLength / 2, stairWidth / 2);
      polygonOne.addVertex(stairLength / 2, -stairWidth / 2);
      polygonOne.addVertex(-stairLength / 2, -stairWidth / 2);
      polygonOne.addVertex(-stairLength / 2, stairWidth / 2);
      polygonOne.update();

      RigidBodyTransform centerTwo = new RigidBodyTransform();
      centerTwo.appendPitchRotation(Math.toRadians(90));
      centerTwo.setTranslation(stairLength / 2, 0.0, stairHeight / 2 - 0.01);
      ConvexPolygon2D polygonTwo = new ConvexPolygon2D();
      polygonTwo.addVertex(stairHeight / 2, stairWidth / 2);
      polygonTwo.addVertex(stairHeight / 2, -stairWidth / 2);
      polygonTwo.addVertex(-stairHeight / 2, -stairWidth / 2);
      polygonTwo.addVertex(-stairHeight / 2, stairWidth / 2);
      polygonTwo.update();

      RigidBodyTransform centerThr = new RigidBodyTransform();
      centerThr.setTranslation(stairLength, 0.0, stairHeight);
      ConvexPolygon2D polygonThr = new ConvexPolygon2D();
      polygonThr.addVertex(stairLength / 2, stairWidth / 2);
      polygonThr.addVertex(stairLength / 2, -stairWidth / 2);
      polygonThr.addVertex(-stairLength / 2, -stairWidth / 2);
      polygonThr.addVertex(-stairLength / 2, stairWidth / 2);
      polygonThr.update();

      centroidPoses.add(centerOne);
      centroidPoses.add(centerTwo);
      centroidPoses.add(centerThr);
      convexPolygons.add(polygonOne);
      convexPolygons.add(polygonTwo);
      convexPolygons.add(polygonThr);

      StereoVisionPointCloudMessage message = SimulatedStereoVisionPointCloudMessageFactory.generateStereoVisionPointCloudMessage(sensorPose, numberOfPoints,
                                                                                                                                  convexPolygons,
                                                                                                                                  centroidPoses);
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addStereoMessage(message);
      viewer.start("testViewer");

      ThreadTools.sleepForever();
   }

   @Test
   public void testSimulatedTranslation()
   {
      double translationX = 0.1;
      double translationY = 0.1;
      double translationZ = 0.1;

      double rotateX = Math.toRadians(5.0);
      double rotateY = Math.toRadians(5.0);
      double rotateZ = Math.toRadians(5.0);

   }

   @Test
   public void testPlanarRegionsForSimulatedPointCloudFrame()
   {

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
