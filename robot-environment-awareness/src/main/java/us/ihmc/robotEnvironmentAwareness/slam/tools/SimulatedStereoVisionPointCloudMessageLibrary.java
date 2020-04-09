package us.ihmc.robotEnvironmentAwareness.slam.tools;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class SimulatedStereoVisionPointCloudMessageLibrary
{
   private static final int NUMBER_OF_POINTS = 5000;
   private static final RigidBodyTransform fixedSensorPose = new RigidBodyTransform();
   static
   {
      fixedSensorPose.getTranslation().set(0.0, 0.0, 100.0);
   }

   public static StereoVisionPointCloudMessage generateMessageSimpleStair(double stairHeight, double stairWidth, double stairLength)
   {
      return generateMessageSimpleStair(new RigidBodyTransform(), stairHeight, stairWidth, stairLength);
   }

   public static StereoVisionPointCloudMessage generateMessageSimpleStair(RigidBodyTransform preMultiplier, double stairHeight, double stairWidth,
                                                                          double stairLength)
   {
      return generateMessageSimpleStair(fixedSensorPose, preMultiplier, stairHeight, stairWidth, stairLength, stairLength, true);
   }

   public static StereoVisionPointCloudMessage generateMessageSimpleStair(double stairHeight, double stairWidth, double stairLengthLower,
                                                                          double stairLengthUpper)
   {
      return generateMessageSimpleStair(fixedSensorPose, new RigidBodyTransform(), stairHeight, stairWidth, stairLengthLower, stairLengthUpper, true);
   }

   public static StereoVisionPointCloudMessage generateMessageSimpleStair(double stairHeight, double stairWidth, double stairLengthLower,
                                                                          double stairLengthUpper, boolean generateVertical)
   {
      return generateMessageSimpleStair(fixedSensorPose,
                                        new RigidBodyTransform(),
                                        stairHeight,
                                        stairWidth,
                                        stairLengthLower,
                                        stairLengthUpper,
                                        generateVertical);
   }

   public static StereoVisionPointCloudMessage generateMessageSimpleStair(RigidBodyTransform preMultiplier, double stairHeight, double stairWidth,
                                                                          double stairLengthLower, double stairLengthUpper, boolean generateVertical)
   {
      return generateMessageSimpleStair(fixedSensorPose, preMultiplier, stairHeight, stairWidth, stairLengthLower, stairLengthUpper, generateVertical);
   }

   /**
    * Generate StereoVisionPointCloudMessage which is watching two stairs with front side.
    */
   public static StereoVisionPointCloudMessage generateMessageSimpleStair(RigidBodyTransform sensorPose, RigidBodyTransform preMultiplier, double stairHeight,
                                                                          double stairWidth, double stairLengthLower, double stairLengthUpper,
                                                                          boolean generateVertical)
   {
      List<RigidBodyTransform> centroidPoses = new ArrayList<>();
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();

      RigidBodyTransform centerOne = new RigidBodyTransform();
      ConvexPolygon2D polygonOne = new ConvexPolygon2D();
      polygonOne.addVertex(stairLengthLower / 2, stairWidth / 2);
      polygonOne.addVertex(stairLengthLower / 2, -stairWidth / 2);
      polygonOne.addVertex(-stairLengthLower / 2, -stairWidth / 2);
      polygonOne.addVertex(-stairLengthLower / 2, stairWidth / 2);
      polygonOne.update();

      RigidBodyTransform centerTwo = new RigidBodyTransform();
      centerTwo.appendPitchRotation(Math.toRadians(90));
      centerTwo.getTranslation().set(stairLengthLower / 2, 0.0, stairHeight / 2 - 0.01);
      ConvexPolygon2D polygonTwo = new ConvexPolygon2D();
      polygonTwo.addVertex(stairHeight / 2, stairWidth / 2);
      polygonTwo.addVertex(stairHeight / 2, -stairWidth / 2);
      polygonTwo.addVertex(-stairHeight / 2, -stairWidth / 2);
      polygonTwo.addVertex(-stairHeight / 2, stairWidth / 2);
      polygonTwo.update();

      RigidBodyTransform centerThr = new RigidBodyTransform();
      centerThr.getTranslation().set(stairLengthLower / 2 + stairLengthUpper / 2, 0.0, stairHeight);
      ConvexPolygon2D polygonThr = new ConvexPolygon2D();
      polygonThr.addVertex(stairLengthUpper / 2, stairWidth / 2);
      polygonThr.addVertex(stairLengthUpper / 2, -stairWidth / 2);
      polygonThr.addVertex(-stairLengthUpper / 2, -stairWidth / 2);
      polygonThr.addVertex(-stairLengthUpper / 2, stairWidth / 2);
      polygonThr.update();

      centroidPoses.add(centerOne);
      if (generateVertical)
         centroidPoses.add(centerTwo);
      centroidPoses.add(centerThr);
      convexPolygons.add(polygonOne);
      if (generateVertical)
         convexPolygons.add(polygonTwo);
      convexPolygons.add(polygonThr);

      for (RigidBodyTransform centroid : centroidPoses)
         centroid.preMultiply(preMultiplier);
      return SimulatedStereoVisionPointCloudMessageFactory.generateStereoVisionPointCloudMessage(sensorPose, NUMBER_OF_POINTS, convexPolygons, centroidPoses);
   }

   public static StereoVisionPointCloudMessage generateMessageSimplePlane(RigidBodyTransform sensorPose, RigidBodyTransform preMultiplier, double length,
                                                                          double width)
   {
      List<RigidBodyTransform> centroidPoses = new ArrayList<>();
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();

      RigidBodyTransform center = new RigidBodyTransform();
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(length / 2, width / 2);
      polygon.addVertex(length / 2, -width / 2);
      polygon.addVertex(-length / 2, -width / 2);
      polygon.addVertex(-length / 2, width / 2);
      polygon.update();

      centroidPoses.add(center);
      convexPolygons.add(polygon);

      for (RigidBodyTransform centroid : centroidPoses)
         centroid.preMultiply(preMultiplier);
      return SimulatedStereoVisionPointCloudMessageFactory.generateStereoVisionPointCloudMessage(sensorPose, NUMBER_OF_POINTS, convexPolygons, centroidPoses);
   }

   public static StereoVisionPointCloudMessage generateMessageCinderBlocks(RigidBodyTransform sensorPose, RigidBodyTransform preMultiplier, double length,
                                                                           double width)
   {
      List<RigidBodyTransform> centroidPoses = new ArrayList<>();
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();

      RigidBodyTransform centerOne = new RigidBodyTransform();
      centerOne.appendTranslation(1.0, 0.0, 0.0);
      centerOne.appendRollRotation(Math.toRadians(15.0));
      centroidPoses.add(centerOne);

      RigidBodyTransform centerTwo = new RigidBodyTransform();
      centerTwo.appendTranslation(1.0, width, 0.3);
      centroidPoses.add(centerTwo);

      RigidBodyTransform centerThree = new RigidBodyTransform();
      centerThree.appendTranslation(1.0, -width, 0.15);
      centroidPoses.add(centerThree);

      for (RigidBodyTransform centroid : centroidPoses)
         centroid.preMultiply(preMultiplier);

      for (int i = 0; i < 3; i++)
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(length / 2, width / 2);
         polygon.addVertex(length / 2, -width / 2);
         polygon.addVertex(-length / 2, -width / 2);
         polygon.addVertex(-length / 2, width / 2);
         polygon.update();
         convexPolygons.add(polygon);
      }

      return SimulatedStereoVisionPointCloudMessageFactory.generateStereoVisionPointCloudMessage(sensorPose, NUMBER_OF_POINTS, convexPolygons, centroidPoses);
   }
}
