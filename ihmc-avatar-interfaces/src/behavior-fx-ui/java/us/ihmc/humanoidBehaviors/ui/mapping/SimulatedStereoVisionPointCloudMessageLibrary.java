package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class SimulatedStereoVisionPointCloudMessageLibrary
{
   public static StereoVisionPointCloudMessage generateMessageSimpleStair(double stairHeight, double stairWidth, double stairLength)
   {
      return generateMessageSimpleStair(new RigidBodyTransform(), stairHeight, stairWidth, stairLength);
   }

   public static StereoVisionPointCloudMessage generateMessageSimpleStair(RigidBodyTransform preMultiplier, double stairHeight, double stairWidth,
                                                                          double stairLength)
   {
      return generateMessageSimpleStair(preMultiplier, stairHeight, stairWidth, stairLength, stairLength);
   }

   /**
    * Generate StereoVisionPointCloudMessage which is watching two stairs with front side.
    * @return
    */
   public static StereoVisionPointCloudMessage generateMessageSimpleStair(double stairHeight, double stairWidth, double stairLengthLower,
                                                                          double stairLengthUpper)
   {
      return generateMessageSimpleStair(new RigidBodyTransform(), stairHeight, stairWidth, stairLengthLower, stairLengthUpper);
   }

   public static StereoVisionPointCloudMessage generateMessageSimpleStair(RigidBodyTransform preMultiplier, double stairHeight, double stairWidth,
                                                                          double stairLengthLower, double stairLengthUpper)
   {
      int numberOfPoints = 5000;

      RigidBodyTransform sensorPose = new RigidBodyTransform();
      sensorPose.setTranslation(0.0, 0.0, 1.0);
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
      centerTwo.setTranslation(stairLengthLower / 2, 0.0, stairHeight / 2 - 0.01);
      ConvexPolygon2D polygonTwo = new ConvexPolygon2D();
      polygonTwo.addVertex(stairHeight / 2, stairWidth / 2);
      polygonTwo.addVertex(stairHeight / 2, -stairWidth / 2);
      polygonTwo.addVertex(-stairHeight / 2, -stairWidth / 2);
      polygonTwo.addVertex(-stairHeight / 2, stairWidth / 2);
      polygonTwo.update();

      RigidBodyTransform centerThr = new RigidBodyTransform();
      centerThr.setTranslation(stairLengthLower / 2 + stairLengthUpper / 2, 0.0, stairHeight);
      ConvexPolygon2D polygonThr = new ConvexPolygon2D();
      polygonThr.addVertex(stairLengthUpper / 2, stairWidth / 2);
      polygonThr.addVertex(stairLengthUpper / 2, -stairWidth / 2);
      polygonThr.addVertex(-stairLengthUpper / 2, -stairWidth / 2);
      polygonThr.addVertex(-stairLengthUpper / 2, stairWidth / 2);
      polygonThr.update();

      centroidPoses.add(centerOne);
      centroidPoses.add(centerTwo);
      centroidPoses.add(centerThr);
      convexPolygons.add(polygonOne);
      convexPolygons.add(polygonTwo);
      convexPolygons.add(polygonThr);

      for (RigidBodyTransform centroid : centroidPoses)
         centroid.preMultiply(preMultiplier);
      return SimulatedStereoVisionPointCloudMessageFactory.generateStereoVisionPointCloudMessage(sensorPose, numberOfPoints, convexPolygons, centroidPoses);
   }
}
