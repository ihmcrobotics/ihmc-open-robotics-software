package us.ihmc.robotEnvironmentAwareness.slam.tools;

import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class SimulatedStereoVisionPointCloudMessageFactory
{
   private static final Random random = new Random(0612L);
   private static final double DEFAULT_WINDOW_SIZE = 10.0;

   public static StereoVisionPointCloudMessage generateStereoVisionPointCloudMessage(RigidBodyTransform sensorPose, int numberOfPoints,
                                                                                     List<ConvexPolygon2D> convexPolygons,
                                                                                     List<RigidBodyTransform> centroidPoses)
   {
      if (convexPolygons.size() != centroidPoses.size())
         throw new RuntimeException("convexPolygons and centerPoses are mismatched." + convexPolygons.size() + " " + centroidPoses.size());

      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();
      message.setTimestamp(0);
      message.getSensorOrientation().set(sensorPose.getRotation());
      message.getSensorPosition().set(sensorPose.getTranslation());

      double totalArea = 0.0;
      for (ConvexPolygon2D polygon : convexPolygons)
      {
         totalArea = totalArea + polygon.getArea();
      }

      for (int i = 0; i < convexPolygons.size(); i++)
      {
         ConvexPolygon2D polygon = convexPolygons.get(i);
         RigidBodyTransform centroidTransform = centroidPoses.get(i);
         int expectedNumberOfPoints = (int) (numberOfPoints * (polygon.getArea() / totalArea));
         int storedNumberOfPoints = 0;
         while (true)
         {
            double randomX = (random.nextDouble() - 0.5) * DEFAULT_WINDOW_SIZE;
            double randomY = (random.nextDouble() - 0.5) * DEFAULT_WINDOW_SIZE;
            if (polygon.isPointInside(randomX, randomY))
            {
               RigidBodyTransform pointTransformer = new RigidBodyTransform(centroidTransform);
               pointTransformer.appendTranslation(randomX, randomY, 0.0);
               Vector3DBasics point3d = pointTransformer.getTranslation();
               message.getPointCloud().add(point3d.getX32());
               message.getPointCloud().add(point3d.getY32());
               message.getPointCloud().add(point3d.getZ32());
               message.getColors().add(0);
               storedNumberOfPoints++;
            }

            if (storedNumberOfPoints == expectedNumberOfPoints)
               break;
         }
      }

      return message;
   }
}
