package us.ihmc.robotEnvironmentAwareness.slam.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;

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

      List<Point3D> pointCloudList = new ArrayList<>();

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
               pointCloudList.add(new Point3D(pointTransformer.getTranslation()));
               storedNumberOfPoints++;
            }

            if (storedNumberOfPoints == expectedNumberOfPoints)
               break;
         }
      }

      int[] colors = new int[pointCloudList.size()];
      Point3D[] pointCloudArray = pointCloudList.toArray(new Point3D[pointCloudList.size()]);

      StereoVisionPointCloudMessage message = PointCloudCompression.compressPointCloud(0, pointCloudArray, colors, pointCloudArray.length, 0.001, null);
      message.getSensorOrientation().set(sensorPose.getRotation());
      message.getSensorPosition().set(sensorPose.getTranslation());

      return message;
   }
}
