package us.ihmc.communication.packets;

import perception_msgs.msg.dds.LidarScanMessage;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class LidarPointCloudCompressionTest
{
   @Test
   public void testEndToEndCompression()
   {
      int numberOfTests = 100;
      Random random = new Random(3242);
      RigidBodyTransformGenerator rigidBodyTransformGenerator = new RigidBodyTransformGenerator();

      for (int i = 0; i < numberOfTests; i++)
      {
         // Generate random point cloud
         int numberOfPoints = random.nextInt(160000) + 5;
         List<Point3D> inputPoints = new ArrayList<>();

         rigidBodyTransformGenerator.identity();
         rigidBodyTransformGenerator.translate(EuclidCoreRandomTools.nextVector3D(random, 1.0));
         rigidBodyTransformGenerator.rotate(EuclidCoreRandomTools.nextRotationMatrix(random));
         RigidBodyTransform transform = rigidBodyTransformGenerator.getRigidBodyTransformCopy();

         for (int j = 0; j < numberOfPoints; j++)
         {
            Point3D point = new Point3D();
            point.setX(EuclidCoreRandomTools.nextDouble(random, 2.0));
            point.setY(EuclidCoreRandomTools.nextDouble(random, 2.0));
            point.applyTransform(transform);
            inputPoints.add(point);
         }

         // Generate compressed message
         LidarScanMessage lidarScanMessage = new LidarScanMessage();
         LidarPointCloudCompression.compressPointCloud(numberOfPoints, lidarScanMessage, (pointIndex, axisIndex) -> inputPoints.get(pointIndex).getElement32(axisIndex));

         // Uncompress message as new Point3D's
         List<Point3D> outputPoints = new ArrayList<>();
         LidarPointCloudCompression.decompressPointCloud(lidarScanMessage.getScan(), lidarScanMessage.getNumberOfPoints(), (pointIndex, x, y, z) -> outputPoints.add(new Point3D(x, y, z)));

         // Test end-to-end compression
         Assertions.assertEquals(inputPoints.size(), outputPoints.size(), "Lidar point cloud compression changed size of point cloud");

         for (int j = 0; j < inputPoints.size(); j++)
         {
            Point3D inputPoint = inputPoints.get(j);
            Point3D outputPoint = outputPoints.get(j);

            Assertions.assertEquals(inputPoint.getX(), outputPoint.getX(), 1e-6 + LidarPointCloudCompression.POINT_RESOLUTION, "Lidar point cloud compression failed");
            Assertions.assertEquals(inputPoint.getY(), outputPoint.getY(), 1e-6 + LidarPointCloudCompression.POINT_RESOLUTION, "Lidar point cloud compression failed");
            Assertions.assertEquals(inputPoint.getZ(), outputPoint.getZ(), 1e-6 + LidarPointCloudCompression.POINT_RESOLUTION, "Lidar point cloud compression failed");
         }
      }
   }
}
