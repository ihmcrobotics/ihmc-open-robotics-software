package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.util.Random;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.communication.converters.ScanPointFilter;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class PointCloudData
{
   private final long timestamp;
   private final int numberOfPoints;
   private final Point3D[] pointCloud;
   private final int[] colors;

   public PointCloudData(long timestamp, Point3D[] scanPoints, int[] scanColors)
   {
      this.timestamp = timestamp;
      pointCloud = scanPoints;
      numberOfPoints = scanPoints.length;

      if (scanColors != null)
      {
         if (scanPoints.length != scanColors.length)
            throw new IllegalArgumentException("wrong size!");
         colors = scanColors;
      }
      else
      {
         colors = null;
      }
   }

   public PointCloudData(PointCloud2 rosPointCloud2, int maxSize, boolean hasColors)
   {
      timestamp = rosPointCloud2.getHeader().getStamp().totalNsecs();

      UnpackedPointCloud unpackPointsAndIntensities = RosPointCloudSubscriber.unpackPointsAndIntensities(rosPointCloud2);
      pointCloud = unpackPointsAndIntensities.getPoints();

      if (hasColors)
         colors = unpackPointsAndIntensities.getPointColorsRGB();
      else
         colors = null;

      if (unpackPointsAndIntensities.getPoints().length <= maxSize)
      {
         numberOfPoints = pointCloud.length;
      }
      else
      {
         Random random = new Random();
         int currentSize = pointCloud.length;

         if (hasColors)
         {
            while (currentSize > maxSize)
            {
               int nextToRemove = random.nextInt(currentSize);
               pointCloud[nextToRemove] = pointCloud[currentSize - 1];
               colors[nextToRemove] = colors[currentSize - 1];
               pointCloud[currentSize - 1] = null;
               colors[currentSize - 1] = -1;

               currentSize--;
            }
         }
         else
         {
            while (currentSize > maxSize)
            {
               int nextToRemove = random.nextInt(currentSize);
               pointCloud[nextToRemove] = pointCloud[currentSize - 1];
               pointCloud[currentSize - 1] = null;

               currentSize--;
            }
         }

         numberOfPoints = maxSize;
      }
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public int getNumberOfPoints()
   {
      return numberOfPoints;
   }

   public Point3D[] getPointCloud()
   {
      return pointCloud;
   }

   public int[] getColors()
   {
      return colors;
   }

   public StereoVisionPointCloudMessage toStereoVisionPointCloudMessage(double minimumResolution)
   {
      return toStereoVisionPointCloudMessage(minimumResolution, (index, point) -> true);
   }

   public StereoVisionPointCloudMessage toStereoVisionPointCloudMessage(double minimumResolution, ScanPointFilter filter)
   {
      if (colors == null)
         throw new IllegalStateException("This pointcloud has no colors.");

      return PointCloudCompression.compressPointCloud(timestamp, pointCloud, colors, numberOfPoints, minimumResolution, filter);
   }

   public LidarScanMessage toLidarScanMessage()
   {
      return toLidarScanMessage((index, point) -> true);
   }

   public LidarScanMessage toLidarScanMessage(ScanPointFilter filter)
   {
      LidarScanMessage message = new LidarScanMessage();
      message.setRobotTimestamp(timestamp);
      message.setSensorPoseConfidence(1.0);
      Float pointCloudBuffer = message.getScan();

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = pointCloud[i];

         if (filter.test(i, scanPoint))
         {
            pointCloudBuffer.add(scanPoint.getX32());
            pointCloudBuffer.add(scanPoint.getY32());
            pointCloudBuffer.add(scanPoint.getZ32());
         }
      }

      return message;
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i].applyTransform(transform);
      }
   }

   @Override
   public String toString()
   {
      return "Pointcloud data, number of points: " + numberOfPoints;
   }
}
