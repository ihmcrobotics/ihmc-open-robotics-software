package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.idl.IDLSequence.Integer;
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

      if (scanPoints.length != scanColors.length)
         throw new IllegalArgumentException("wrong size!");

      this.pointCloud = scanPoints;
      this.colors = scanColors;
      this.numberOfPoints = scanPoints.length;
   }

   public PointCloudData(PointCloud2 rosPointCloud2, int maxSize)
   {
      timestamp = rosPointCloud2.getHeader().getStamp().totalNsecs();

      UnpackedPointCloud unpackPointsAndIntensities = RosPointCloudSubscriber.unpackPointsAndIntensities(rosPointCloud2);
      pointCloud = unpackPointsAndIntensities.getPoints();
      colors = unpackPointsAndIntensities.getPointColorsRGB();

      if (unpackPointsAndIntensities.getPoints().length <= maxSize)
      {
         numberOfPoints = pointCloud.length;
      }
      else
      {
         Random random = new Random();
         int currentSize = pointCloud.length;

         while (currentSize > maxSize)
         {
            int nextToRemove = random.nextInt(currentSize);
            pointCloud[nextToRemove] = pointCloud[currentSize - 1];
            colors[nextToRemove] = colors[currentSize - 1];
            pointCloud[currentSize - 1] = null;
            colors[currentSize - 1] = -1;

            currentSize--;
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

   public StereoVisionPointCloudMessage toStereoVisionPointCloudMessage()
   {
      return toStereoVisionPointCloudMessage((index, point) -> true);
   }

   public StereoVisionPointCloudMessage toStereoVisionPointCloudMessage(ScanPointFilter filter)
   {
      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();
      message.setTimestamp(timestamp);
      message.setSensorPoseConfidence(1.0);
      Float pointCloudBuffer = message.getPointCloud();
      Integer colorBuffer = message.getColors();

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = pointCloud[i];
         int color = colors[i];

         if (filter.test(i, scanPoint))
         {
            pointCloudBuffer.add(scanPoint.getX32());
            pointCloudBuffer.add(scanPoint.getY32());
            pointCloudBuffer.add(scanPoint.getZ32());
            colorBuffer.add(color);
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
}
