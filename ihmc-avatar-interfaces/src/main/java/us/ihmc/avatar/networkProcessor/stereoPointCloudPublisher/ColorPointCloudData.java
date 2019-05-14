package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

import java.util.Arrays;
import java.util.Random;

public class ColorPointCloudData
{
   private final long timestamp;
   private final int numberOfPoints;
   private final Point3D[] pointCloud;
   private final int[] colors;

   public ColorPointCloudData(long timestamp, Point3D[] scanPoints, int[] scanColors)
   {
      this.timestamp = timestamp;

      if (scanPoints.length != scanColors.length)
         throw new IllegalArgumentException("wrong size!");

      this.pointCloud = scanPoints;
      this.colors = scanColors;
      this.numberOfPoints = scanPoints.length;
   }

   public ColorPointCloudData(PointCloud2 rosPointCloud2, int maxSize)
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

   public int[] getColors()
   {
      return colors;
   }

   public StereoVisionPointCloudMessage toStereoVisionPointCloudMessage()
   {
      long timestamp = this.timestamp;
      float[] pointCloudBuffer = new float[3 * numberOfPoints];
      int[] colorsInteger;

      if (colors.length == numberOfPoints)
         colorsInteger = colors;
      else
         colorsInteger = Arrays.copyOf(colors, numberOfPoints);

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = pointCloud[i];

         pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
         pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
         pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();
      }

      return MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer, colorsInteger);
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i].applyTransform(transform);
      }
   }
}
