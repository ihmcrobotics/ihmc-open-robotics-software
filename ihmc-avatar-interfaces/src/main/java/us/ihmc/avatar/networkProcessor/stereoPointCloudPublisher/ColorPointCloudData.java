package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class ColorPointCloudData
{
   private final long timestamp;
   private final int numberOfPoints;
   private final Point3D[] pointCloud;
   private final int[] colors;
   private final List<Integer> collidingPointIndices = new ArrayList<>();

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
      int numberOfPointsToAdd = numberOfPoints - collidingPointIndices.size();
      float[] pointCloudBuffer = new float[3 * numberOfPointsToAdd];
      int[] colorsInteger;

      if (colors.length == numberOfPointsToAdd)
      {
         colorsInteger = colors;
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];

            pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
            pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
            pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();
         }
      }
      else
      {
         int bufferIndex = 0;
         colorsInteger = Arrays.copyOf(colors, numberOfPoints);

         for (int i = 0; i < numberOfPoints; i++)
         {
            if (collidingPointIndices.contains(i))
            {
               continue;
            }
            Point3D scanPoint = pointCloud[i];

            pointCloudBuffer[3 * bufferIndex + 0] = (float) scanPoint.getX();
            pointCloudBuffer[3 * bufferIndex + 1] = (float) scanPoint.getY();
            pointCloudBuffer[3 * bufferIndex + 2] = (float) scanPoint.getZ();
            bufferIndex++;
         }

         if (bufferIndex != numberOfPointsToAdd)
         {
            LogTools.info("bufferIndex is different with numberOfPointsToAdd!!!!!!!!!!!!!!!!!!!!!");
         }
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

   public void updateCollisionBox(CollisionShapeTester collisionBoxNode)
   {
      if (collisionBoxNode != null)
      {
         collidingPointIndices.clear();
         for (int i = 0; i < numberOfPoints; i++)
         {
            if (collisionBoxNode.contains(pointCloud[i]))
               collidingPointIndices.add(i);
         }
      }
   }
}
