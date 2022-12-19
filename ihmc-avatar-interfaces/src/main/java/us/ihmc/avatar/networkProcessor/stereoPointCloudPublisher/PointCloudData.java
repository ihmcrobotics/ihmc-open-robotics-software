package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import net.jpountz.lz4.LZ4Compressor;
import net.jpountz.lz4.LZ4Decompressor;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.ScanPointFilter;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.elements.DiscretizedColoredPointCloud;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class PointCloudData
{
   private final long timestamp;
   private final int numberOfPoints;
   private Point3D[] pointCloud;
   private final List<Point3D> filteredPointCloud = new ArrayList<>();
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

   public PointCloudData(FusedSensorHeadPointCloudMessage sensorData, int maxSize, boolean hasColors)
   {
      double discreteResolution = DiscretizedColoredPointCloud.DISCRETE_RESOLUTION;
      timestamp = Conversions.secondsToNanoseconds(sensorData.aquisition_seconds_since_epoch_) + sensorData.aquisition_additional_nanos_;
      int intsPerPoint = DiscretizedColoredPointCloud.DISCRETE_INTS_PER_POINT;
      int bytesPerPoint = DiscretizedColoredPointCloud.DISCRETE_BYTES_PER_POINT;
      int bytesPerSegment = sensorData.getPointsPerSegment() * DiscretizedColoredPointCloud.DISCRETE_BYTES_PER_POINT;

      int numberOfBytes = sensorData.getScan().size();
      ByteBuffer compressedScanData = ByteBuffer.allocate(bytesPerSegment);
      compressedScanData.order(ByteOrder.nativeOrder());
      compressedScanData.rewind();
      compressedScanData.limit(numberOfBytes);
      for (int i = 0; i < numberOfBytes; i++)
         compressedScanData.put(sensorData.getScan().get(i));
      compressedScanData.flip();

      ByteBuffer decompressedData = ByteBuffer.allocate(bytesPerSegment);
      decompressedData.order(ByteOrder.nativeOrder());

      LZ4FastDecompressor lz4Decompressor = LZ4Factory.nativeInstance().fastDecompressor();
      decompressedData.rewind();
      lz4Decompressor.decompress(compressedScanData, decompressedData);
      decompressedData.rewind();

      pointCloud = new Point3D[sensorData.getPointsPerSegment()];
      if (hasColors)
         colors = new int[sensorData.getPointsPerSegment()];
      else
         colors = null;

      for (int i = 0; i < sensorData.getPointsPerSegment(); i++)
      {
         int x = decompressedData.getInt();
         int y = decompressedData.getInt();
         int z = decompressedData.getInt();
         int color = decompressedData.getInt();
         pointCloud[i] = new Point3D(x * discreteResolution, y * discreteResolution, z * discreteResolution);

         if (colors != null)
            colors[i] = color;
      }

      if (sensorData.getPointsPerSegment() <= maxSize)
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
               pointCloud[currentSize - 1] = null;

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

   public StereoVisionPointCloudMessage toStereoVisionPointCloudMessage(double minimumResolution)
   {
      return toStereoVisionPointCloudMessage(minimumResolution, (index, point) -> true);
   }

   public StereoVisionPointCloudMessage toStereoVisionPointCloudMessage(double minimumResolution, ScanPointFilter filter)
   {
      if (colors == null)
         throw new IllegalStateException("This pointcloud has no colors.");

      return StereoPointCloudCompression.compressPointCloud(timestamp, pointCloud, colors, numberOfPoints, minimumResolution, filter);
   }

   public LidarScanMessage toLidarScanMessage()
   {
      return toLidarScanMessage(null);
   }

   public LidarScanMessage toLidarScanMessage(ScanPointFilter filter)
   {
      return toLidarScanMessage(filter, null);
   }

   public LidarScanMessage toLidarScanMessage(ScanPointFilter filter, Point3D[] extraPoints)
   {
      LidarScanMessage message = new LidarScanMessage();
      message.setRobotTimestamp(timestamp);
      message.setSensorPoseConfidence(1.0);

      if (extraPoints != null)
      {
         Point3D[] newPointCloud = new Point3D[pointCloud.length + extraPoints.length];
         System.arraycopy(pointCloud, 0, newPointCloud, 0, pointCloud.length);
         System.arraycopy(extraPoints, 0, newPointCloud, pointCloud.length, extraPoints.length);
         pointCloud = newPointCloud;
      }

      if (filter == null)
      {
         LidarPointCloudCompression.compressPointCloud(pointCloud.length, message, (i, j) -> pointCloud[i].getElement32(j));
      }
      else
      {
         filteredPointCloud.clear();
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];

            if (filter.test(i, scanPoint))
            {
               filteredPointCloud.add(scanPoint);
            }
         }

         LidarPointCloudCompression.compressPointCloud(filteredPointCloud.size(), message, (i, j) -> filteredPointCloud.get(i).getElement32(j));
      }

      return message;
   }

   public void flipToZUp()
   {
      for (int i = 0; i < numberOfPoints; i++)
      {
         double x = pointCloud[i].getX();
         double y = pointCloud[i].getY();
         double z = pointCloud[i].getZ();
         pointCloud[i].set(z, -x, -y);
      }
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
