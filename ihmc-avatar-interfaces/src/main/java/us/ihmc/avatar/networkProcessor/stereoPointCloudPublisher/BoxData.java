package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import lidar_obstacle_detection.GDXBoxMessage;
import lidar_obstacle_detection.GDXBoxesMessage;

import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.communication.converters.ScanPointFilter;
//import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
//import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class BoxData
{
//   private final long timestamp;
   private static int numberOfBoxs;
   private static GDXBoxMessage[] BoxCloud;
   private static List<GDXBoxMessage> filteredBoxCloud = new ArrayList<>();
   private int[] colors;
   Point3D point_min;
   Point3D point_max;

   public BoxData(long timestamp, GDXBoxMessage[] scanPoints, int[] scanColors)
   {
//      this.timestamp = timestamp;
      BoxCloud = scanPoints;
      numberOfBoxs = scanPoints.length;

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

   public BoxData(GDXBoxesMessage rosBoxCloud2, int maxSize)
   {
//      timestamp = rosBoxCloud2.getHeader().getStamp().totalNsecs();

      filteredBoxCloud = rosBoxCloud2.getBoxes();
      BoxCloud = new GDXBoxMessage[filteredBoxCloud.size()];
      BoxCloud = filteredBoxCloud.toArray(BoxCloud);

      colors = new int[]{255,0,0};

      if (BoxCloud.length <= maxSize)
      {
         numberOfBoxs = BoxCloud.length;
      }
      else
      {
         Random random = new Random();
         int currentSize = BoxCloud.length;


         while (currentSize > maxSize)
         {
            int nextToRemove = random.nextInt(currentSize);
            BoxCloud[nextToRemove] = BoxCloud[currentSize - 1];
            colors[nextToRemove] = colors[currentSize - 1];
            BoxCloud[currentSize - 1] = null;
            colors[currentSize - 1] = -1;

            currentSize--;
         }

         numberOfBoxs = maxSize;
      }
   }


   public static int getNumberOfBoxs()
   {
      return numberOfBoxs;
   }

   public static GDXBoxMessage[] getBoxCloud()
   {
      return BoxCloud;
   }

   public int[] getColors()
   {
      return colors;
   }

//   public StereoVisionPointCloudMessage toStereoVisionPointCloudMessage(double minimumResolution)
//   {
//      return toStereoVisionPointCloudMessage(minimumResolution, (index, point) -> true);
//   }
//
//   public StereoVisionPointCloudMessage toStereoVisionPointCloudMessage(double minimumResolution, ScanPointFilter filter)
//   {
//      if (colors == null)
//         throw new IllegalStateException("This pointcloud has no colors.");
//
//      return StereoPointCloudCompression.compressPointCloud(timestamp, pointCloud, colors, numberOfPoints, minimumResolution, filter);
//   }
//
//   public LidarScanMessage toLidarScanMessage()
//   {
//      return toLidarScanMessage(null);
//   }
//
//   public LidarScanMessage toLidarScanMessage(ScanPointFilter filter)
//   {
//      LidarScanMessage message = new LidarScanMessage();
//      message.setRobotTimestamp(timestamp);
//      message.setSensorPoseConfidence(1.0);
//
//      if (filter == null)
//      {
//         LidarPointCloudCompression.compressPointCloud(pointCloud.length, message, (i, j) -> pointCloud[i].getElement32(j));
//      }
//      else
//      {
//         filteredPointCloud.clear();
//         for (int i = 0; i < numberOfPoints; i++)
//         {
//            Point3D scanPoint = pointCloud[i];
//
//            if (filter.test(i, scanPoint))
//            {
//               filteredPointCloud.add(scanPoint);
//            }
//         }
//
//         LidarPointCloudCompression.compressPointCloud(filteredPointCloud.size(), message, (i, j) -> filteredPointCloud.get(i).getElement32(j));
//      }
//
//      return message;
//   }

   public void flipToZUp()
   {
      for (int i = 0; i < numberOfBoxs; i++)
      {
         double XMin = BoxCloud[i].getXMin();
         double YMin = BoxCloud[i].getYMin();
         double ZMin = BoxCloud[i].getZMin();
         double XMax = BoxCloud[i].getXMax();
         double YMax = BoxCloud[i].getYMax();
         double ZMax = BoxCloud[i].getZMax();
         BoxCloud[i].setXMin(-XMin);
         BoxCloud[i].setYMin(-YMin);
         BoxCloud[i].setXMax(-XMax);
         BoxCloud[i].setYMax(-YMax);

//         BoxCloud[i].set(z, -x, -y);
      }
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      for (int i = 0; i < numberOfBoxs; i++)
      {
         point_min = new Point3D(BoxCloud[i].getXMin(),
                                 BoxCloud[i].getYMin(),
                                 BoxCloud[i].getZMin());
         point_max = new Point3D(BoxCloud[i].getXMax(),
                                 BoxCloud[i].getYMax(),
                                 BoxCloud[i].getZMax());
         point_min.applyTransform(transform);
         point_max.applyTransform(transform);
         BoxCloud[i].setXMin(point_min.getX());
         BoxCloud[i].setYMin(point_min.getY());
         BoxCloud[i].setZMin(point_min.getZ());

         BoxCloud[i].setXMax(point_max.getX());
         BoxCloud[i].setYMax(point_max.getY());
         BoxCloud[i].setZMax(point_max.getZ());


      }
   }

   @Override
   public String toString()
   {
      return "BoxCloud data, number of points: " + numberOfBoxs;
   }
}
