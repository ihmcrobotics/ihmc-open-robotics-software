package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import lidar_obstacle_detection.Box3DO32;
import lidar_obstacle_detection.GDXBoxMessage;
import lidar_obstacle_detection.GDXBoxesMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
//import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
//import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class BoxData
{
//   private final long timestamp;
   private static int numberOfBoxs;
   private static GDXBoxMessage[] BoxCloud;
   private static Box3DO32[] BoxCloud_transformed;
   private static List<GDXBoxMessage> filteredBoxCloud = new ArrayList<>();
   private int[] colors;
   Point3D point_min;
   Point3D point_max;

   public BoxData(long timestamp, GDXBoxMessage[] scanPoints, int[] scanColors)
   {
//      this.timestamp = timestamp;
      BoxCloud = scanPoints;
      BoxCloud_transformed = new Box3DO32[scanPoints.length];
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
      BoxCloud_transformed = new Box3DO32[filteredBoxCloud.size()];
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
         double XMax = BoxCloud[i].getXMax();
         double YMax = BoxCloud[i].getYMax();
         BoxCloud[i].setXMin(-XMin);
         BoxCloud[i].setYMin(-YMin);
         BoxCloud[i].setXMax(-XMax);
         BoxCloud[i].setYMax(-YMax);

//         BoxCloud[i].set(z, -x, -y);
      }
   }

//   public void applyTransform(RigidBodyTransform transform)
//   {
//      for (int i = 0; i < numberOfBoxs; i++)
//      {
//         point_min = new Point3D(BoxCloud[i].getXMin(),
//                                 BoxCloud[i].getYMin(),
//                                 BoxCloud[i].getZMin());
//         point_max = new Point3D(BoxCloud[i].getXMax(),
//                                 BoxCloud[i].getYMax(),
//                                 BoxCloud[i].getZMax());
//         point_min.applyTransform(transform);
//         point_max.applyTransform(transform);
//         BoxCloud[i].setXMin(point_min.getX());
//         BoxCloud[i].setYMin(point_min.getY());
//         BoxCloud[i].setZMin(point_min.getZ());
//
//         BoxCloud[i].setXMax(point_max.getX());
//         BoxCloud[i].setYMax(point_max.getY());
//         BoxCloud[i].setZMax(point_max.getZ());
//
//
//      }
//   }

   public Box3DO32[] applyTransform(RigidBodyTransform transform)
   {
      for (int i = 0; i < numberOfBoxs; i++)
      {
//         point_min = new Point3D(BoxCloud[i].getXMin(),
//                                 BoxCloud[i].getYMin(),
//                                 BoxCloud[i].getZMin());
//         point_max = new Point3D(BoxCloud[i].getXMax(),
//                                 BoxCloud[i].getYMax(),
//                                 BoxCloud[i].getZMax());
//         point_min.applyTransform(transform);
//         point_max.applyTransform(transform);
//         BoxCloud[i].setXMin(point_min.getX());
//         BoxCloud[i].setYMin(point_min.getY());
//         BoxCloud[i].setZMin(point_min.getZ());
//
//         BoxCloud[i].setXMax(point_max.getX());
//         BoxCloud[i].setYMax(point_max.getY());
//         BoxCloud[i].setZMax(point_max.getZ());
         Point3D point1 = new Point3D(BoxCloud[i].getXMin(),
                                      BoxCloud[i].getYMin(),
                                      BoxCloud[i].getZMin());
         Point3D point2 = new Point3D(BoxCloud[i].getXMin(),
                                      BoxCloud[i].getYMin(),
                                      BoxCloud[i].getZMax());
         Point3D point3 = new Point3D(BoxCloud[i].getXMin(),
                                      BoxCloud[i].getYMax(),
                                      BoxCloud[i].getZMin());
         Point3D point4 = new Point3D(BoxCloud[i].getXMin(),
                                      BoxCloud[i].getYMax(),
                                      BoxCloud[i].getZMax());
         Point3D point5 = new Point3D(BoxCloud[i].getXMax(),
                                      BoxCloud[i].getYMin(),
                                      BoxCloud[i].getZMin());
         Point3D point6 = new Point3D(BoxCloud[i].getXMax(),
                                      BoxCloud[i].getYMin(),
                                      BoxCloud[i].getZMax());
         Point3D point7 = new Point3D(BoxCloud[i].getXMax(),
                                      BoxCloud[i].getYMax(),
                                      BoxCloud[i].getZMin());
         Point3D point8 = new Point3D(BoxCloud[i].getXMax(),
                                      BoxCloud[i].getYMax(),
                                      BoxCloud[i].getZMax());

         point1.applyTransform(transform);
         point2.applyTransform(transform);
         point3.applyTransform(transform);
         point4.applyTransform(transform);
         point5.applyTransform(transform);
         point6.applyTransform(transform);
         point7.applyTransform(transform);
         point8.applyTransform(transform);

         BoxCloud_transformed[i] = new Box3DO32(point1,point2,
                                                point3,point4,
                                                point5,point6,
                                                point7,point8);
      }
      return BoxCloud_transformed;
   }

   @Override
   public String toString()
   {
      return "BoxCloud data, number of points: " + numberOfBoxs;
   }
}
