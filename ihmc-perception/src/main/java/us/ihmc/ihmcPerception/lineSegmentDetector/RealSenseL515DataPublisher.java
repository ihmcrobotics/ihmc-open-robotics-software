package us.ihmc.ihmcPerception.lineSegmentDetector;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import sensor_msgs.PointCloud2;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.net.URI;
import java.net.URISyntaxException;

public class RealSenseL515DataPublisher
{
   private Point3D32[] points;

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;

   public static void main(String[] args) throws URISyntaxException
   {
      System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
      new RealSenseL515DataPublisher();
   }

   public RealSenseL515DataPublisher() throws URISyntaxException
   {
      RosMainNode rosMainNode = new RosMainNode(new URI("http://localhost:11311"), "RealSenseL515DataPublisher", true);
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");

      stereoVisionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.IHMC_ROOT);

      new RealSenseL515CompressedImageSubscriber(rosMainNode, ros2Node);
      new RealSenseL515DepthImageSubscriber(rosMainNode, ros2Node);
      new RealSenseL515PointCloudROS1Subscriber(rosMainNode)
      {
         @Override
         protected void cloudReceived(PointCloud2 cloud)
         {
            RosPointCloudSubscriber.UnpackedPointCloud unpackedPointCloud = RosPointCloudSubscriber.unpackPointsAndIntensities(cloud);
            points = new Point3D32[unpackedPointCloud.getPoints().length];
            int i = 0;
            for (Point3D point : unpackedPointCloud.getPoints())
            {
               Point3D32 pointf = new Point3D32(point.getX32(), point.getY32(), point.getZ32());
               points[i] = pointf;
               i++;
            }
         }
      };

      rosMainNode.execute();
   }

   public static int displayBufferedImage(BufferedImage img, int delay, boolean depth)
   {
      Mat mat;
      if (depth)
      {
         mat = new Mat(img.getHeight(), img.getWidth(), CvType.CV_16U);
      }
      else
      {
         mat = new Mat(img.getHeight(), img.getWidth(), CvType.CV_8UC3);
         byte[] bufferData = ((DataBufferByte) img.getRaster().getDataBuffer()).getData();
         mat.put(0, 0, bufferData);
         mat.convertTo(mat, CvType.CV_8U);
      }

      Imgproc.resize(mat, mat, new Size(1200, 900));
      HighGui.namedWindow("InfraDepth", HighGui.WINDOW_AUTOSIZE);
      HighGui.resizeWindow("InfraDepth", 1200, 900);
      HighGui.imshow("InfraDepth", mat);

      return HighGui.waitKey(delay);
   }
}
