package us.ihmc.ihmcPerception.lineSegmentDetector;

import controller_msgs.msg.dds.VideoPacket;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import sensor_msgs.Image;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ihmcPerception.camera.RosCameraInfoSubscriber;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.awt.*;
import java.awt.color.ColorSpace;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;

public class RealSenseL515DepthImageSubscriber extends AbstractRosTopicSubscriber<Image>
{
   private final ColorModel colorModel;

   private final String l515CompressedDepthTopic = "/camera/depth/image_rect_raw";
   private String l515CameraInfoTopic = "/camera/depth/camera_info";

   private BufferedImage latestBufferedDepth;

   private final IHMCROS2Publisher<VideoPacket> depthPublisher;
   private final RosCameraInfoSubscriber depthImageInfoSubscriber;

   public RealSenseL515DepthImageSubscriber(RosMainNode rosNode, ROS2Node ros2Node)
   {
      super(sensor_msgs.Image._TYPE);
      depthPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT);
      depthImageInfoSubscriber = new RosCameraInfoSubscriber(l515CameraInfoTopic);
      rosNode.attachSubscriber(l515CompressedDepthTopic, this);
      rosNode.attachSubscriber(l515CameraInfoTopic, depthImageInfoSubscriber);
      ColorSpace colorSpace = ColorSpace.getInstance(ColorSpace.CS_GRAY);
      this.colorModel = new ComponentColorModel(colorSpace, false, false, Transparency.OPAQUE, DataBuffer.TYPE_SHORT);
   }

   public void onNewMessage(sensor_msgs.Image message)
   {
      long timeStamp = message.getHeader().getStamp().totalNsecs();
      System.out.println(message.getHeight() * message.getWidth() + " " + message.getData().array().length);

      byte[] payload = message.getData().array();
      Mat mat = new Mat(message.getHeight(), message.getWidth(), CvType.CV_8U);
      mat.put(0, 0, payload);

      Mat output = new Mat(mat.height(), mat.width(), CvType.CV_8U);
      mat.convertTo(output, CvType.CV_8U);

      Imgproc.resize(output, output, new Size(1200, 900));
      HighGui.namedWindow("LineEstimator", HighGui.WINDOW_AUTOSIZE);
      HighGui.resizeWindow("LineEstimator", 1200, 900);
      HighGui.imshow("LineEstimator", output);
      int code = HighGui.waitKey(32);

      // imageReceived(timeStamp, RosTools.bufferedImageFromRosMessageRaw(colorModel, message));
   }
}
