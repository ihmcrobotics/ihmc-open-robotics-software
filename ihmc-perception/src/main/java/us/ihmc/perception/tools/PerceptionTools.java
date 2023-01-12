package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.IntrinsicParametersMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PlanarRegionsListWithPoseMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.ros2.ROS2Topic;

import java.time.Instant;

public class PerceptionTools
{
   private static Mat yuvColorImage = new Mat();

   public static void setDepthExtrinsicsFromRealsense(BytedecoRealsense sensor, IntrinsicParametersMessage intrinsicParametersMessage)
   {
      intrinsicParametersMessage.setFx(sensor.getDepthFocalLengthPixelsX());
      intrinsicParametersMessage.setFy(sensor.getDepthFocalLengthPixelsY());
      intrinsicParametersMessage.setSkew(0.0);
      intrinsicParametersMessage.setCx(sensor.getDepthPrincipalOffsetXPixels());
      intrinsicParametersMessage.setCy(sensor.getDepthPrincipalOffsetYPixels());
   }

   public static void setColorExtrinsicsFromRealsense(BytedecoRealsense sensor, IntrinsicParametersMessage intrinsicParametersMessage)
   {
      intrinsicParametersMessage.setFx(sensor.getColorFocalLengthPixelsX());
      intrinsicParametersMessage.setFy(sensor.getColorFocalLengthPixelsY());
      intrinsicParametersMessage.setSkew(0.0);
      intrinsicParametersMessage.setCx(sensor.getColorPrincipalOffsetXPixels());
      intrinsicParametersMessage.setCy(sensor.getColorPrincipalOffsetYPixels());
   }

   public static void publishCompressedDepth(Mat depth16UC1Image, ROS2Topic<ImageMessage> topic, ImageMessage depthImageMessage,
                                             ROS2Helper helper, FramePose3D cameraPose, Instant now, long seq, int height, int width)
   {
      BytePointer compressedDepthPointer = new BytePointer();
      BytedecoOpenCVTools.compressImagePNG(depth16UC1Image, compressedDepthPointer);
      BytedecoOpenCVTools.fillImageMessage(compressedDepthPointer, depthImageMessage, cameraPose, seq, height, width);
      MessageTools.toMessage(now, depthImageMessage.getAcquisitionTime());
      helper.publish(topic, depthImageMessage);
   }

   public static void publishCompressedColor(Mat color8UC3Image, ROS2Topic<ImageMessage> topic, ImageMessage colorImageMessage,
                                             ROS2Helper helper, FramePose3D cameraPose, Instant now, long seq, int height, int width)
   {
      BytePointer compressedColorPointer = new BytePointer();
      BytedecoOpenCVTools.compressRGBImageJPG(color8UC3Image, yuvColorImage, compressedColorPointer);
      BytedecoOpenCVTools.fillImageMessage(compressedColorPointer, colorImageMessage, cameraPose, seq, height, width);
      MessageTools.toMessage(now, colorImageMessage.getAcquisitionTime());
      helper.publish(topic, colorImageMessage);
   }

   public static void publishPlanarRegionsListWithPose(PlanarRegionsListWithPose planarRegionsListWithPose, ROS2Topic<PlanarRegionsListWithPoseMessage> topic,
                                                ROS2Helper ros2Helper)
   {
      ros2Helper.publish(topic, PlanarRegionMessageConverter.convertToPlanarRegionsListWithPoseMessage(planarRegionsListWithPose));
   }

   public static void publishPlanarRegionsList(PlanarRegionsList planarRegionsList, ROS2Topic<PlanarRegionsListMessage> topic,
                                               ROS2Helper ros2Helper)
   {
      ros2Helper.publish(topic, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
   }
}
