package us.ihmc.perception.tools;

import boofcv.struct.calib.CameraPinhole;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.time.Instant;

public class PerceptionMessageTools
{
   public static void setDepthExtrinsicsFromRealsense(BytedecoRealsense sensor, ImageMessage imageMessage)
   {
      imageMessage.setFocalLengthXPixels((float) sensor.getDepthFocalLengthPixelsX());
      imageMessage.setFocalLengthYPixels((float) sensor.getDepthFocalLengthPixelsY());
      imageMessage.setPrincipalPointXPixels((float) sensor.getDepthPrincipalOffsetXPixels());
      imageMessage.setPrincipalPointYPixels((float) sensor.getDepthPrincipalOffsetYPixels());
   }

   public static void setColorExtrinsicsFromRealsense(BytedecoRealsense sensor, ImageMessage imageMessage)
   {
      imageMessage.setFocalLengthXPixels((float) sensor.getColorFocalLengthPixelsX());
      imageMessage.setFocalLengthYPixels((float) sensor.getColorFocalLengthPixelsY());
      imageMessage.setPrincipalPointXPixels((float) sensor.getColorPrincipalOffsetXPixels());
      imageMessage.setPrincipalPointYPixels((float) sensor.getColorPrincipalOffsetYPixels());
   }

   public static void toMessage(CameraPinhole cameraPinhole, ImageMessage imageMessage)
   {
      imageMessage.setFocalLengthXPixels((float) cameraPinhole.getFx());
      imageMessage.setFocalLengthYPixels((float) cameraPinhole.getFy());
      imageMessage.setPrincipalPointXPixels((float) cameraPinhole.getCx());
      imageMessage.setPrincipalPointYPixels((float) cameraPinhole.getCy());
   }

   public static void toBoofCV(ImageMessage imageMessage, CameraPinhole cameraPinhole)
   {
      cameraPinhole.setFx(imageMessage.getFocalLengthXPixels());
      cameraPinhole.setFy(imageMessage.getFocalLengthYPixels());
      cameraPinhole.setCx(imageMessage.getPrincipalPointXPixels());
      cameraPinhole.setCy(imageMessage.getPrincipalPointYPixels());
   }

   public static void publishPNGCompressedDepthImage(Mat depth16UC1Image,
                                                     ROS2Topic<ImageMessage> topic,
                                                     ImageMessage depthImageMessage,
                                                     ROS2Helper helper,
                                                     FramePose3D cameraPose,
                                                     Instant aquisitionTime,
                                                     long sequenceNumber,
                                                     int height,
                                                     int width)
   {
      BytePointer compressedDepthPointer = new BytePointer();
      BytedecoOpenCVTools.compressImagePNG(depth16UC1Image, compressedDepthPointer);
      BytedecoOpenCVTools.packImageMessage(depthImageMessage,
                                           compressedDepthPointer,
                                           cameraPose,
                                           aquisitionTime,
                                           sequenceNumber,
                                           height,
                                           width,
                                           ImageMessageFormat.DEPTH_PNG_16UC1);
      helper.publish(topic, depthImageMessage);
   }

   public static void publishJPGCompressedColorImage(Mat color8UC3Image,
                                                     Mat yuvColorImage,
                                                     ROS2Topic<ImageMessage> topic,
                                                     ImageMessage colorImageMessage,
                                                     ROS2Helper helper,
                                                     FramePose3D cameraPose,
                                                     Instant aquisitionTime,
                                                     long sequenceNumber,
                                                     int height,
                                                     int width)
   {
      BytePointer compressedColorPointer = new BytePointer();
      BytedecoOpenCVTools.compressRGBImageJPG(color8UC3Image, yuvColorImage, compressedColorPointer);
      BytedecoOpenCVTools.packImageMessage(colorImageMessage,
                                           compressedColorPointer,
                                           cameraPose,
                                           aquisitionTime,
                                           sequenceNumber,
                                           height,
                                           width,
                                           ImageMessageFormat.COLOR_JPEG_RGB8);
      helper.publish(topic, colorImageMessage);
   }

   public static void publishFramePlanarRegionsList(FramePlanarRegionsList framePlanarRegionsList,
                                                       ROS2Topic<FramePlanarRegionsListMessage> topic,
                                                       ROS2Helper ros2Helper)
   {
      ros2Helper.publish(topic, PlanarRegionMessageConverter.convertToFramePlanarRegionsListMessage(framePlanarRegionsList));
   }

   public static void publishPlanarRegionsList(PlanarRegionsList planarRegionsList, ROS2Topic<PlanarRegionsListMessage> topic, ROS2Helper ros2Helper)
   {
      ros2Helper.publish(topic, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
   }

   public static void extractImageMessageData(ImageMessage imageMessage, ByteBuffer byteBuffer)
   {
      int numberOfBytes = imageMessage.getData().size();
      int bytesPerPixel = ImageMessageFormat.getFormat(imageMessage).getBytesPerPixel();
      byteBuffer.rewind();
      byteBuffer.limit(imageMessage.getImageWidth() * imageMessage.getImageHeight() * bytesPerPixel);
      for (int i = 0; i < numberOfBytes; i++)
      {
         byteBuffer.put(imageMessage.getData().get(i));
      }
      byteBuffer.flip();
   }

   public static double calculateDelay(ImageMessage imageMessage)
   {
      return TimeTools.calculateDelay(imageMessage.getAcquisitionTime().getSecondsSinceEpoch(),
                                      imageMessage.getAcquisitionTime().getAdditionalNanos());
   }
}
