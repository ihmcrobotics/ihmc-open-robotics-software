package us.ihmc.perception.tools;

import boofcv.struct.calib.CameraPinhole;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.time.Instant;

public class PerceptionMessageTools
{
   public static void setDepthExtrinsicsFromRealsense(BytedecoRealsense sensor, ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setFocalLengthXPixels((float) sensor.getDepthFocalLengthPixelsX());
      imageMessageToPack.setFocalLengthYPixels((float) sensor.getDepthFocalLengthPixelsY());
      imageMessageToPack.setPrincipalPointXPixels((float) sensor.getDepthPrincipalOffsetXPixels());
      imageMessageToPack.setPrincipalPointYPixels((float) sensor.getDepthPrincipalOffsetYPixels());
   }

   public static void setColorExtrinsicsFromRealsense(BytedecoRealsense sensor, ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setFocalLengthXPixels((float) sensor.getColorFocalLengthPixelsX());
      imageMessageToPack.setFocalLengthYPixels((float) sensor.getColorFocalLengthPixelsY());
      imageMessageToPack.setPrincipalPointXPixels((float) sensor.getColorPrincipalOffsetXPixels());
      imageMessageToPack.setPrincipalPointYPixels((float) sensor.getColorPrincipalOffsetYPixels());
   }

   public static void copyToMessage(CameraPinhole cameraPinhole, ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setFocalLengthXPixels((float) cameraPinhole.getFx());
      imageMessageToPack.setFocalLengthYPixels((float) cameraPinhole.getFy());
      imageMessageToPack.setPrincipalPointXPixels((float) cameraPinhole.getCx());
      imageMessageToPack.setPrincipalPointYPixels((float) cameraPinhole.getCy());
   }

   public static void toBoofCV(ImageMessage imageMessage, CameraPinhole cameraPinholeToPack)
   {
      cameraPinholeToPack.setFx(imageMessage.getFocalLengthXPixels());
      cameraPinholeToPack.setFy(imageMessage.getFocalLengthYPixels());
      cameraPinholeToPack.setCx(imageMessage.getPrincipalPointXPixels());
      cameraPinholeToPack.setCy(imageMessage.getPrincipalPointYPixels());
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
      packImageMessage(depthImageMessage,
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
      packImageMessage(colorImageMessage,
                                           compressedColorPointer,
                                           cameraPose,
                                           aquisitionTime,
                                           sequenceNumber,
                                           height,
                                           width,
                                           ImageMessageFormat.COLOR_JPEG_YUVI420);
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

   public static void extractImageMessageData(ImageMessage imageMessage, ByteBuffer dataByteBuffer)
   {
      MessageTools.extractIDLSequence(imageMessage.getData(), dataByteBuffer);
   }

   public static void packImageMessageData(ByteBuffer dataByteBuffer, ImageMessage imageMessage)
   {
      MessageTools.packIDLSequence(dataByteBuffer, imageMessage.getData());
   }

   public static void packImageMessageData(BytePointer dataBytePointer, ImageMessage imageMessage)
   {
      imageMessage.getData().resetQuick();
      for (int i = 0; i < dataBytePointer.limit(); i++)
      {
         imageMessage.getData().add(dataBytePointer.get(i));
      }
   }

   public static void packImageMessage(ImageMessage imageMessage,
                                       BytePointer dataBytePointer,
                                       FramePose3D cameraPose,
                                       Instant aquisitionTime,
                                       long sequenceNumber,
                                       int height,
                                       int width,
                                       ImageMessageFormat format)
   {
      packImageMessageData(dataBytePointer, imageMessage);
      format.packMessageFormat(imageMessage);
      imageMessage.setImageHeight(height);
      imageMessage.setImageWidth(width);
      imageMessage.getPosition().set(cameraPose.getPosition());
      imageMessage.getOrientation().set(cameraPose.getOrientation());
      imageMessage.setSequenceNumber(sequenceNumber);
      MessageTools.toMessage(aquisitionTime, imageMessage.getAcquisitionTime());
   }

   public static void packVideoPacket(BytePointer compressedBytes, byte[] heapArray, VideoPacket packet, int height, int width, long nanoTime)
   {
      compressedBytes.asBuffer().get(heapArray, 0, compressedBytes.asBuffer().remaining());
      packet.setTimestamp(nanoTime);
      packet.getData().resetQuick();
      packet.getData().add(heapArray);
      packet.setImageHeight(height);
      packet.setImageWidth(width);
      packet.setVideoSource(VideoSource.MULTISENSE_LEFT_EYE.toByte());
   }

   public static void displayVideoPacketColor(VideoPacket videoPacket)
   {
      Mat colorImage = new Mat(videoPacket.getImageHeight(), videoPacket.getImageWidth(), opencv_core.CV_8UC3);
      byte[] compressedByteArray = videoPacket.getData().toArray();
      BytedecoOpenCVTools.decompressJPG(compressedByteArray, colorImage);
      BytedecoOpenCVTools.display("Color Image", colorImage, 1);
   }
}
