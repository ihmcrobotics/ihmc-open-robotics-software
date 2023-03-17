package us.ihmc.perception.tools;

import boofcv.struct.calib.CameraPinhole;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.LongPointer;
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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
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
   public static void setDepthIntrinsicsFromRealsense(BytedecoRealsense sensor, ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setFocalLengthXPixels((float) sensor.getDepthFocalLengthPixelsX());
      imageMessageToPack.setFocalLengthYPixels((float) sensor.getDepthFocalLengthPixelsY());
      imageMessageToPack.setPrincipalPointXPixels((float) sensor.getDepthPrincipalOffsetXPixels());
      imageMessageToPack.setPrincipalPointYPixels((float) sensor.getDepthPrincipalOffsetYPixels());
   }

   public static void setColorIntrinsicsFromRealsense(BytedecoRealsense sensor, ImageMessage imageMessageToPack)
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

   public static void publishCompressedDepthImage(BytePointer compressedDepthPointer,
                                                  ROS2Topic<ImageMessage> topic,
                                                  ImageMessage depthImageMessage,
                                                  ROS2Helper helper,
                                                  FramePose3D cameraPose,
                                                  Instant aquisitionTime,
                                                  long sequenceNumber,
                                                  int height,
                                                  int width,
                                                  float depthToMetersRatio)
   {
      packImageMessage(depthImageMessage,
                       compressedDepthPointer,
                       cameraPose,
                       aquisitionTime,
                       sequenceNumber,
                       height,
                       width,
                       depthToMetersRatio);

      ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(depthImageMessage);
      helper.publish(topic, depthImageMessage);
   }

   public static void publishJPGCompressedColorImage(BytePointer compressedColorPointer,
                                                     ROS2Topic<ImageMessage> topic,
                                                     ImageMessage colorImageMessage,
                                                     ROS2Helper helper,
                                                     FramePose3D cameraPose,
                                                     Instant aquisitionTime,
                                                     long sequenceNumber,
                                                     int height,
                                                     int width,
                                                     float depthToMetersRatio)
   {
      packImageMessage(colorImageMessage,
                       compressedColorPointer,
                       cameraPose,
                       aquisitionTime,
                       sequenceNumber,
                       height,
                       width,
                       depthToMetersRatio);
      ImageMessageFormat.COLOR_JPEG_YUVI420.packMessageFormat(colorImageMessage);
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
      int numberOfBytes = imageMessage.getData().size();
      dataByteBuffer.rewind();
      dataByteBuffer.limit(dataByteBuffer.capacity());
      for (int i = 0; i < numberOfBytes; i++)
      {
         dataByteBuffer.put(imageMessage.getData().get(i));
      }
      dataByteBuffer.flip();
   }

   public static void packImageMessageData(ByteBuffer dataByteBuffer, ImageMessage imageMessage)
   {
      imageMessage.getData().resetQuick();
      for (int i = 0; i < dataByteBuffer.limit(); i++)
      {
         imageMessage.getData().add(dataByteBuffer.get(i));
      }
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
                                       float depthToMetersRatio)
   {
      packImageMessageData(dataBytePointer, imageMessage);
      imageMessage.setImageHeight(height);
      imageMessage.setImageWidth(width);
      imageMessage.getPosition().set(cameraPose.getPosition());
      imageMessage.getOrientation().set(cameraPose.getOrientation());
      imageMessage.setSequenceNumber(sequenceNumber);
      MessageTools.toMessage(aquisitionTime, imageMessage.getAcquisitionTime());
      imageMessage.setDepthDiscretization(depthToMetersRatio);
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

   public static void copyToFloatPointer(IDLSequence.Float floatSequence, FloatPointer floatBuffer, int startIndex)
   {
      for (int i = 0; i < floatSequence.size(); i++)
      {
         floatBuffer.put(i + startIndex, floatSequence.get(i));
      }
   }

   public static void copyToLongPointer(IDLSequence.Long longSequence, LongPointer longPointer, int startIndex)
   {
      for (int i = 0; i < longSequence.size(); i++)
      {
         longPointer.put(i + startIndex, longSequence.get(i));
      }
   }

   public static void copyToFloatPointer(Point3D point, FloatPointer floatPointer, int startIndex)
   {
      floatPointer.put(startIndex, (float) point.getX());
      floatPointer.put(startIndex + 1, (float) point.getY());
      floatPointer.put(startIndex + 2, (float) point.getZ());
   }

   public static void copyToFloatPointer(Quaternion quaternion, FloatPointer floatPointer, int startIndex)
   {
      floatPointer.put(startIndex, (float) quaternion.getX());
      floatPointer.put(startIndex + 1, (float) quaternion.getY());
      floatPointer.put(startIndex + 2, (float) quaternion.getZ());
      floatPointer.put(startIndex + 3, (float) quaternion.getS());
   }
}
