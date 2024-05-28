package us.ihmc.perception.tools;

import boofcv.struct.calib.CameraPinhole;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.*;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.realsense.RealsenseDevice;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.perception.gpuHeightMap.HeightMapTools;

import java.nio.ByteBuffer;
import java.time.Instant;

public class PerceptionMessageTools
{
   public static void setDepthIntrinsicsFromRealsense(RealsenseDevice sensor, ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setFocalLengthXPixels((float) sensor.getDepthFocalLengthPixelsX());
      imageMessageToPack.setFocalLengthYPixels((float) sensor.getDepthFocalLengthPixelsY());
      imageMessageToPack.setPrincipalPointXPixels((float) sensor.getDepthPrincipalOffsetXPixels());
      imageMessageToPack.setPrincipalPointYPixels((float) sensor.getDepthPrincipalOffsetYPixels());
   }

   public static void setColorIntrinsicsFromRealsense(RealsenseDevice sensor, ImageMessage imageMessageToPack)
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
      packImageMessage(depthImageMessage, compressedDepthPointer, cameraPose, aquisitionTime, sequenceNumber, height, width, depthToMetersRatio);

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
      packImageMessage(colorImageMessage, compressedColorPointer, cameraPose, aquisitionTime, sequenceNumber, height, width, depthToMetersRatio);
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
      OpenCVTools.decompressJPG(compressedByteArray, colorImage);
      PerceptionDebugTools.display("Color Image", colorImage, 1);
   }

   public static void copyToBytePointer(IDLSequence.Byte sourceIDLSequence, BytePointer pointerToPack)
   {
      pointerToPack.position(0);
      pointerToPack.limit(sourceIDLSequence.size());
      for (int i = 0; i < sourceIDLSequence.size(); i++)
      {
         pointerToPack.put(i, sourceIDLSequence.get(i));
      }
   }

   public static void copyToFloatPointer(IDLSequence.Float sourceIDLSequence, FloatPointer floatPointerToPack, int startIndex)
   {
      for (int i = 0; i < sourceIDLSequence.size(); i++)
      {
         floatPointerToPack.put(i + startIndex, sourceIDLSequence.get(i));
      }
   }

   public static void copyToLongPointer(IDLSequence.Long sourceIDLSequence, LongPointer longPointerToPack, int startIndex)
   {
      for (int i = 0; i < sourceIDLSequence.size(); i++)
      {
         longPointerToPack.put(i + startIndex, sourceIDLSequence.get(i));
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

   public static void convertToHeightMapData(Mat heightMapPointer, HeightMapData heightMapData, Point3D gridCenter, float widthInMeters, float cellSizeInMeters)
   {
      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
      int cellsPerAxis = 2 * centerIndex + 1;

      heightMapData.setGridCenter(gridCenter.getX(), gridCenter.getY());

      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            int height = ((int) heightMapPointer.ptr(xIndex, yIndex).getShort() & 0xFFFF);
            float cellHeight = (float) ((float) (height) / RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor())
                               - (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightOffset();

            int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
            heightMapData.setHeightAt(key, cellHeight);
         }
      }
   }

   public static void convertToHeightMapImage(ImageMessage imageMessage,
                                              Mat heightMapImageToPack,
                                              ByteBuffer compressedByteBuffer,
                                              BytePointer byteBufferAccessPointer,
                                              Mat compressedBytesMat)
   {
      int numberOfBytes = imageMessage.getData().size();
      compressedByteBuffer.rewind();
      compressedByteBuffer.limit(numberOfBytes);
      for (int i = 0; i < numberOfBytes; i++)
      {
         compressedByteBuffer.put(imageMessage.getData().get(i));
      }
      compressedByteBuffer.flip();

      compressedBytesMat.cols(numberOfBytes);
      compressedBytesMat.data(byteBufferAccessPointer);

      // Decompress the height map image
      opencv_imgcodecs.imdecode(compressedBytesMat, opencv_imgcodecs.IMREAD_UNCHANGED, heightMapImageToPack);
   }

   public static void unpackMessage(HeightMapMessage heightMapMessage, TerrainMapData terrainMapData)
   {
      terrainMapData.getHeightMapCenter().set(heightMapMessage.getGridCenterX(), heightMapMessage.getGridCenterY());
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapMessage.getGridSizeXy(), heightMapMessage.getXyResolution());

      for (int i = 0; i < heightMapMessage.getHeights().size(); i++)
      {
         int key = heightMapMessage.getKeys().get(i);
         int xIndex = HeightMapTools.keyToXIndex(key, centerIndex);
         int yIndex = HeightMapTools.keyToYIndex(key, centerIndex);
         double height = heightMapMessage.getHeights().get(key);
         terrainMapData.setHeightLocal((float) height, yIndex, xIndex);
      }
   }
}
