package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import javax.annotation.Nullable;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.time.Instant;

public class PerceptionMessageTools
{
   public static void packCameraIntrinsics(CameraIntrinsics cameraIntrinsics, ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setFocalLengthXPixels((float) cameraIntrinsics.getFx());
      imageMessageToPack.setFocalLengthYPixels((float) cameraIntrinsics.getFy());
      imageMessageToPack.setPrincipalPointXPixels((float) cameraIntrinsics.getCx());
      imageMessageToPack.setPrincipalPointYPixels((float) cameraIntrinsics.getCy());
      imageMessageToPack.setImageWidth(cameraIntrinsics.getWidth());
      imageMessageToPack.setImageHeight(cameraIntrinsics.getHeight());
   }

   public static void publishCompressedDepthImage(BytePointer compressedDepthPointer,
                                                  ImageMessage depthImageMessage,
                                                  ROS2Publisher<ImageMessage> publisher,
                                                  Pose3DReadOnly cameraPose,
                                                  Instant acquisitionTime,
                                                  long sequenceNumber,
                                                  int height,
                                                  int width,
                                                  float depthToMetersRatio)
   {
      packCompressedDepthImage(compressedDepthPointer, depthImageMessage, cameraPose, acquisitionTime, sequenceNumber, height, width, depthToMetersRatio);
      publisher.publish(depthImageMessage);
   }

   public static void publishJPGCompressedColorImage(BytePointer compressedColorPointer,
                                                     ROS2Topic<ImageMessage> topic,
                                                     ImageMessage colorImageMessage,
                                                     ROS2Helper helper,
                                                     Pose3DReadOnly cameraPose,
                                                     Instant acquisitionTime,
                                                     long sequenceNumber,
                                                     int height,
                                                     int width,
                                                     float depthToMetersRatio)
   {
      packJPGCompressedColorImage(compressedColorPointer, colorImageMessage, cameraPose, acquisitionTime, sequenceNumber, height, width, depthToMetersRatio);
      helper.publish(topic, colorImageMessage);
   }

   public static void publishFramePlanarRegionsList(FramePlanarRegionsList framePlanarRegionsList,
                                                    ROS2Topic<FramePlanarRegionsListMessage> topic,
                                                    ROS2Helper ros2)
   {
      ros2.publish(topic, PlanarRegionMessageConverter.convertToFramePlanarRegionsListMessage(framePlanarRegionsList));
   }

   public static void packImageMessageData(ImageMessage imageMessage, ByteBuffer dataBuffer)
   {
      imageMessage.getData().resetQuick();
      imageMessage.getData().getBuffer().put(dataBuffer);
   }

   public static void packImageMessageData(ImageMessage imageMessage, BytePointer dataPointer)
   {
      packImageMessageData(imageMessage, dataPointer.asBuffer());
   }

   /**
    * Packs the {@link ImageMessage} with the {@link RawImage} metadata,
    * EXCEPT:
    * <ul>
    * <li> the compressed data, </li>
    * <li> the {@link CompressionType}, </li>
    * <li> the ouster beam altitude angles, and </li>
    * <li> the ouster beam azimuth angles </li>
    * </ul>
    * To pack everything, use this instead:
    * {@link #packImageMessage(RawImage, BytePointer, CompressionType, ImageMessage)}
    * @param messageToPack The message to pack
    * @param image The image from which metadata is taken
    */
   public static void packImageMessageMetadata(ImageMessage messageToPack, RawImage image)
   {
      packCameraIntrinsics(image.getIntrinsicsCopy(), messageToPack);
      messageToPack.setPixelFormat(image.getPixelFormat().toByte());
      messageToPack.setCameraModel(image.getCameraModel().toByte());
      messageToPack.setDepthDiscretization(image.getDepthDiscretization());
      messageToPack.setSequenceNumber(image.getSequenceNumber());
      MessageTools.toMessage(image.getAcquisitionTime(), messageToPack.getAcquisitionTime());
      messageToPack.getPosition().set(image.getTranslation());
      messageToPack.getOrientation().set(image.getRotation());
   }

   public static void packCompressedDepthImage(BytePointer compressedDepthPointer,
                                               ImageMessage depthImageMessage,
                                               Pose3DReadOnly cameraPose,
                                               Instant acquisitionTime,
                                               long sequenceNumber,
                                               int height,
                                               int width,
                                               float depthToMetersRatio)
   {
      packImageMessage(depthImageMessage, compressedDepthPointer, cameraPose, acquisitionTime, sequenceNumber, height, width, depthToMetersRatio);
      depthImageMessage.setPixelFormat(PixelFormat.GRAY16.toByte());
      depthImageMessage.setCompressionType(CompressionType.PNG.toByte());
   }

   public static void packJPGCompressedColorImage(BytePointer compressedColorPointer,
                                                  ImageMessage colorImageMessage,
                                                  Pose3DReadOnly cameraPose,
                                                  Instant acquisitionTime,
                                                  long sequenceNumber,
                                                  int height,
                                                  int width,
                                                  float depthToMetersRatio)
   {
      packImageMessage(colorImageMessage, compressedColorPointer, cameraPose, acquisitionTime, sequenceNumber, height, width, depthToMetersRatio);
      colorImageMessage.setPixelFormat(PixelFormat.YUV_I420.toByte());
      colorImageMessage.setCompressionType(CompressionType.JPEG.toByte());
   }

   public static void packImageMessage(ImageMessage imageMessage,
                                       BytePointer dataBytePointer,
                                       Pose3DReadOnly cameraPose,
                                       Instant acquisitionTime,
                                       long sequenceNumber,
                                       int height,
                                       int width,
                                       float depthToMetersRatio)
   {
      packImageMessageData(imageMessage, dataBytePointer);
      imageMessage.setImageHeight(height);
      imageMessage.setImageWidth(width);
      imageMessage.getPosition().set(cameraPose.getPosition());
      imageMessage.getOrientation().set(cameraPose.getOrientation());
      imageMessage.setSequenceNumber(sequenceNumber);
      MessageTools.toMessage(acquisitionTime, imageMessage.getAcquisitionTime());
      imageMessage.setDepthDiscretization(depthToMetersRatio);
   }

   public static void packImageMessage(RawImage originalImage, BytePointer compressedData, CompressionType compressionType, ImageMessage imageMessageToPack)
   {
      packImageMessage(originalImage, compressedData, compressionType, null, null, imageMessageToPack);
   }

   public static void packImageMessage(RawImage originalImage,
                                       BytePointer compressedData,
                                       CompressionType compressionType,
                                       @Nullable ByteBuffer ousterBeamAltitudeAngles,
                                       @Nullable ByteBuffer ousterBeamAzimuthAngles,
                                       ImageMessage imageMessageToPack)
   {
      packImageMessageMetadata(imageMessageToPack, originalImage);
      packImageMessageData(imageMessageToPack, compressedData);
      imageMessageToPack.setCompressionType(compressionType.toByte());
      if (ousterBeamAltitudeAngles != null)
         MessageTools.packIDLSequence(ousterBeamAltitudeAngles, imageMessageToPack.getOusterBeamAltitudeAngles());
      if (ousterBeamAzimuthAngles != null)
         MessageTools.packIDLSequence(ousterBeamAzimuthAngles, imageMessageToPack.getOusterBeamAzimuthAngles());
   }

   public static void packImageMessage(RawImage image, String cameraFrameId, Image messageToPack)
   {
      // Set the header
      Instant imageAcquisitionTime = image.getAcquisitionTime();
      messageToPack.getHeader().getStamp().setSec((int) imageAcquisitionTime.getEpochSecond());
      messageToPack.getHeader().getStamp().setNanosec(imageAcquisitionTime.getNano());
      messageToPack.getHeader().setFrameId(cameraFrameId);

      // Set dimensions
      messageToPack.setWidth(image.getWidth());
      messageToPack.setHeight(image.getHeight());

      // Set encoding
      PixelFormat pixelFormat = image.getPixelFormat();
      String encoding = switch (pixelFormat)
      {
         case BGR8, BGRA8, RGB8, RGBA8 -> pixelFormat.name().toLowerCase();
         case GRAY8, GRAY16 -> pixelFormat.name().toLowerCase().replace("gray", "mono");
         default -> getOpenCVTypeString(image.getOpenCVType());
      };
      messageToPack.setEncoding(encoding);

      // Get the message's internal buffer
      ByteBuffer dataBuffer = messageToPack.getData().getBuffer();

      // Set byte order
      messageToPack.setIsBigendian((byte) (dataBuffer.order().equals(ByteOrder.BIG_ENDIAN) ? 1 : 0));

      // Set step
      Mat cpuImage = image.getCpuImageMat();
      messageToPack.setStep(cpuImage.step());

      // Set data
      int memorySize = (int) OpenCVTools.memorySize(cpuImage);
      dataBuffer.position(0).put(cpuImage.data().limit(memorySize).asByteBuffer());
   }

   private static String getOpenCVTypeString(int openCVType)
   {
      // Reverse the opencv_core.CV_MAKETYPE method to get the type depth and number of channels
      int depth = openCVType & opencv_core.CV_MAT_DEPTH_MASK;
      int channels = ((openCVType - depth) >> opencv_core.CV_CN_SHIFT) + 1;

      StringBuilder typeString = new StringBuilder(5);
      switch (depth)
      {
         case opencv_core.CV_8U -> typeString.append("8UC");
         case opencv_core.CV_8S -> typeString.append("8SC");
         case opencv_core.CV_16U -> typeString.append("16UC");
         case opencv_core.CV_16S -> typeString.append("16SC");
         case opencv_core.CV_32S -> typeString.append("32SC");
         case opencv_core.CV_32F -> typeString.append("32FC");
         case opencv_core.CV_64F -> typeString.append("64FC");
      }

      typeString.append(channels);

      return typeString.toString();
   }

   // TODO: Support non-rectified images and stereo images
   public static void packCameraInfo(RawImage image, String cameraFrameId, CameraInfo cameraInfoToPack)
   {
      // Set the header
      Instant imageAcquisitionTime = image.getAcquisitionTime();
      cameraInfoToPack.getHeader().getStamp().setSec((int) imageAcquisitionTime.getEpochSecond());
      cameraInfoToPack.getHeader().getStamp().setNanosec(imageAcquisitionTime.getNano());
      cameraInfoToPack.getHeader().setFrameId(cameraFrameId);

      // Set the calibration parameters
      // Image dimensions
      cameraInfoToPack.setHeight(image.getHeight());
      cameraInfoToPack.setWidth(image.getWidth());

      // Distortion model
      cameraInfoToPack.setDistortionModel("plumb_bob");
      cameraInfoToPack.getD().clear(5);
      cameraInfoToPack.getD().add(new double[]{0.0, 0.0, 0.0, 0.0, 0.0}); // We (mostly) work with rectified images, so assume no distortion

      /*
       * Set the intrinsics matrix
       *      [fx  0 cx]
       *  K = [ 0 fy cy]
       *      [ 0  0  1]
       */
      cameraInfoToPack.getK()[0] = image.getFocalLengthX();
      cameraInfoToPack.getK()[1] = 0.0;
      cameraInfoToPack.getK()[2] = image.getPrincipalPointX();
      cameraInfoToPack.getK()[3] = 0.0;
      cameraInfoToPack.getK()[4] = image.getFocalLengthY();
      cameraInfoToPack.getK()[5] = image.getPrincipalPointY();
      cameraInfoToPack.getK()[6] = 0.0;
      cameraInfoToPack.getK()[7] = 0.0;
      cameraInfoToPack.getK()[8] = 1.0;

      // Set the rotation matrix (only used for stereo images, so we assume identity)
      cameraInfoToPack.getR()[0] = 1.0;
      cameraInfoToPack.getR()[1] = 0.0;
      cameraInfoToPack.getR()[2] = 0.0;
      cameraInfoToPack.getR()[3] = 0.0;
      cameraInfoToPack.getR()[4] = 1.0;
      cameraInfoToPack.getR()[5] = 0.0;
      cameraInfoToPack.getR()[6] = 0.0;
      cameraInfoToPack.getR()[7] = 0.0;
      cameraInfoToPack.getR()[8] = 1.0;

      /*
       * Set the projection matrix
       *     [fx'  0  cx' Tx]
       * P = [ 0  fy' cy' Ty]
       *     [ 0   0   1   0]
       * Since we're not using stereo images, Tx = Ty = 0
       * We also assume fx` = fx, cx` = cx, etc.
       */
      cameraInfoToPack.getP()[0] = image.getFocalLengthX();
      cameraInfoToPack.getP()[1] = 0.0;
      cameraInfoToPack.getP()[2] = image.getPrincipalPointX();
      cameraInfoToPack.getP()[3] = 0.0;
      cameraInfoToPack.getP()[4] = 0.0;
      cameraInfoToPack.getP()[5] = image.getFocalLengthY();
      cameraInfoToPack.getP()[6] = image.getPrincipalPointY();
      cameraInfoToPack.getP()[7] = 0.0;
      cameraInfoToPack.getP()[8] = 0.0;
      cameraInfoToPack.getP()[9] = 0.0;
      cameraInfoToPack.getP()[10] = 1.0;
      cameraInfoToPack.getP()[11] = 0.0;

      // Set "Operational Parameters"
      // Assume no binning
      cameraInfoToPack.setBinningX(0);
      cameraInfoToPack.setBinningY(0);

      // Set the ROI to the full image
      cameraInfoToPack.getRoi().setXOffset(0);
      cameraInfoToPack.getRoi().setYOffset(0);
      cameraInfoToPack.getRoi().setHeight(0);
      cameraInfoToPack.getRoi().setWidth(0);
      cameraInfoToPack.getRoi().setDoRectify(false);
   }

   public static void copyToFloatPointer(IDLSequence.Float sourceIDLSequence, FloatPointer floatPointerToPack, int startIndex)
   {
      for (int i = 0; i < sourceIDLSequence.size(); i++)
      {
         floatPointerToPack.put(i + startIndex, sourceIDLSequence.get(i));
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

   public static void convertToHeightMapData(Mat heightMapPointer,
                                             HeightMapData heightMapDataToPack,
                                             Point3D gridCenter,
                                             float widthInMeters,
                                             float cellSizeInMeters,
                                             HeightMapParameters heightMapParameters)
   {
      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      heightMapDataToPack.setGridCenter(gridCenter.getX(), gridCenter.getY());

      // Read data into byte[]
      byte[] data = new byte[Short.BYTES * totalCells];
      heightMapPointer.data().get(data);

      // Put height values into HeightMapData object
      for (int i = 0; i < totalCells; ++i)
      {
         // Get the start index of the bytes for a short
         int dataIndex = Short.BYTES * i;

         // Get the most and least significant bits, combine into integer
         int major = (data[dataIndex + 1] << 8) & 0xFF00;
         int minor = data[dataIndex] & 0x00FF;
         int height = major | minor;

         // Calculate cell height
         float cellHeight = (float) (((float) height / heightMapParameters.getHeightScaleFactor()) - heightMapParameters.getHeightOffset());

         // Put it into the HeightMapData object
         int key = cellsPerAxis * (i % cellsPerAxis) + (i / cellsPerAxis);
         heightMapDataToPack.setHeightAt(key, cellHeight);
      }
   }

   public static Mat convertHeightMapDataToMat(HeightMapData heightMapData, HeightMapParameters heightMapParameters)
   {
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      int cellsPerAxis = 2 * centerIndex + 1;

      // Create a new Mat object to hold the height map data
      Mat heightMapMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);

      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
            double cellHeight = heightMapData.getHeightAt(key);

            // Reverse the height calculation to get the raw height value
            int height = (int) ((cellHeight + (float) heightMapParameters.getHeightOffset()) * heightMapParameters.getHeightScaleFactor());

            // Store the height value in the Mat object
            heightMapMat.ptr(xIndex, yIndex).putShort((short) height);
         }
      }

      return heightMapMat;
   }

   public static void convertToHeightMapImage(ImageMessage imageMessage, Mat heightMapImageToPack)
   {
      int numberOfBytes = imageMessage.getData().size();

      // Create a pointer to the compressed data
      BytePointer compressedDataPointer = new BytePointer(imageMessage.getData().getBuffer().position(0));
      compressedDataPointer.limit(numberOfBytes);

      // Wrap the pointer in a Mat (for the imdecode function)
      Mat compressedDataMat = new Mat(1, numberOfBytes, opencv_core.CV_8UC1, compressedDataPointer);

      // Decompress the height map image
      opencv_imgcodecs.imdecode(compressedDataMat, opencv_imgcodecs.IMREAD_UNCHANGED, heightMapImageToPack);

      // Close pointers
      compressedDataMat.close();
      compressedDataPointer.close();
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
