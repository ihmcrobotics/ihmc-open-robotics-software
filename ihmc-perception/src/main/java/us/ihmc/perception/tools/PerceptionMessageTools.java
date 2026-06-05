package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.FramePlanarRegionsListMessage;
import perception_msgs.ImageMessage;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

import javax.annotation.Nullable;
import java.nio.ByteBuffer;
import java.nio.ShortBuffer;
import java.time.Instant;

public class PerceptionMessageTools
{
   public static void packCameraIntrinsics(CameraIntrinsics cameraIntrinsics, ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setFocalLengthXPixels((float) cameraIntrinsics.getFx());
      imageMessageToPack.setFocalLengthYPixels((float) cameraIntrinsics.getFy());
      imageMessageToPack.setPrincipalPointXPixels((float) cameraIntrinsics.getCx());
      imageMessageToPack.setPrincipalPointYPixels((float) cameraIntrinsics.getCy());
      imageMessageToPack.setImageWidth((short) cameraIntrinsics.getWidth());
      imageMessageToPack.setImageHeight((short) cameraIntrinsics.getHeight());
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
      packDataArray(imageMessage.getData(), dataBuffer);
   }

   public static void packDataArray(IDLByteSequence dataToPack, ByteBuffer dataBuffer)
   {
      ByteBuffer source = dataBuffer.duplicate();
      int byteCount = source.remaining();

      dataToPack.clear();
      if (byteCount <= 0)
         return;

      dataToPack.ensureMinCapacity(byteCount);
      dataToPack.getBuffer().put(source);
   }

   public static void packDataArray(IDLByteSequence dataToPack, BytePointer dataPointer)
   {
      int start = (int) dataPointer.position();
      int end = (int) dataPointer.limit();
      int byteCount = end - start;

      dataToPack.clear();
      if (byteCount <= 0)
         return;

      dataToPack.ensureMinCapacity(byteCount);

      ByteBuffer source = dataPointer.asBuffer().duplicate();
      source.position(start);
      source.limit(end);
      dataToPack.getBuffer().put(source);
   }

   public static void packImageMessageData(ImageMessage imageMessage, Mat mat)
   {
      packDataArray(imageMessage.getData(), mat);
   }

   public static void packDataArray(IDLByteSequence dataToPack, Mat mat)
   {
      long size = mat.step() * mat.rows();
      packDataArray(dataToPack, mat.data().limit(size).asBuffer());
   }

   public static void packShortDataArray(IDLByteSequence dataToPack, Mat mat)
   {
      if (mat.type() != opencv_core.CV_16UC1)
         throw new IllegalArgumentException("Expected CV_16UC1 Mat");

      // Note: Due to how the backing native memory layout is, ensure we get the total Buffer
      // mat.asByteBuffer() doesn't always return the entire buffer, also the data may be wrong
      int totalBytes = mat.rows() * mat.cols() * 2; // 2 bytes per short
      ByteBuffer matBuffer = mat.data().limit(totalBytes).asByteBuffer();
      ShortBuffer shortBuffer = matBuffer.asShortBuffer();

      while (shortBuffer.hasRemaining())
      {
         int ushort = shortBuffer.get() & 0xFFFF; // Mask to make sure it's unsigned
         byte low = (byte) (ushort & 0xFF);      // Lower 8 bits
         byte high = (byte) ((ushort >> 8) & 0xFF); // Upper 8 bits

         // Pack in little-endian format (low byte first)
         dataToPack.add(low);
         dataToPack.add(high);
      }
   }

   public static void packImageMessageData(ImageMessage imageMessage, BytePointer dataPointer)
   {
      packDataArray(imageMessage.getData(), dataPointer);
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
      messageToPack.setSequenceNumber((int) image.getSequenceNumber());
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
      imageMessage.setImageHeight((short) height);
      imageMessage.setImageWidth((short) width);
      imageMessage.getPosition().set(cameraPose.getPosition());
      imageMessage.getOrientation().set(cameraPose.getOrientation());
      imageMessage.setSequenceNumber((int) sequenceNumber);
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
      else
         imageMessageToPack.getOusterBeamAltitudeAngles().clear();
      if (ousterBeamAzimuthAngles != null)
         MessageTools.packIDLSequence(ousterBeamAzimuthAngles, imageMessageToPack.getOusterBeamAzimuthAngles());
      else
         imageMessageToPack.getOusterBeamAzimuthAngles().clear();
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

      // Set byte order
      messageToPack.setIsBigendian((byte) 0);

      // Set step
      Mat cpuImage = image.getCpuImageMat();
      messageToPack.setStep((int) cpuImage.step());

      // Set data
      packDataArray(messageToPack.getData(), cpuImage);
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

   public static void packCameraInfo(RawImage image, String cameraFrameId, CameraInfo cameraInfoToPack)
   {
      packCameraInfo(image.getAcquisitionTime(), image.getIntrinsicsCopy(), image.getTransformToWorld(), cameraFrameId, cameraInfoToPack);
   }

   // TODO: Support non-rectified images and stereo images
   public static void packCameraInfo(Instant imageAcquisitionTime,
                                     CameraIntrinsics intrinsics,
                                     RigidBodyTransformReadOnly transformToWorld,
                                     String cameraFrameId,
                                     CameraInfo cameraInfoToPack)
   {
      // Set the header
      cameraInfoToPack.getHeader().getStamp().setSec((int) imageAcquisitionTime.getEpochSecond());
      cameraInfoToPack.getHeader().getStamp().setNanosec(imageAcquisitionTime.getNano());
      cameraInfoToPack.getHeader().setFrameId(cameraFrameId);

      // Set the calibration parameters
      // Image dimensions
      cameraInfoToPack.setHeight(intrinsics.getHeight());
      cameraInfoToPack.setWidth(intrinsics.getWidth());

      // Distortion model
      cameraInfoToPack.setDistortionModel("plumb_bob");
      cameraInfoToPack.getD().clear();
      cameraInfoToPack.getD().addAll(new double[]{0.0, 0.0, 0.0, 0.0, 0.0}); // We (mostly) work with rectified images, so assume no distortion

      /*
       * Set the intrinsics matrix
       *      [fx  0 cx]
       *  K = [ 0 fy cy]
       *      [ 0  0  1]
       */
      cameraInfoToPack.getK()[0] = intrinsics.getFx();
      cameraInfoToPack.getK()[1] = 0.0;
      cameraInfoToPack.getK()[2] = intrinsics.getCx();
      cameraInfoToPack.getK()[3] = 0.0;
      cameraInfoToPack.getK()[4] = intrinsics.getFy();
      cameraInfoToPack.getK()[5] = intrinsics.getCy();
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
      cameraInfoToPack.getP()[0] = intrinsics.getFx();
      cameraInfoToPack.getP()[1] = 0.0;
      cameraInfoToPack.getP()[2] = intrinsics.getCx();
      cameraInfoToPack.getP()[3] = 0.0;
      cameraInfoToPack.getP()[4] = 0.0;
      cameraInfoToPack.getP()[5] = intrinsics.getFy();
      cameraInfoToPack.getP()[6] = intrinsics.getCy();
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

   public static void copyToFloatPointer(IDLFloatSequence sourceIDLSequence, FloatPointer floatPointerToPack, int startIndex)
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
}
