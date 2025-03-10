package us.ihmc.perception;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.SRTStreamStatus;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.streaming.ROS2SRTSensorStreamer;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.time.Instant;
import java.util.Arrays;

import static us.ihmc.perception.imageMessage.CompressionType.NVJPEG;
import static us.ihmc.perception.imageMessage.CompressionType.ZSTD_NVJPEG_HYBRID;

public class RawImagePublisher implements AutoCloseable
{
   private final CUDACompressionTools compressionTools;
   private final CUDAJPEGProcessor jpegProcessor;
   private final ROS2SRTSensorStreamer sensorStreamer;

   private final ROS2Helper ros2Helper;
   private final ImageMessage imageMessage;
   private final Image ros2Image;

   private boolean destroyed = false;

   public RawImagePublisher(ROS2Node ros2Node)
   {
      compressionTools = new CUDACompressionTools();
      jpegProcessor = new CUDAJPEGProcessor();
      sensorStreamer = new ROS2SRTSensorStreamer(ros2Node);

      ros2Helper = new ROS2Helper(ros2Node);
      imageMessage = new ImageMessage();
      ros2Image = new Image();
   }

   @SuppressWarnings("unchecked") // Trust me bro, I know what I'm doing
   public synchronized void publishImage(ROS2Topic<? extends Packet<?>> imageTopic, RawImage imageToPublish)
   {
      if (destroyed)
         return;

      if (imageTopic.getType().equals(ImageMessage.class))
      {  // Topic is an ImageMessage topic -> publish as image message
         publishAsImageMessage((ROS2Topic<ImageMessage>) imageTopic, imageToPublish);
      }
      else if (imageTopic.getType().equals(Image.class))
      {  // Topic is a standard ROS2 Image topic -> publish as standard ROS2 Image message
         publishAsROS2Image((ROS2Topic<Image>) imageTopic, imageToPublish);
      }
      else if (imageTopic.getType().equals(CameraInfo.class))
      {  // Topic is a camera info topic -> publish the image's camera info
         publishCameraInfo((ROS2Topic<CameraInfo>) imageTopic, imageToPublish);
      }
      else if (imageTopic.getType().equals(SRTStreamStatus.class))
      {  // Topic is an SRT stream topic -> stream video over SRT
         sensorStreamer.sendFrame((ROS2Topic<SRTStreamStatus>) imageTopic, imageToPublish);
      }
   }

   private void publishAsImageMessage(ROS2Topic<ImageMessage> imageTopic, RawImage imageToPublish)
   {
      GpuMat imageToCompress = imageToPublish.getGpuImageMat();
      BytePointer compressedImage;
      CompressionType compressionType;

      switch (imageToPublish.getPixelFormat())
      {
         case GRAY16: // Depth image -> compress using ZSTD nvJPEG hybrid compression
            compressedImage = compressionTools.compressDepth(imageToCompress);
            compressionType = ZSTD_NVJPEG_HYBRID;
            break;
         case BGRA8: // BGRA image -> convert to BGR, then compress using nvJPEG
            GpuMat bgr8Image = new GpuMat();
            opencv_cudaimgproc.cvtColor(imageToCompress, bgr8Image, opencv_imgproc.COLOR_BGRA2BGR);
            imageToCompress = bgr8Image;
         case BGR8: // BGR image -> compress using nvJPEG
            compressedImage = new BytePointer(OpenCVTools.dataSize(imageToCompress));
            jpegProcessor.encodeBGR(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         case RGBA8: // RGBA image -> convert to RGB, then compress using nvJPEG
            GpuMat rgb8Image = new GpuMat();
            opencv_cudaimgproc.cvtColor(imageToCompress, rgb8Image, opencv_imgproc.COLOR_RGBA2RGB);
            imageToCompress = rgb8Image;
         case RGB8: // RGB image -> compress using nvJPEG
            compressedImage = new BytePointer(OpenCVTools.dataSize(imageToCompress));
            jpegProcessor.encodeRGB(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         case GRAY8: // Black and white image -> compress using nvJPEG
            compressedImage = new BytePointer(OpenCVTools.dataSize(imageToCompress));
            jpegProcessor.encodeGray(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         default:
            throw new NotImplementedException("Tomasz has not implemented the compression method for this pixel format yet.");
      }

      // Pack the message and send it off
      PerceptionMessageTools.packImageMessage(imageToPublish, compressedImage, compressionType, imageMessage);
      ros2Helper.publish(imageTopic, imageMessage);
   }

   private void publishAsROS2Image(ROS2Topic<Image> imageTopic, RawImage imageToPublish)
   {
      // Set the header
      Instant imageAcquisitionTime = imageToPublish.getAcquisitionTime();
      ros2Image.getHeader().getStamp().setSec((int) imageAcquisitionTime.getEpochSecond());
      ros2Image.getHeader().getStamp().setNanosec(imageAcquisitionTime.getNano());
      ros2Image.getHeader().setFrameId("world"); // TODO: Figure out how to do frame ids with RawImage

      // Set dimensions
      ros2Image.setWidth(imageToPublish.getWidth());
      ros2Image.setHeight(imageToPublish.getHeight());

      // Set encoding
      PixelFormat pixelFormat = imageToPublish.getPixelFormat();
      String encoding = switch (pixelFormat)
      {
         case BGR8, BGRA8, RGB8, RGBA8 -> pixelFormat.name().toLowerCase();
         case GRAY8, GRAY16 -> pixelFormat.name().toLowerCase().replace("gray", "mono");
         default -> getOpenCVTypeString(imageToPublish.getOpenCVType());
      };
      ros2Image.setEncoding(encoding);

      // Get the message's internal buffer
      ByteBuffer dataBuffer = ros2Image.getData().getBuffer();

      // Set byte order
      ros2Image.setIsBigendian((byte) (dataBuffer.order().equals(ByteOrder.BIG_ENDIAN) ? 1 : 0));

      // Set step
      Mat cpuImage = imageToPublish.getCpuImageMat();
      ros2Image.setStep(cpuImage.step());

      // Set data
      int memorySize = (int) OpenCVTools.memorySize(cpuImage);
      dataBuffer.position(0).put(cpuImage.data().limit(memorySize).asByteBuffer());

      // Publish the image
      ros2Helper.publish(imageTopic, ros2Image);
   }

   private String getOpenCVTypeString(int openCVType)
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
   private void publishCameraInfo(ROS2Topic<CameraInfo> cameraInfoTopic, RawImage image)
   {
      CameraInfo cameraInfo = new CameraInfo();

      // Set the header
      Instant imageAcquisitionTime = image.getAcquisitionTime();
      cameraInfo.getHeader().getStamp().setSec((int) imageAcquisitionTime.getEpochSecond());
      cameraInfo.getHeader().getStamp().setNanosec(imageAcquisitionTime.getNano());
      cameraInfo.getHeader().setFrameId("world"); // TODO: should be camera frame

      // Set the calibration parameters
      // Image dimensions
      cameraInfo.setHeight(image.getHeight());
      cameraInfo.setWidth(image.getWidth());

      // Distortion model
      cameraInfo.setDistortionModel("plumb_bob");
      cameraInfo.getD().clear(5);
      cameraInfo.getD().add(new double[]{0.0, 0.0, 0.0, 0.0, 0.0}); // We (mostly) work with rectified images, so assume no distortion

      /*
       * Set the intrinsics matrix
       *      [fx  0 cx]
       *  K = [ 0 fy cy]
       *      [ 0  0  1]
       */
      cameraInfo.getK()[0] = image.getFocalLengthX();
      cameraInfo.getK()[1] = 0.0;
      cameraInfo.getK()[2] = image.getPrincipalPointX();
      cameraInfo.getK()[3] = 0.0;
      cameraInfo.getK()[4] = image.getFocalLengthY();
      cameraInfo.getK()[5] = image.getPrincipalPointY();
      cameraInfo.getK()[6] = 0.0;
      cameraInfo.getK()[7] = 0.0;
      cameraInfo.getK()[8] = 1.0;

      // Set the rotation matrix (only used for stereo images, so we assume identity)
      cameraInfo.getR()[0] = 1.0;
      cameraInfo.getR()[1] = 0.0;
      cameraInfo.getR()[2] = 0.0;
      cameraInfo.getR()[3] = 0.0;
      cameraInfo.getR()[4] = 1.0;
      cameraInfo.getR()[5] = 0.0;
      cameraInfo.getR()[6] = 0.0;
      cameraInfo.getR()[7] = 0.0;
      cameraInfo.getR()[8] = 1.0;

      /*
       * Set the projection matrix
       *     [fx'  0  cx' Tx]
       * P = [ 0  fy' cy' Ty]
       *     [ 0   0   1   0]
       * Since we're not using stereo images, Tx = Ty = 0
       * We also assume fx` = fx, cx` = cx, etc.
       */
      cameraInfo.getP()[0] = image.getFocalLengthX();
      cameraInfo.getP()[1] = 0.0;
      cameraInfo.getP()[2] = image.getPrincipalPointX();
      cameraInfo.getP()[3] = 0.0;
      cameraInfo.getP()[4] = 0.0;
      cameraInfo.getP()[5] = image.getFocalLengthY();
      cameraInfo.getP()[6] = image.getPrincipalPointY();
      cameraInfo.getP()[7] = 0.0;
      cameraInfo.getP()[8] = 0.0;
      cameraInfo.getP()[9] = 0.0;
      cameraInfo.getP()[10] = 1.0;
      cameraInfo.getP()[11] = 0.0;

      // Set "Operational Parameters"
      // Assume no binning
      cameraInfo.setBinningX(0);
      cameraInfo.setBinningY(0);

      // Set the ROI to the full image
      cameraInfo.getRoi().setXOffset(0);
      cameraInfo.getRoi().setYOffset(0);
      cameraInfo.getRoi().setHeight(0);
      cameraInfo.getRoi().setWidth(0);
      cameraInfo.getRoi().setDoRectify(false);

      // Publish the message
      ros2Helper.publish(cameraInfoTopic, cameraInfo);
   }

   @Override
   public synchronized void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());
      compressionTools.destroy();
      jpegProcessor.destroy();
      sensorStreamer.destroy();
      destroyed = true;
      System.out.println("Closed " + getClass().getSimpleName());
   }
}
