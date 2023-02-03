package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.IntrinsicParametersMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PlanarRegionsListWithPoseMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.ros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.time.Instant;

public class PerceptionMessageTools
{
   private int totalNumberOfPoints;
   private final OpenCLManager openCLManager;
   private final _cl_program openCLProgram;
   private final _cl_kernel unpackPointCloudKernel;

   private ByteBuffer decompressionInputBuffer;
   private BytePointer decompressionInputBytePointer;
   private Mat decompressionInputMat;
   private BytedecoImage decompressionOutputImage;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private final OpenCLFloatParameters parametersOpenCLFloatBuffer = new OpenCLFloatParameters();
   private int depthWidth;
   private int depthHeight;
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   public PerceptionMessageTools()
   {
      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("PerceptionMessageTools");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "imageToPointCloud");
   }

   public void destroy()
   {
      openCLProgram.close();
      unpackPointCloudKernel.close();

      decompressionOutputImage.destroy(openCLManager);
      pointCloudVertexBuffer.destroy(openCLManager);

      openCLProgram.close();
   }

   private void resizeDepthImageData(int depthHeight, int depthWidth)
   {
      if (this.depthHeight == depthHeight && this.depthWidth == depthWidth)
         return;

      this.depthWidth = depthWidth;
      this.depthHeight = depthHeight;
      totalNumberOfPoints = depthWidth * depthHeight;
      decompressionInputBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
      decompressionInputBytePointer = new BytePointer(decompressionInputBuffer);
      decompressionOutputImage.resize(depthWidth, depthHeight, openCLManager, null);
      pointCloudVertexBuffer.resize(totalNumberOfPoints * 3, openCLManager);
   }

   public Point3D[] unpackDepthImage(ImageMessage imageMessage, double verticalFieldOfView, double horizontalFieldOfView)
   {
      int numberOfBytes;

      if (decompressionInputBuffer == null)
      {
         depthWidth = imageMessage.getImageWidth();
         depthHeight = imageMessage.getImageHeight();
         totalNumberOfPoints = depthWidth * depthHeight;
         decompressionInputBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
         decompressionInputBytePointer = new BytePointer(decompressionInputBuffer);
         decompressionInputMat = new Mat(1, 1, opencv_core.CV_8UC1);
         decompressionOutputImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
         decompressionOutputImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         pointCloudVertexBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * 3);
         pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
         LogTools.info("Allocated new buffers. {} points.", totalNumberOfPoints);
      }
      else
      {
         resizeDepthImageData(imageMessage.getImageHeight(), imageMessage.getImageWidth());
      }

      numberOfBytes = imageMessage.getData().size();
      decompressionInputBuffer.rewind();
      decompressionInputBuffer.limit(totalNumberOfPoints * 2);
      for (int i = 0; i < numberOfBytes; i++)
      {
         decompressionInputBuffer.put(imageMessage.getData().get(i));
      }
      decompressionInputBuffer.flip();

      decompressionInputBytePointer.position(0);
      decompressionInputBytePointer.limit(numberOfBytes);

      decompressionInputMat.cols(numberOfBytes);
      decompressionInputMat.data(decompressionInputBytePointer);

      decompressionOutputImage.getBackingDirectByteBuffer().rewind();
      opencv_imgcodecs.imdecode(decompressionInputMat, opencv_imgcodecs.IMREAD_UNCHANGED, decompressionOutputImage.getBytedecoOpenCVMat());
      decompressionOutputImage.getBackingDirectByteBuffer().rewind();

      sensorTransformToWorld.getTranslation().set(imageMessage.getPosition());
      sensorTransformToWorld.getRotation().set(imageMessage.getOrientation());

      parametersOpenCLFloatBuffer.setParameter((float) horizontalFieldOfView);
      parametersOpenCLFloatBuffer.setParameter((float) verticalFieldOfView);
      parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getX32());
      parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getY32());
      parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getZ32());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM00());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM01());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM02());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM10());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM11());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM12());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM20());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM21());
      parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM22());
      parametersOpenCLFloatBuffer.setParameter(depthWidth);
      parametersOpenCLFloatBuffer.setParameter(depthHeight);

      parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
      decompressionOutputImage.writeOpenCLImage(openCLManager);
      pointCloudVertexBuffer.syncWithBackingBuffer();

      openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressionOutputImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(unpackPointCloudKernel, depthWidth, depthHeight);

      pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();

      Point3D[] pointCloud = new Point3D[totalNumberOfPoints];
      FloatBuffer pointElementBuffer = pointCloudVertexBuffer.getBackingDirectFloatBuffer();
      pointElementBuffer.rewind();
      for (int i = 0; i < totalNumberOfPoints; i++)
         pointCloud[i] = new Point3D(pointElementBuffer.get(), pointElementBuffer.get(), pointElementBuffer.get());

      return pointCloud;
   }

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
                                           ImageMessageFormat.DEPTH_PNG_R16);
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

   public static void publishPlanarRegionsListWithPose(PlanarRegionsListWithPose planarRegionsListWithPose,
                                                       ROS2Topic<PlanarRegionsListWithPoseMessage> topic,
                                                       ROS2Helper ros2Helper)
   {
      ros2Helper.publish(topic, PlanarRegionMessageConverter.convertToPlanarRegionsListWithPoseMessage(planarRegionsListWithPose));
   }

   public static void publishPlanarRegionsList(PlanarRegionsList planarRegionsList, ROS2Topic<PlanarRegionsListMessage> topic, ROS2Helper ros2Helper)
   {
      ros2Helper.publish(topic, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
   }
}
