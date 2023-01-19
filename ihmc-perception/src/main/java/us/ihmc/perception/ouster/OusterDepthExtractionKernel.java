package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLIntBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.memory.NativeMemoryTools;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.perception.opencl.OpenCLFloatParameters;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Extracts the ranges (depth) from the incoming Ouster data buffer
 * and provides an OpenCV CV_16UC1 Mat using an OpenCL kernel.
 */
public class OusterDepthExtractionKernel
{
   private static final double horizontalFieldOfView = Math.PI * 2.0;
   private static final double verticalFieldOfView = Math.PI / 2.0;

   private final NettyOuster nettyOuster;
   private final BytePointer lidarFrameByteBufferPointer;
   private final ByteBuffer lidarFrameByteBufferCopy;
   private final BytePointer lidarFrameByteBufferPointerCopy;

   private final OpenCLManager openCLManager;
   private final _cl_program depthImageExtractionProgram;
   private final _cl_kernel extractDepthImageKernel;
   private _cl_kernel imageToPointCloudKernel;
   private final _cl_mem lidarFrameBufferObject;
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private final OpenCLFloatParameters sensorValuesBuffer = new OpenCLFloatParameters();

   private final BytedecoImage extractedDepthImage;
   private final OpenCLIntBuffer pixelShiftOpenCLBuffer;
   private final OpenCLFloatBuffer pointCloudXYZBuffer;

   private final List<Class<?>> outputTopicsTypes;

   public OusterDepthExtractionKernel(NettyOuster nettyOuster,
                                      OpenCLManager openCLManager)
   {
      this(nettyOuster, openCLManager, Arrays.asList(ImageMessage.class));
   }

   public OusterDepthExtractionKernel(NettyOuster nettyOuster,
                                      OpenCLManager openCLManager,
                                      List<Class<?>> outputTopicsTypes)
   {
      this.nettyOuster = nettyOuster;
      this.openCLManager = openCLManager;
      this.outputTopicsTypes = outputTopicsTypes;

      lidarFrameByteBufferCopy = ByteBuffer.allocateDirect(nettyOuster.getLidarFrameByteBuffer().limit());
      lidarFrameByteBufferPointerCopy = new BytePointer(lidarFrameByteBufferCopy);
      lidarFrameByteBufferPointer = new BytePointer(nettyOuster.getLidarFrameByteBuffer());

      extractedDepthImage = new BytedecoImage(nettyOuster.getImageWidth(), nettyOuster.getImageHeight(), opencv_core.CV_16UC1);
      depthImageExtractionProgram = openCLManager.loadProgram("OusterDepthImageExtraction");
      extractDepthImageKernel = openCLManager.createKernel(depthImageExtractionProgram, "extractDepthImage");
      imageToPointCloudKernel = openCLManager.createKernel(depthImageExtractionProgram, "imageToPointCloud");

      lidarFrameBufferObject = openCLManager.createBufferObject(lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);
      extractedDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      pixelShiftOpenCLBuffer = new OpenCLIntBuffer(nettyOuster.getPixelShiftBuffer());
      pixelShiftOpenCLBuffer.createOpenCLBufferObject(openCLManager);

      pointCloudXYZBuffer = new OpenCLFloatBuffer(3 * nettyOuster.getImageHeight() * nettyOuster.getImageWidth());
      pointCloudXYZBuffer.createOpenCLBufferObject(openCLManager);
   }

   public void copyLidarFrameBuffer()
   {
      lidarFrameByteBufferPointer.position(0);
      lidarFrameByteBufferPointerCopy.position(0);
      NativeMemoryTools.copy(lidarFrameByteBufferPointer, lidarFrameByteBufferPointerCopy);
   }

   private void populateSensorValuesBuffer(RigidBodyTransformReadOnly sensorTransformToWorld)
   {
      sensorValuesBuffer.setParameter((float) horizontalFieldOfView);
      sensorValuesBuffer.setParameter((float) verticalFieldOfView);
      sensorValuesBuffer.setParameter(sensorTransformToWorld.getTranslation().getX32());
      sensorValuesBuffer.setParameter(sensorTransformToWorld.getTranslation().getY32());
      sensorValuesBuffer.setParameter(sensorTransformToWorld.getTranslation().getZ32());
      RotationMatrix rotationMatrix = new RotationMatrix(sensorTransformToWorld.getRotation());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM00());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM01());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM02());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM10());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM11());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM12());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM20());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM21());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM22());
      sensorValuesBuffer.setParameter(nettyOuster.getImageWidth());
      sensorValuesBuffer.setParameter(nettyOuster.getImageHeight());
      sensorValuesBuffer.setParameter((float) LidarPointCloudCompression.POINT_RESOLUTION);

      sensorValuesBuffer.writeOpenCLBufferObject(openCLManager);
   }

   public void runKernel(RigidBodyTransformReadOnly cameraPose)
   {
      parametersBuffer.setParameter(nettyOuster.getColumnsPerFrame());
      parametersBuffer.setParameter(nettyOuster.getMeasurementBlockSize());
      parametersBuffer.setParameter(NettyOuster.HEADER_BLOCK_BYTES);
      parametersBuffer.setParameter(NettyOuster.CHANNEL_DATA_BLOCK_BYTES);
      parametersBuffer.setParameter(NettyOuster.MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM);

      parametersBuffer.writeOpenCLBufferObject(openCLManager);
      pixelShiftOpenCLBuffer.writeOpenCLBufferObject(openCLManager);
      openCLManager.enqueueWriteBuffer(lidarFrameBufferObject, lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);

      openCLManager.setKernelArgument(extractDepthImageKernel, 0, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(extractDepthImageKernel, 1, pixelShiftOpenCLBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(extractDepthImageKernel, 2, lidarFrameBufferObject);
      openCLManager.setKernelArgument(extractDepthImageKernel, 3, extractedDepthImage.getOpenCLImageObject());
      openCLManager.execute2D(extractDepthImageKernel, nettyOuster.getImageWidth(), nettyOuster.getImageHeight());

      extractedDepthImage.readOpenCLImage(openCLManager);

      if (outputTopicsTypes.contains(LidarScanMessage.class))
      {
         populateSensorValuesBuffer(cameraPose);
         openCLManager.setKernelArgument(imageToPointCloudKernel, 0, sensorValuesBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(imageToPointCloudKernel, 1, extractedDepthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(imageToPointCloudKernel, 2, pointCloudXYZBuffer.getOpenCLBufferObject());
         openCLManager.execute2D(imageToPointCloudKernel, nettyOuster.getImageWidth(), nettyOuster.getImageHeight());

         pointCloudXYZBuffer.readOpenCLBufferObject(openCLManager);
      }

      openCLManager.finish();
   }

   public BytedecoImage getExtractedDepthImage()
   {
      return extractedDepthImage;
   }

   public FloatBuffer getPointCloudInSensorFrame()
   {
      return pointCloudXYZBuffer.getBackingDirectFloatBuffer();
   }
}
