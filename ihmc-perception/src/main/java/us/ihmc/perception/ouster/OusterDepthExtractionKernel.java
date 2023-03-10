package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLIntBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.tools.NativeMemoryTools;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.function.Supplier;

/**
 * Extracts the ranges (depth) from the incoming Ouster data buffer
 * and provides an OpenCV CV_16UC1 Mat using an OpenCL kernel.
 *
 * FIXME: We need to be able to resize the buffers of this kernel without reloading the program.
 */
public class OusterDepthExtractionKernel
{
   private static final double horizontalFieldOfView = Math.PI * 2.0;
   private static final double verticalFieldOfView = Math.PI / 2.0;

   private final NettyOuster nettyOuster;
   private final BytePointer lidarFrameByteBufferPointer;
   private final ByteBuffer lidarFrameByteBufferCopy;
   private final Supplier<Boolean> computeHeightMap;
   private final BytePointer lidarFrameByteBufferPointerCopy;
   private final Supplier<Boolean> computeLidarScan;

   private final OpenCLManager openCLManager;
   private final _cl_program depthImageExtractionProgram;
   private final _cl_kernel extractDepthImageKernel;
   private final _cl_kernel imageToPointCloudKernel;
   private final _cl_mem lidarFrameBufferObject;
   private final OpenCLFloatParameters depthImageParametersBuffer = new OpenCLFloatParameters();
   private final BytedecoImage extractedDepthImage;
   private final OpenCLIntBuffer pixelShiftOpenCLBuffer;
   private final OpenCLFloatParameters pointCloudParametersBuffer = new OpenCLFloatParameters();
   private final OpenCLFloatBuffer altitudeAnglesOpenCLBuffer;
   private final OpenCLFloatBuffer azimuthAnglesOpenCLBuffer;
   private final OpenCLFloatBuffer pointCloudXYZBuffer;

   public OusterDepthExtractionKernel(NettyOuster nettyOuster,
                                      OpenCLManager openCLManager)
   {
      this(nettyOuster, openCLManager, () -> true, () -> true);
   }

   public OusterDepthExtractionKernel(NettyOuster nettyOuster,
                                      OpenCLManager openCLManager,
                                      Supplier<Boolean> computeLidarScan,
                                      Supplier<Boolean> computeHeightMap)
   {
      this.nettyOuster = nettyOuster;
      this.openCLManager = openCLManager;
      this.computeLidarScan = computeLidarScan;
      this.computeHeightMap = computeHeightMap;

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
      altitudeAnglesOpenCLBuffer = new OpenCLFloatBuffer(nettyOuster.getBeamAltitudeAnglesBuffer());
      altitudeAnglesOpenCLBuffer.createOpenCLBufferObject(openCLManager);
      azimuthAnglesOpenCLBuffer = new OpenCLFloatBuffer(nettyOuster.getBeamAzimuthAnglesBuffer());
      azimuthAnglesOpenCLBuffer.createOpenCLBufferObject(openCLManager);

      pointCloudXYZBuffer = new OpenCLFloatBuffer(3 * nettyOuster.getImageHeight() * nettyOuster.getImageWidth());
      pointCloudXYZBuffer.createOpenCLBufferObject(openCLManager);
   }

   public void copyLidarFrameBuffer()
   {
      lidarFrameByteBufferPointer.position(0);
      lidarFrameByteBufferPointerCopy.position(0);
      NativeMemoryTools.copy(lidarFrameByteBufferPointer, lidarFrameByteBufferPointerCopy);
   }

   public void runKernel(RigidBodyTransformReadOnly cameraPose)
   {
      depthImageParametersBuffer.setParameter(nettyOuster.getColumnsPerFrame());
      depthImageParametersBuffer.setParameter(nettyOuster.getMeasurementBlockSize());
      depthImageParametersBuffer.setParameter(NettyOuster.HEADER_BLOCK_BYTES);
      depthImageParametersBuffer.setParameter(NettyOuster.CHANNEL_DATA_BLOCK_BYTES);

      depthImageParametersBuffer.writeOpenCLBufferObject(openCLManager);
      pixelShiftOpenCLBuffer.writeOpenCLBufferObject(openCLManager);
      openCLManager.enqueueWriteBuffer(lidarFrameBufferObject, lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);

      openCLManager.setKernelArgument(extractDepthImageKernel, 0, depthImageParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(extractDepthImageKernel, 1, pixelShiftOpenCLBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(extractDepthImageKernel, 2, lidarFrameBufferObject);
      openCLManager.setKernelArgument(extractDepthImageKernel, 3, extractedDepthImage.getOpenCLImageObject());
      openCLManager.execute2D(extractDepthImageKernel, nettyOuster.getImageWidth(), nettyOuster.getImageHeight());

      extractedDepthImage.readOpenCLImage(openCLManager);

      if (computeLidarScan.get() || computeHeightMap.get())
      {
         pointCloudParametersBuffer.setParameter((float) horizontalFieldOfView);
         pointCloudParametersBuffer.setParameter((float) verticalFieldOfView);
         pointCloudParametersBuffer.setParameter(nettyOuster.getImageWidth());
         pointCloudParametersBuffer.setParameter(nettyOuster.getImageHeight());
         pointCloudParametersBuffer.setParameter(NettyOuster.DISCRETE_RESOLUTION);
         pointCloudParametersBuffer.setParameter(nettyOuster.getLidarOriginToBeamOrigin());
         pointCloudParametersBuffer.writeOpenCLBufferObject(openCLManager);

         openCLManager.setKernelArgument(imageToPointCloudKernel, 0, pointCloudParametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(imageToPointCloudKernel, 1, extractedDepthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(imageToPointCloudKernel, 2, pointCloudXYZBuffer.getOpenCLBufferObject());
         openCLManager.execute2D(imageToPointCloudKernel, nettyOuster.getImageWidth(), nettyOuster.getImageHeight());

         pointCloudXYZBuffer.readOpenCLBufferObject(openCLManager);
      }
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
