package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLIntBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.perception.opencl.OpenCLFloatParameters;

import java.nio.ByteBuffer;

/**
 * Extracts the ranges (depth) from the incoming Ouster data buffer
 * and provides an OpenCV CV_16UC1 Mat using an OpenCL kernel.
 */
public class OusterDepthExtractionKernel
{
   private final NettyOuster nettyOuster;
   private final BytePointer lidarFrameByteBufferPointer;
   private final ByteBuffer lidarFrameByteBufferCopy;
   private final BytePointer lidarFrameByteBufferPointerCopy;

   private final OpenCLManager openCLManager;
   private final _cl_program depthImageExtractionProgram;
   private final _cl_kernel extractDepthImageKernel;
   private final _cl_mem lidarFrameBufferObject;
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private final BytedecoImage extractedDepthImage;
   private final OpenCLIntBuffer pixelShiftOpenCLBuffer;

   public OusterDepthExtractionKernel(NettyOuster nettyOuster, OpenCLManager openCLManager)
   {
      this.nettyOuster = nettyOuster;
      this.openCLManager = openCLManager;

      lidarFrameByteBufferCopy = ByteBuffer.allocateDirect(nettyOuster.getLidarFrameByteBuffer().limit());
      lidarFrameByteBufferPointerCopy = new BytePointer(lidarFrameByteBufferCopy);
      lidarFrameByteBufferPointer = new BytePointer(nettyOuster.getLidarFrameByteBuffer());

      extractedDepthImage = new BytedecoImage(nettyOuster.getImageWidth(), nettyOuster.getImageHeight(), opencv_core.CV_16UC1);
      depthImageExtractionProgram = openCLManager.loadProgram("OusterDepthImagePublisher");
      extractDepthImageKernel = openCLManager.createKernel(depthImageExtractionProgram, "extractDepthImage");
      lidarFrameBufferObject = openCLManager.createBufferObject(lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);
      extractedDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      pixelShiftOpenCLBuffer = new OpenCLIntBuffer(nettyOuster.getPixelShiftBuffer());
      pixelShiftOpenCLBuffer.createOpenCLBufferObject(openCLManager);
   }

   public void copyLidarFrameBuffer()
   {
      lidarFrameByteBufferPointer.position(0);
      lidarFrameByteBufferPointerCopy.position(0);
      lidarFrameByteBufferPointerCopy.put(lidarFrameByteBufferPointer);
   }

   public void runKernel()
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
      openCLManager.finish();
   }

   public BytedecoImage getExtractedDepthImage()
   {
      return extractedDepthImage;
   }
}
