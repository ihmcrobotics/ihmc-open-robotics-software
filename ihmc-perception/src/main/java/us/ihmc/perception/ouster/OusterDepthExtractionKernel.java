package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.*;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLRigidBodyTransformParameter;
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
   private final OusterNetServer ousterNetServer;
   private final BytePointer lidarFrameByteBufferPointer;
   private final ByteBuffer lidarFrameByteBufferCopy;
   private final Supplier<Boolean> computeHeightMap;
   private final BytePointer lidarFrameByteBufferPointerCopy;
   private final Supplier<Boolean> computeLidarScan;

   private final OpenCLManager openCLManager;
   private final _cl_program depthImageExtractionProgram;
   private final _cl_kernel extractDepthImageKernel;
   private final _cl_kernel computePointCloudKernel;
   private final _cl_mem lidarFrameBufferObject;
   private final OpenCLFloatParameters depthImageExtractionParametersBuffer = new OpenCLFloatParameters();
   private final BytedecoImage extractedDepthImage;
   private final OpenCLFloatParameters pointCloudComputationParametersBuffer = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter ousterToWorldTransformParameter = new OpenCLRigidBodyTransformParameter();
   private final OpenCLFloatBuffer altitudeAnglesOpenCLBuffer;
   private final OpenCLFloatBuffer azimuthAnglesOpenCLBuffer;
   private final OpenCLFloatBuffer pointCloudXYZBuffer;

   public OusterDepthExtractionKernel(OusterNetServer ousterNetServer,
                                      OpenCLManager openCLManager)
   {
      this(ousterNetServer, openCLManager, () -> true, () -> true);
   }

   public OusterDepthExtractionKernel(OusterNetServer ousterNetServer,
                                      OpenCLManager openCLManager,
                                      Supplier<Boolean> computeLidarScan,
                                      Supplier<Boolean> computeHeightMap)
   {
      this.ousterNetServer = ousterNetServer;
      this.openCLManager = openCLManager;
      this.computeLidarScan = computeLidarScan;
      this.computeHeightMap = computeHeightMap;

      lidarFrameByteBufferCopy = ByteBuffer.allocateDirect(ousterNetServer.getLidarFrameByteBuffer().limit());
      lidarFrameByteBufferPointerCopy = new BytePointer(lidarFrameByteBufferCopy);
      lidarFrameByteBufferPointer = new BytePointer(ousterNetServer.getLidarFrameByteBuffer());

      extractedDepthImage = new BytedecoImage(ousterNetServer.getImageWidth(), ousterNetServer.getImageHeight(), opencv_core.CV_16UC1);
      depthImageExtractionProgram = openCLManager.loadProgram("OusterDepthImageExtraction");
      extractDepthImageKernel = openCLManager.createKernel(depthImageExtractionProgram, "extractDepthImage");
      computePointCloudKernel = openCLManager.createKernel(depthImageExtractionProgram, "computePointCloud");

      lidarFrameBufferObject = openCLManager.createBufferObject(lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);
      extractedDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      altitudeAnglesOpenCLBuffer = new OpenCLFloatBuffer(ousterNetServer.getBeamAltitudeAnglesBuffer());
      altitudeAnglesOpenCLBuffer.createOpenCLBufferObject(openCLManager);
      azimuthAnglesOpenCLBuffer = new OpenCLFloatBuffer(ousterNetServer.getBeamAzimuthAnglesBuffer());
      azimuthAnglesOpenCLBuffer.createOpenCLBufferObject(openCLManager);

      pointCloudXYZBuffer = new OpenCLFloatBuffer(3 * ousterNetServer.getImageHeight() * ousterNetServer.getImageWidth());
      pointCloudXYZBuffer.createOpenCLBufferObject(openCLManager);
   }

   public void copyLidarFrameBuffer()
   {
      lidarFrameByteBufferPointer.position(0);
      lidarFrameByteBufferPointerCopy.position(0);
      NativeMemoryTools.copy(lidarFrameByteBufferPointer, lidarFrameByteBufferPointerCopy);
   }

   public void runKernel(RigidBodyTransform ousterToWorldTransform)
   {
      depthImageExtractionParametersBuffer.setParameter(ousterNetServer.getColumnsPerFrame());
      depthImageExtractionParametersBuffer.setParameter(ousterNetServer.getMeasurementBlockSize());
      depthImageExtractionParametersBuffer.setParameter(OusterNetServer.HEADER_BLOCK_BYTES);
      depthImageExtractionParametersBuffer.setParameter(OusterNetServer.CHANNEL_DATA_BLOCK_BYTES);

      depthImageExtractionParametersBuffer.writeOpenCLBufferObject(openCLManager);
      openCLManager.enqueueWriteBuffer(lidarFrameBufferObject, lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);

      openCLManager.setKernelArgument(extractDepthImageKernel, 0, depthImageExtractionParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(extractDepthImageKernel, 1, lidarFrameBufferObject);
      openCLManager.setKernelArgument(extractDepthImageKernel, 2, extractedDepthImage.getOpenCLImageObject());
      openCLManager.execute2D(extractDepthImageKernel, ousterNetServer.getImageWidth(), ousterNetServer.getImageHeight());

      extractedDepthImage.readOpenCLImage(openCLManager);

      if (computeLidarScan.get() || computeHeightMap.get())
      {
         pointCloudComputationParametersBuffer.setParameter(ousterNetServer.getImageWidth());
         pointCloudComputationParametersBuffer.setParameter(ousterNetServer.getImageHeight());
         pointCloudComputationParametersBuffer.setParameter(ousterNetServer.getLidarOriginToBeamOrigin());
         pointCloudComputationParametersBuffer.setParameter(OusterNetServer.DISCRETE_RESOLUTION);
         ousterToWorldTransformParameter.setParameter(ousterToWorldTransform);

         pointCloudComputationParametersBuffer.writeOpenCLBufferObject(openCLManager);
         ousterToWorldTransformParameter.writeOpenCLBufferObject(openCLManager);

         openCLManager.setKernelArgument(computePointCloudKernel, 0, pointCloudComputationParametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computePointCloudKernel, 1, altitudeAnglesOpenCLBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computePointCloudKernel, 2, azimuthAnglesOpenCLBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computePointCloudKernel, 3, ousterToWorldTransformParameter.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computePointCloudKernel, 4, extractedDepthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(computePointCloudKernel, 5, pointCloudXYZBuffer.getOpenCLBufferObject());
         openCLManager.execute2D(computePointCloudKernel, ousterNetServer.getImageWidth(), ousterNetServer.getImageHeight());

         pointCloudXYZBuffer.readOpenCLBufferObject(openCLManager);
      }
   }

   public BytedecoImage getExtractedDepthImage()
   {
      return extractedDepthImage;
   }

   public FloatBuffer getPointCloudInWorldFrame()
   {
      return pointCloudXYZBuffer.getBackingDirectFloatBuffer();
   }
}
