package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLRigidBodyTransformParameter;

public class RDXColoredPointCloudPinholePinholeKernel
{
   private final OpenCLManager openCLManager;
   private final _cl_program openCLProgram;
   private final _cl_kernel kernel;
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter depthToWorldTransformParameter = new OpenCLRigidBodyTransformParameter();
   private final OpenCLRigidBodyTransformParameter depthToColorTransformParameter = new OpenCLRigidBodyTransformParameter();

   public RDXColoredPointCloudPinholePinholeKernel(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
      openCLProgram = openCLManager.loadProgram("PinholePinholeColoredPointCloudVisualizer", "PerceptionCommon.cl");
      kernel = openCLManager.createKernel(openCLProgram, "createPointCloud");
   }

   public void runKernel(RDXROS2ColoredPointCloudVisualizerColorChannel colorChannel,
                         RDXROS2ColoredPointCloudVisualizerDepthChannel depthChannel,
                         boolean useSensorColor,
                         int gradientMode,
                         boolean useSinusoidalGradientPattern,
                         float pointSize,
                         OpenCLFloatBuffer pointCloudVertexBuffer)
   {
      parametersBuffer.setParameter(colorChannel.getFx());
      parametersBuffer.setParameter(colorChannel.getFy());
      parametersBuffer.setParameter(colorChannel.getCx());
      parametersBuffer.setParameter(colorChannel.getCy());
      parametersBuffer.setParameter(depthChannel.getFx());
      parametersBuffer.setParameter(depthChannel.getFy());
      parametersBuffer.setParameter(depthChannel.getCx());
      parametersBuffer.setParameter(depthChannel.getCy());
      parametersBuffer.setParameter((float) depthChannel.getImageWidth());
      parametersBuffer.setParameter((float) depthChannel.getImageHeight());
      parametersBuffer.setParameter((float) colorChannel.getImageWidth());
      parametersBuffer.setParameter((float) colorChannel.getImageHeight());
      parametersBuffer.setParameter(depthChannel.getDepthDiscretization());
      parametersBuffer.setParameter(useSensorColor);
      parametersBuffer.setParameter(gradientMode);
      parametersBuffer.setParameter(useSinusoidalGradientPattern);
      parametersBuffer.setParameter(pointSize);
      depthToWorldTransformParameter.setParameter(depthChannel.getTranslationToWorld(), depthChannel.getRotationMatrixToWorld());
      depthToColorTransformParameter.setParameter(colorChannel.getTranslationToWorld(), colorChannel.getRotationMatrixToWorld());

      // Upload the buffers to the OpenCL device (GPU)
      depthChannel.getDepth16UC1Image().writeOpenCLImage(openCLManager);
      if (useSensorColor)
         colorChannel.getColor8UC4Image().writeOpenCLImage(openCLManager);
      parametersBuffer.writeOpenCLBufferObject(openCLManager);
      depthToWorldTransformParameter.writeOpenCLBufferObject(openCLManager);
      depthToColorTransformParameter.writeOpenCLBufferObject(openCLManager);

      // Set the OpenCL kernel arguments
      openCLManager.setKernelArgument(kernel, 0, depthChannel.getDepth16UC1Image().getOpenCLImageObject());
      if (useSensorColor)
         openCLManager.setKernelArgument(kernel, 1, colorChannel.getColor8UC4Image().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 3, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 4, depthToWorldTransformParameter.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 5, depthToColorTransformParameter.getOpenCLBufferObject());

      // Run the OpenCL kernel
      openCLManager.execute2D(kernel, depthChannel.getImageWidth(), depthChannel.getImageHeight());

      // Read the OpenCL buffer back to the CPU
      pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);
   }
}
