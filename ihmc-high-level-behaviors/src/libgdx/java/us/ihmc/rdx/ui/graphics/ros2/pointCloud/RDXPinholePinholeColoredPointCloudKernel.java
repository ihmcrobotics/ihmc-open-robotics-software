package us.ihmc.rdx.ui.graphics.ros2.pointCloud;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLRigidBodyTransformParameter;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

/**
 * This class is for rendering colored point clouds when the depth camera
 * and the color camera are both pinhole model.
 */
public class RDXPinholePinholeColoredPointCloudKernel
{
   private final OpenCLManager openCLManager;
   private final _cl_program openCLProgram;
   private final _cl_kernel kernel;
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter depthToWorldTransformParameter = new OpenCLRigidBodyTransformParameter();
   private final OpenCLRigidBodyTransformParameter depthToColorTransformParameter = new OpenCLRigidBodyTransformParameter();
   private final BytedecoImage placeholderColorImage;
   private final MutableReferenceFrame depthFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame colorFrame = new MutableReferenceFrame();
   private final RigidBodyTransform depthToColorTransform = new RigidBodyTransform();


   public RDXPinholePinholeColoredPointCloudKernel(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
      openCLProgram = openCLManager.loadProgram("PinholePinholeColoredPointCloud", "PerceptionCommon.cl");
      kernel = openCLManager.createKernel(openCLProgram, "computeVertexBuffer");
      placeholderColorImage = new BytedecoImage(1, 1, opencv_core.CV_8UC4);
      placeholderColorImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
   }

   public void computeVertexBuffer(RDXROS2ColoredPointCloudVisualizerColorChannel colorChannel,
                                   RDXROS2ColoredPointCloudVisualizerDepthChannel depthChannel,
                                   boolean useSensorColor,
                                   int gradientMode,
                                   boolean useSinusoidalGradientPattern,
                                   float pointSize,
                                   OpenCLFloatBuffer pointCloudVertexBuffer)
   {
      depthFrame.update(transformToWorld -> transformToWorld.set(depthChannel.getRotationMatrixToWorld(), depthChannel.getTranslationToWorld()));
      colorFrame.update(transformToWorld -> transformToWorld.set(colorChannel.getRotationMatrixToWorld(), colorChannel.getTranslationToWorld()));
      depthFrame.getReferenceFrame().getTransformToDesiredFrame(depthToColorTransform, colorFrame.getReferenceFrame());

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
      depthToColorTransformParameter.setParameter(depthToColorTransform.getTranslation(), depthToColorTransform.getRotation());

      // Upload the buffers to the OpenCL device (GPU)
      depthChannel.getDepth16UC1Image().writeOpenCLImage(openCLManager);
      // It appears you've got to write something to the OpenCL argument even if you don't use it,
      // so we write a placeholder image.
      BytedecoImage colorImage = useSensorColor ? colorChannel.getColor8UC4Image() : placeholderColorImage;
      colorImage.writeOpenCLImage(openCLManager);
      parametersBuffer.writeOpenCLBufferObject(openCLManager);
      depthToWorldTransformParameter.writeOpenCLBufferObject(openCLManager);
      depthToColorTransformParameter.writeOpenCLBufferObject(openCLManager);

      // Set the OpenCL kernel arguments
      openCLManager.setKernelArgument(kernel, 0, depthChannel.getDepth16UC1Image().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, colorImage.getOpenCLImageObject());
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
