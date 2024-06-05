package us.ihmc.perception.opencl;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

public class OpenCLDepthImageSegmenter
{
   private final OpenCLManager openCLManager = new OpenCLManager();
   private final _cl_program openCLProgram = openCLManager.loadProgram("SegmentDepthImage", "PerceptionCommon.cl");
   private final _cl_kernel kernel = openCLManager.createKernel(openCLProgram, "segmentDepthImage");
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter depthToMaskTransformParameter = new OpenCLRigidBodyTransformParameter();

   private final MutableReferenceFrame depthFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame maskFrame = new MutableReferenceFrame();
   private final RigidBodyTransform depthToMaskTransform = new RigidBodyTransform();

   public RawImage removeBackground(RawImage depthImage, RawImage imageMask)
   {
      depthImage.get();
      imageMask.get();

      depthFrame.update(transformToWorld -> transformToWorld.set(depthImage.getOrientation(), depthImage.getPosition()));
      maskFrame.update(transformToWorld -> transformToWorld.set(imageMask.getOrientation(), imageMask.getPosition()));
      depthFrame.getReferenceFrame().getTransformToDesiredFrame(depthToMaskTransform, maskFrame.getReferenceFrame());
      depthToMaskTransformParameter.setParameter(depthToMaskTransform);
      depthToMaskTransformParameter.writeOpenCLBufferObject(openCLManager);

      parametersBuffer.setParameter(depthImage.getDepthDiscretization());
      parametersBuffer.setParameter(depthImage.getPrincipalPointX());
      parametersBuffer.setParameter(depthImage.getPrincipalPointY());
      parametersBuffer.setParameter(depthImage.getFocalLengthX());
      parametersBuffer.setParameter(depthImage.getFocalLengthY());
      parametersBuffer.setParameter(imageMask.getPrincipalPointX());
      parametersBuffer.setParameter(imageMask.getPrincipalPointY());
      parametersBuffer.setParameter(imageMask.getFocalLengthX());
      parametersBuffer.setParameter(imageMask.getFocalLengthY());
      parametersBuffer.writeOpenCLBufferObject(openCLManager);

      BytedecoImage bytedecoDepthImage = new BytedecoImage(depthImage.getCpuImageMat());
      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoDepthImage.writeOpenCLImage(openCLManager);

      BytedecoImage bytedecoMaskImage = new BytedecoImage(imageMask.getCpuImageMat());
      bytedecoMaskImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoMaskImage.writeOpenCLImage(openCLManager);

      BytedecoImage bytedecoSegmentedDepth = new BytedecoImage(depthImage.getImageWidth(), depthImage.getImageHeight(), depthImage.getOpenCVType());
      bytedecoSegmentedDepth.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);

      openCLManager.setKernelArgument(kernel, 0, bytedecoDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, bytedecoMaskImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 2, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 3, depthToMaskTransformParameter.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 4, bytedecoSegmentedDepth.getOpenCLImageObject());

      openCLManager.execute2D(kernel, depthImage.getImageWidth(), depthImage.getImageHeight());

      bytedecoSegmentedDepth.readOpenCLImage(openCLManager);
      Mat segmentedDepthImageMat = bytedecoSegmentedDepth.getBytedecoOpenCVMat().clone();

      bytedecoDepthImage.destroy(openCLManager);
      bytedecoMaskImage.destroy(openCLManager);
      bytedecoSegmentedDepth.destroy(openCLManager);
      depthImage.release();
      imageMask.release();

      return new RawImage(depthImage.getSequenceNumber(),
                          depthImage.getAcquisitionTime(),
                          depthImage.getImageWidth(),
                          depthImage.getImageHeight(),
                          depthImage.getDepthDiscretization(),
                          segmentedDepthImageMat,
                          null,
                          depthImage.getOpenCVType(),
                          depthImage.getFocalLengthX(),
                          depthImage.getFocalLengthY(),
                          depthImage.getPrincipalPointX(),
                          depthImage.getPrincipalPointY(),
                          depthImage.getPosition(),
                          depthImage.getOrientation());
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      kernel.close();
      openCLProgram.close();
      System.out.println("Destroyed " + getClass().getSimpleName());
   }
}