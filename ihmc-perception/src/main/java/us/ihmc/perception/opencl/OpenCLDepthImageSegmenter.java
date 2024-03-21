package us.ihmc.perception.opencl;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

public class OpenCLDepthImageSegmenter
{
   private final OpenCLManager openCLManager;
   private final _cl_program openCLProgram;
   private final _cl_kernel kernel;
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter depthToMaskTransformParameter = new OpenCLRigidBodyTransformParameter();
   private BytedecoImage bytedecoDepthImage;
   private BytedecoImage bytedecoMaskImage;
   private BytedecoImage bytedecoSegmentedDepth;

   private final MutableReferenceFrame depthFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame maskFrame = new MutableReferenceFrame();
   private final RigidBodyTransform depthToMaskTransform = new RigidBodyTransform();

   public OpenCLDepthImageSegmenter(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
      openCLProgram = openCLManager.loadProgram("SegmentDepthImage", "PerceptionCommon.cl");
      kernel = openCLManager.createKernel(openCLProgram, "segmentDepthImage");
   }

   // TODO: Add mask to depth transform so we can use YOLO with other cameras besides ZED
   public RawImage removeBackground(RawImage depthImage, RawImage imageMask, int erosionKernelRadius)
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

      Mat erodedMat = new Mat(imageMask.getImageHeight(), imageMask.getImageWidth(), imageMask.getOpenCVType());
      opencv_imgproc.erode(imageMask.getCpuImageMat(),
                           erodedMat,
                           opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                new Point(erosionKernelRadius, erosionKernelRadius)));

      if (bytedecoDepthImage != null)
         bytedecoDepthImage.destroy(openCLManager);
      bytedecoDepthImage = new BytedecoImage(depthImage.getCpuImageMat());
      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoDepthImage.writeOpenCLImage(openCLManager);

      if (bytedecoMaskImage != null)
         bytedecoMaskImage.destroy(openCLManager);
      bytedecoMaskImage = new BytedecoImage(erodedMat);
      bytedecoMaskImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoMaskImage.writeOpenCLImage(openCLManager);

      if (bytedecoSegmentedDepth != null)
         bytedecoSegmentedDepth.destroy(openCLManager);
      bytedecoSegmentedDepth = new BytedecoImage(depthImage.getImageWidth(), depthImage.getImageHeight(), depthImage.getOpenCVType());
      bytedecoSegmentedDepth.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);

      openCLManager.setKernelArgument(kernel, 0, bytedecoDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, bytedecoMaskImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 2, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 3, depthToMaskTransformParameter.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 4, bytedecoSegmentedDepth.getOpenCLImageObject());

      openCLManager.execute2D(kernel, depthImage.getImageWidth(), depthImage.getImageHeight());

      bytedecoSegmentedDepth.readOpenCLImage(openCLManager);

      erodedMat.release();
      depthImage.release();
      imageMask.release();
      return new RawImage(depthImage.getSequenceNumber(),
                          depthImage.getAcquisitionTime(),
                          depthImage.getImageWidth(),
                          depthImage.getImageHeight(),
                          depthImage.getDepthDiscretization(),
                          bytedecoSegmentedDepth.getBytedecoOpenCVMat(),
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
      if (bytedecoDepthImage != null)
         bytedecoDepthImage.destroy(openCLManager);
      if (bytedecoMaskImage != null)
         bytedecoMaskImage.destroy(openCLManager);
      if (bytedecoSegmentedDepth != null)
         bytedecoSegmentedDepth.destroy(openCLManager);

      openCLProgram.close();
      kernel.close();
   }
}