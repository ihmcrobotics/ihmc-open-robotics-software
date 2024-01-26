package us.ihmc.perception;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.perception.opencl.OpenCLManager;

public class OpenCLDepthImageSegmenter
{
   private final OpenCLManager openCLManager;
   private final _cl_program openCLProgram;
   private final _cl_kernel kernel;
   private BytedecoImage bytedecoDepthImage;
   private BytedecoImage bytedecoMaskImage;
   private BytedecoImage bytedecoSegmentedDepth;

   public OpenCLDepthImageSegmenter(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
      openCLProgram = openCLManager.loadProgram("SegmentDepthImage", "PerceptionCommon.cl");
      kernel = openCLManager.createKernel(openCLProgram, "segmentDepthImage");
   }

   // TODO: Add mask to depth transform so we can use YOLO with other cameras besides ZED
   public RawImage removeBackground(RawImage depthImage, Mat imageMask)
   {
      depthImage.get();

      if (bytedecoDepthImage != null)
         bytedecoDepthImage.destroy(openCLManager);
      bytedecoDepthImage = new BytedecoImage(depthImage.getCpuImageMat());
      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoDepthImage.writeOpenCLImage(openCLManager);

      if (bytedecoMaskImage != null)
         bytedecoMaskImage.destroy(openCLManager);
      Mat resizedMask = new Mat(depthImage.getImageWidth(), depthImage.getImageHeight(), opencv_core.CV_8UC1);
      opencv_imgproc.resize(imageMask, resizedMask, new Size(depthImage.getImageWidth(), depthImage.getImageHeight()));
      bytedecoMaskImage = new BytedecoImage(resizedMask);
      bytedecoMaskImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoMaskImage.writeOpenCLImage(openCLManager);

      if (bytedecoSegmentedDepth != null)
         bytedecoSegmentedDepth.destroy(openCLManager);
      bytedecoSegmentedDepth = new BytedecoImage(depthImage.getImageWidth(), depthImage.getImageHeight(), depthImage.getOpenCVType());
      bytedecoSegmentedDepth.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);

      openCLManager.setKernelArgument(kernel, 0, bytedecoDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, bytedecoMaskImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 2, bytedecoSegmentedDepth.getOpenCLImageObject());

      openCLManager.execute2D(kernel, depthImage.getImageWidth(), depthImage.getImageHeight());

      bytedecoSegmentedDepth.readOpenCLImage(openCLManager);

      depthImage.release();
      resizedMask.release();

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
      openCLManager.destroy();
   }
}
