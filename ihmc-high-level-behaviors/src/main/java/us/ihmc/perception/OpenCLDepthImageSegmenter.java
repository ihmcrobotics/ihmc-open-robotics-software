package us.ihmc.perception;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
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
   public RawImage removeBackground(RawImage depthImage, Mat imageMask, int dilationSize)
   {
      depthImage.get();
      Mat segmentMat = depthImage.getCpuImageMat();

      opencv_imgproc.erode(imageMask,
                           imageMask,
                           opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                new Size(2 * dilationSize + 1, 2 * dilationSize + 1),
                                                                new Point(dilationSize, dilationSize)));

      // Stupid amount of checking to avoid errors
      if (segmentMat != null && !segmentMat.isNull() && !segmentMat.empty())
      {

         if (bytedecoDepthImage != null)
            bytedecoDepthImage.destroy(openCLManager);
         bytedecoDepthImage = new BytedecoImage(segmentMat);
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
         segmentMat = bytedecoSegmentedDepth.getBytedecoOpenCVMat();

         resizedMask.release();
      }

      depthImage.release();
      return new RawImage(depthImage.getSequenceNumber(),
                          depthImage.getAcquisitionTime(),
                          depthImage.getImageWidth(),
                          depthImage.getImageHeight(),
                          depthImage.getDepthDiscretization(),
                          segmentMat,
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
